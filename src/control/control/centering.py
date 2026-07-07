#!/usr/bin/env python3
"""Task-aware centering framework for the AUV.

This is the future-proof home for "how the sub centers on a target and knows
when/where to stop." It is deliberately ROS-free (pure data + math) so it can be
unit-tested offline and so a future world-frame mapping source can feed it
without touching the controller.

Layers
------
1. ``TargetState``  — frame-agnostic snapshot of where the target is right now.
   Carries BOTH image-space (cx, cy, bbox) and metric (range_m + derived
   lateral_m/vertical_m/forward_m) fields, plus a ``frame`` tag ('camera'
   today; 'world' when a mapping node feeds it later). This is the seam for a
   future OSU ``riptide_mapping``-style world-frame object map.
2. ``TargetTracker`` — per-label filtered state: exponential moving average on
   the raw signals + short "coast" persistence through dropped frames so one
   missed detection does not slam the controller to a stop. This is the
   robustness layer on top of reactive per-frame centering.
3. ``CenteringPolicy`` — per-task config: desired standoff, lateral/vertical
   offsets, tolerances, active axes, convergence confirmation. Subclass per
   task (``GatePolicy`` implemented; torpedo/bin policies are stubs that return
   sensible defaults so the controller works for every label today).

The controller (``autonomous_controller.py``) wires these together: each
``track_object`` tick it (a) feeds the freshest detection into the tracker,
(b) reads the filtered ``TargetState``, (c) computes body-frame errors per the
active policy via ``centering_errors()``, and (d) dispatches a simultaneous
4-axis setpoint (surge/strafe/heave/yaw_rate).

Why reactive (no world map) today?
  VSLAM is intentionally disabled for competition (see
  ``src/localization/launch/vslam_localization_launch.py``), so a world-frame
  object map built on unreliable localization would be LESS robust than
  reactive per-frame centering. ``TargetState.frame`` is the seam: when
  localization is reliable, a mapping node publishes ``TargetState`` with
  ``frame='world'`` and the SAME policies/controller work unchanged.

Distance source
  Each ZED camera already solves the 3D pose of every detected object (ZED SDK
  custom-box object tracking) and publishes the slant range as
  ``ObjectDetection.position.z``. That is OSU's "stereo → depth → distance"
  step — already done. We use that range for the metric standoff + stop
  condition, and fall back to bounding-box size when range is unavailable
  (2D-only / NEURAL depth disabled).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Protocol


# Structural type for any detection carrying the fields we read. Matches
# auv_msgs/ObjectDetection without importing it (keeps this module ROS-free
# and unit-testable).
class _PointLike(Protocol):
    x: float
    y: float
    z: float


class DetectionLike(Protocol):
    label: str
    confidence: float
    bbox_width: float
    bbox_height: float
    position: _PointLike


# ─────────────────────────────────────────────────────────────────────────────
# Target state
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TargetState:
    """Frame-agnostic snapshot of a tracked target's current pose.

    Image-space fields come straight from the detector (normalized [0,1]).
    Metric fields are derived on demand via :meth:`metric_offsets` using the
    camera FOV (convention-independent — does not depend on the ZED's
    configured ``sl.COORDINATE_SYSTEM``, only on bearing + range + FOV).
    """

    label: str
    cx: float            # normalized image x [0,1], +right
    cy: float            # normalized image y [0,1], +down
    range_m: float       # slant range to target (m), -1 if unknown
    bbox_w: float        # normalized width  [0,1]
    bbox_h: float        # normalized height [0,1]
    confidence: float
    stamp: float         # monotonic seconds
    frame: str = 'camera'   # 'camera' (reactive) or 'world' (future map)

    def metric_offsets(self, hfov_rad: float, vfov_rad: float):
        """Body-frame metric offsets at the target's range.

        Returns ``(lateral_m, vertical_m, forward_m)`` in metres:
          lateral_m  — +right of the optical axis
          vertical_m — +below the optical axis (down)
          forward_m  — straight-ahead range (slant range, good approximation
                       for centered targets)

        Derived from image bearing + range via pinhole unprojection, so it is
        correct regardless of the ZED coordinate convention. Returns ``None``
        when range is unknown — the caller falls back to image-space control.
        """
        r = self.range_m
        if r is None or r <= 0.0:
            return None
        lateral = (self.cx - 0.5) * 2.0 * math.tan(hfov_rad * 0.5) * r
        vertical = (self.cy - 0.5) * 2.0 * math.tan(vfov_rad * 0.5) * r
        return (lateral, vertical, r)


# ─────────────────────────────────────────────────────────────────────────────
# Target tracker (filtering + coast-through-dropouts)
# ─────────────────────────────────────────────────────────────────────────────

def _ema(prev: float, new: float, alpha: float) -> float:
    """Exponential moving average: ``alpha`` is the weight on the new sample."""
    return prev + alpha * (new - prev)


class TargetTracker:
    """Per-label filtered target state with short persistence through dropouts.

    - EMA-smooths cx/cy/bbox/range so a single jittery detection does not
      spike the controller.
    - "Coasts": if no fresh detection arrives within ``coast_s``, the state
      expires and the controller holds instead of acting on stale data.
    - Range is only smoothed when the new frame has a valid range; otherwise
      the last known range is retained (and ages out with the coast).
    """

    def __init__(self, alpha: float = 0.3, coast_s: float = 0.6,
                 stale_s: float = 2.0):
        self.alpha = max(0.01, min(1.0, alpha))
        self.coast_s = max(0.0, coast_s)
        self.stale_s = max(coast_s, stale_s)
        self._state: Optional[TargetState] = None
        self._last_update: Optional[float] = None

    def reset(self) -> None:
        self._state = None
        self._last_update = None

    def update(self, det: DetectionLike, now: float,
               label: Optional[str] = None) -> None:
        """Feed one raw ObjectDetection into the tracker.

        ``det.position.x/.y`` are normalized image coords; ``det.position.z``
        is the slant range in metres (-1 if unknown). ``now`` is monotonic
        seconds. A malformed frame is skipped (the last good state coasts).
        """
        try:
            cx = float(det.position.x)
            cy = float(det.position.y)
            rng = float(det.position.z)
            bw = float(det.bbox_width)
            bh = float(det.bbox_height)
            conf = float(det.confidence)
        except (AttributeError, ValueError, TypeError):
            # Malformed detection — skip this frame; the coast window keeps
            # the controller acting on the last good state until it expires.
            return
        lab = label or getattr(det, 'label', '')

        s = self._state
        if (s is None or self._last_update is None
                or (now - self._last_update) > self.stale_s):
            # First sighting, or stale enough that EMA would smear: hard init.
            self._state = TargetState(lab, cx, cy, rng, bw, bh, conf, now)
        else:
            a = self.alpha
            prev_range = s.range_m if s.range_m > 0 else rng
            nrng = _ema(prev_range, rng, a) if rng > 0 else s.range_m
            self._state = TargetState(
                lab,
                _ema(s.cx, cx, a),
                _ema(s.cy, cy, a),
                nrng,
                _ema(s.bbox_w, bw, a),
                _ema(s.bbox_h, bh, a),
                conf,
                now,
            )
        self._last_update = now

    def state(self, now: float) -> Optional[TargetState]:
        """Filtered state if within the coast window, else ``None``."""
        if self._state is None or self._last_update is None:
            return None
        if (now - self._last_update) > self.coast_s:
            return None
        return self._state


# ─────────────────────────────────────────────────────────────────────────────
# Centering policies (per-task)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CenteringPolicy:
    """Per-task centering configuration.

    Subclass (or override) to specialise a task. All distances are metres
    unless noted. The controller reads these to shape the control law + decide
    convergence (the "when to stop" + "where").
    """

    label: str = ''
    # Where to stop (the standoff): metric forward range (m) when range is
    # available, else the bbox-width fraction at which we consider ourselves
    # "in range" (2D-only fallback).
    standoff_m: float = 1.0
    standoff_bbox: float = 0.30
    # Desired body-frame offset from the target centre (m). 0 = on the
    # centerline; e.g. a gate-side pass sets lateral_offset_m to ±0.3.
    lateral_offset_m: float = 0.0
    vertical_offset_m: float = 0.0
    # Convergence tolerances (the "when is it centered enough" thresholds).
    tol_cx: float = 0.06           # image-space (fallback mode)
    tol_cy: float = 0.06
    tol_lateral_m: float = 0.15    # metric (preferred mode)
    tol_vertical_m: float = 0.15
    tol_range_m: float = 0.30
    # Approach speed shaping.
    approach_speed: float = 0.30  # max surge effort while closing
    min_speed: float = 0.12        # floor so the sub does not stall out
    surge_gain: float = 0.25       # remaining(m) * gain → surge effort
    # How many consecutive ticks within tolerance before declaring converged.
    converge_ticks: int = 3
    # Active axes (let a task opt out of e.g. surge for a pure-aim task).
    use_yaw: bool = True
    use_strafe: bool = True
    use_depth: bool = True
    use_surge: bool = True


@dataclass
class GatePolicy(CenteringPolicy):
    """Gate: center on the gate centreline, approach to a transit standoff.

    The gate is wide and the sub transits straight through, so the goal is to
    be ON the gate's centerline (strafe to null lateral offset), FACING it
    (yaw to null bearing), at the right depth (vertical), at the transit
    standoff. A separate BT step (ForwardTransit) then pushes through.
    """

    label: str = 'gate'
    standoff_m: float = 1.5
    standoff_bbox: float = 0.32
    tol_lateral_m: float = 0.15
    tol_vertical_m: float = 0.15
    tol_range_m: float = 0.30
    approach_speed: float = 0.30
    min_speed: float = 0.12
    surge_gain: float = 0.25


@dataclass
class DefaultPolicy(CenteringPolicy):
    """Generic fallback: approach to a standoff and center. Used for any label
    without a dedicated policy, and as the base behaviour for the torpedo/bin
    stubs until their task-specific policies are fleshed out."""

    label: str = ''
    standoff_m: float = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# Policy registry — add a task here to specialise its centering.
# ─────────────────────────────────────────────────────────────────────────────

def policy_for(label: str, approach_dist: float = 0.0) -> CenteringPolicy:
    """Return the centering policy for a target label.

    ``approach_dist`` (from NavigationCommand) overrides the policy's
    ``standoff_m`` when > 0, so a BT call site can dial in the desired standoff
    per task without a new policy subclass.
    """
    lab = (label or '').lower().strip()

    if lab == 'gate':
        p: CenteringPolicy = GatePolicy()
    elif lab in ('large_opening', 'small_opening'):
        # Torpedo: aim at the SPECIFIC opening from a firing standoff, tighter
        # tolerances than a gate. Stub until TorpedoPolicy is wired.
        p = DefaultPolicy(label=lab, standoff_m=1.2, standoff_bbox=0.18,
                          tol_cx=0.04, tol_cy=0.04,
                          tol_lateral_m=0.08, tol_vertical_m=0.08,
                          tol_range_m=0.20,
                          approach_speed=0.18, min_speed=0.08,
                          surge_gain=0.20)
    elif lab in ('bin1', 'bin2'):
        # Bin: approach to a hover standoff above the bin centre. Top-down
        # centering via the bottom camera is a separate geometry handled
        # elsewhere; this is the front-facing approach stub.
        p = DefaultPolicy(label=lab, standoff_m=0.8, standoff_bbox=0.25,
                          approach_speed=0.18, min_speed=0.08)
    else:
        p = DefaultPolicy(label=lab, standoff_m=1.0)

    try:
        ad = float(approach_dist)
    except (TypeError, ValueError):
        ad = 0.0
    if ad > 0.0:
        p.standoff_m = ad
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Control-error computation (pure; the controller adds PIDs)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CenteringErrors:
    """Body-frame centering errors + convergence flags for one tick."""

    yaw_err: float        # + → target is to the right → turn CW
    strafe_err: float     # + → target is to the right → strafe right
    depth_err: float      # + → target is below → submerge (heave down)
    surge_remaining: float  # + → still need to close range (m, or bbox-frac)
    metric: bool          # True = range available (metric mode)
    in_range: bool        # True = within range tolerance
    centered: bool        # True = lateral + vertical + range all within tol


def centering_errors(policy: CenteringPolicy, state: TargetState,
                     hfov_rad: float, vfov_rad: float) -> CenteringErrors:
    """Compute body-frame centering errors for one filtered target state.

    Two modes:
      * Metric (range available): derive lateral/vertical/forward from bearing
        + range + FOV. Yaw error is the bearing angle to the desired lateral
        position; strafe/depth errors are the metric offsets.
      * Fallback (no range): drive directly from normalized cx/cy and use
        bbox-width as the range proxy.
    """
    m = state.metric_offsets(hfov_rad, vfov_rad)
    if m is not None:
        lateral, vertical, fwd = m
        yaw_err = math.atan2(lateral - policy.lateral_offset_m,
                             max(fwd, 0.3))
        strafe_err = lateral - policy.lateral_offset_m
        depth_err = vertical - policy.vertical_offset_m
        surge_remaining = max(0.0, fwd - policy.standoff_m)
        in_range = abs(fwd - policy.standoff_m) < policy.tol_range_m
        lat_ok = abs(strafe_err) < policy.tol_lateral_m
        vert_ok = abs(depth_err) < policy.tol_vertical_m
        metric = True
    else:
        ex = state.cx - 0.5
        ey = state.cy - 0.5
        yaw_err = ex
        strafe_err = ex
        depth_err = ey
        surge_remaining = max(0.0, policy.standoff_bbox - state.bbox_w)
        in_range = state.bbox_w >= policy.standoff_bbox
        lat_ok = abs(ex) < policy.tol_cx
        vert_ok = abs(ey) < policy.tol_cy
        metric = False

    return CenteringErrors(
        yaw_err=yaw_err,
        strafe_err=strafe_err,
        depth_err=depth_err,
        surge_remaining=surge_remaining,
        metric=metric,
        in_range=in_range,
        centered=lat_ok and vert_ok and in_range,
    )


def shape_surge(policy: CenteringPolicy, errs: CenteringErrors) -> float:
    """Map the remaining range to a forward surge effort in [0, approach_speed].

    Returns 0 when the sub is in range or surge is disabled. In metric mode
    the gain is per-metre; in fallback mode it is per bbox-fraction (×3).
    """
    if not policy.use_surge or errs.in_range:
        return 0.0
    gain = policy.surge_gain if errs.metric else 3.0
    raw = errs.surge_remaining * gain
    return max(policy.min_speed, min(policy.approach_speed, raw))


def clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))
