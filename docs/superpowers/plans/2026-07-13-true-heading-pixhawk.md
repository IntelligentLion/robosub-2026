# True Vehicle Heading (Backward-Mounted Pixhawk) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One canonical, shared definition of "the direction the sub is actually facing" relative to the Pixhawk's reported attitude — so every script stops hand-rolling the backward-mount 180° and the `-x`-is-forward sign, and gains a drift-corrected heading when the ZED is healthy.

**Architecture:** The Pixhawk is physically mounted facing BACKWARD (`AHRS_ORIENTATION` untouched at 0), so `ATTITUDE.yaw` is vehicle-heading + 180°, and vehicle-forward is MANUAL_CONTROL `-x`. Yaw is gyro-only (`EK3_SRC1_YAW=0`, compass hard-iron unusable) → it drifts slowly and has an arbitrary power-on origin, so "true" heading is only meaningful (a) relative to a captured reference (lined up on the gate at dive) or (b) fused against ZED visual yaw. New root module `heading_common.py` owns: the 180° offset constant, the forward-x sign, reference capture, and a ZED↔Pixhawk fusion estimator. Pure math, fully unit-testable; scripts adopt it afterwards.

**Why not `AHRS_ORIENTATION=4` (Yaw180) instead:** that re-frames the ENTIRE vehicle for the EKF *and* the mixer's notion of front — every motor-table assumption, the all-8 param preflight expectations, the `-x` convention in every script, and the trim state would need simultaneous re-validation in water. Mid-season that's a rollback-hostile flag day. It stays the documented endgame (one QGC change + one sweep of `FORWARD_X_SIGN`), and this plan's single-constant design is exactly what makes that flip cheap later.

**Tech Stack:** Python 3.10 / pymavlink 2.4.49 / ArduSub 4.5.7 / pytest.

## Global Constraints

- This machine IS the vehicle. Never arm the Pixhawk executing this plan; **WATER TEST** steps are for humans.
- Do NOT write FC parameters anywhere in this plan (read-only param policy). `AHRS_ORIENTATION` stays 0.
- ZED heading convention: `vslam/odometry` is RIGHT_HANDED_Y_UP; heading is about **Y** (`field_common.heading_about_axis`, `VERTICAL_AXIS='y'`).
- ATTITUDE.yaw sign is unaffected by the backward mount (board z still points the same way): positive yaw rate = clockwise from above = right turn. The mount shifts yaw by π; it does not mirror it.
- Run pytest as: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/ -v`.
- Commit after every task, conventional style, end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `heading_common.py` (new, repo root) | offset constant, `vehicle_yaw`, `forward_x`, `HeadingRef`, `HeadingFusion` | 1, 2 |
| `tests/test_heading_common.py` (new) | unit tests, pure math | 1, 2 |
| `submerge_forward_10ft.py`, `submerge_forward.py`, `diagnose_forward_veer.py` | consume `forward_x` (kill hand-rolled `-x`) | 3 |
| `gate_task.py`, `run_course.py` | reference capture at dive / gate line-up | 3 |
| `docs/water-tests/2026-true-heading.md` (new) | pool calibration + verification procedure | 4 |

---

### Task 1: Core mapping — offset, forward sign, reference capture

**Files:**
- Create: `heading_common.py`
- Test: `tests/test_heading_common.py`

**Interfaces:**
- Produces: `PIXHAWK_YAW_OFFSET_RAD: float` (= math.pi), `FORWARD_X_SIGN: int` (= -1), `wrap(a) -> float` ([-π, π]), `vehicle_yaw(pixhawk_yaw) -> float`, `forward_x(effort: 0..1) -> int` (MANUAL_CONTROL x units), `HeadingRef` with `.capture(vehicle_yaw_rad)`, `.captured: bool`, `.relative(vehicle_yaw_rad) -> float` (signed error, +ve = vehicle is clockwise/right of reference).

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_heading_common.py
import math

import pytest

import heading_common as hc


def test_vehicle_yaw_is_pixhawk_plus_pi():
    assert hc.vehicle_yaw(0.0) == pytest.approx(math.pi)
    assert hc.vehicle_yaw(math.pi) == pytest.approx(0.0, abs=1e-9)
    # wraps: pixhawk +170deg -> vehicle -10deg, not +350
    assert hc.vehicle_yaw(math.radians(170)) == pytest.approx(
        math.radians(-10), abs=1e-9)


def test_forward_x_sign_and_scale():
    # Pixhawk mounted backward: vehicle-forward is autopilot -x
    assert hc.FORWARD_X_SIGN == -1
    assert hc.forward_x(0.7) == -700
    assert hc.forward_x(0.0) == 0
    assert hc.forward_x(1.5) == -1000     # clamped


def test_heading_ref_relative():
    r = hc.HeadingRef()
    assert r.captured is False
    r.capture(math.radians(10))
    assert r.captured is True
    # turned right by 30deg -> relative +30
    assert r.relative(math.radians(40)) == pytest.approx(
        math.radians(30), abs=1e-9)
    # wrap: reference +170, now -170 -> +20 right, not -340
    r.capture(math.radians(170))
    assert r.relative(math.radians(-170)) == pytest.approx(
        math.radians(20), abs=1e-9)


def test_heading_ref_before_capture_raises():
    with pytest.raises(RuntimeError):
        hc.HeadingRef().relative(0.0)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_heading_common.py -v`
Expected: FAIL — `ModuleNotFoundError: heading_common`.

- [ ] **Step 3: Implement**

```python
#!/usr/bin/env python3
"""Single source of truth for vehicle heading vs the backward Pixhawk.

The Pixhawk is mounted facing BACKWARD (AHRS_ORIENTATION deliberately left
at 0 — flipping it re-frames the mixer + preflight expectations, see the
true-heading plan). Consequences, encoded HERE and nowhere else:

  * ATTITUDE.yaw  = vehicle heading + pi      -> vehicle_yaw()
  * vehicle-forward = MANUAL_CONTROL -x       -> forward_x(), FORWARD_X_SIGN

Yaw is gyro-only (EK3_SRC1_YAW=0; the compass hard-iron chase of 2026-07-09
made the EKF blind to real rotation) — so yaw has an ARBITRARY origin and
slow drift. Absolute heading therefore means "relative to a captured
reference" (HeadingRef, e.g. lined up on the gate at dive) or "fused with
ZED visual yaw" (HeadingFusion).

If the team ever sets AHRS_ORIENTATION=4 (Yaw180) in QGC: set
PIXHAWK_YAW_OFFSET_RAD = 0.0 and FORWARD_X_SIGN = +1 here, re-verify the
all-8 thruster preflight in water, and nothing else should change.
"""

import math
import time
from collections import deque

PIXHAWK_YAW_OFFSET_RAD = math.pi   # board yaw -> vehicle yaw
FORWARD_X_SIGN = -1                # vehicle-forward on the MANUAL_CONTROL x


def wrap(a):
    """Wrap radians to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def vehicle_yaw(pixhawk_yaw):
    """ATTITUDE.yaw -> the direction the sub's NOSE actually points."""
    return wrap(pixhawk_yaw + PIXHAWK_YAW_OFFSET_RAD)


def forward_x(effort):
    """Effort fraction 0..1 -> MANUAL_CONTROL x that drives the sub
    FORWARD (its nose direction), backward mount folded in."""
    effort = min(max(effort, 0.0), 1.0)
    return FORWARD_X_SIGN * int(round(effort * 1000))


class HeadingRef:
    """Heading relative to a physically-known reference (e.g. pointed at
    the gate right before the dive). Positive relative() = vehicle has
    rotated clockwise/right of the reference."""

    def __init__(self):
        self._ref = None

    @property
    def captured(self):
        return self._ref is not None

    def capture(self, vehicle_yaw_rad):
        self._ref = vehicle_yaw_rad

    def relative(self, vehicle_yaw_rad):
        if self._ref is None:
            raise RuntimeError('HeadingRef.relative before capture()')
        return wrap(vehicle_yaw_rad - self._ref)
```

- [ ] **Step 4: Run tests** — 4 PASS; full suite green.

- [ ] **Step 5: Commit**

```bash
git add heading_common.py tests/test_heading_common.py
git commit -m "feat(heading): heading_common — backward-mount offset, forward sign, HeadingRef"
```

---

### Task 2: HeadingFusion — ZED-corrected drift-free heading

**Files:**
- Modify: `heading_common.py`
- Test: `tests/test_heading_common.py`

**Interfaces:**
- Consumes: `wrap`, `vehicle_yaw`.
- Produces: `HeadingFusion(window_s=20.0, zed_stale_s=1.0)` with `update_pix(pixhawk_yaw, t)`, `update_zed(zed_heading, t)`, `heading(t) -> float | None` (drift-corrected vehicle yaw; falls back to gyro-only when ZED stale; None before any pix sample), `.zed_healthy(t) -> bool`.

The estimator: ZED visual yaw is drift-corrected but lives in its own frame; Pixhawk yaw is continuous but drifts. Estimate the (slowly-varying) offset `zed_heading - vehicle_yaw(pix)` as a windowed circular mean; fused heading = `vehicle_yaw(pix) + offset` re-expressed in the ZED frame's stable orientation. When ZED is stale (>1 s, matches `field_common.ZED_STALE_S`), hold the last offset — gyro drift then accrues at its slow rate instead of a step.

- [ ] **Step 1: Write the failing tests**

```python
def test_fusion_none_before_data():
    f = hc.HeadingFusion()
    assert f.heading(0.0) is None


def test_fusion_corrects_linear_gyro_drift():
    f = hc.HeadingFusion(window_s=20.0)
    # vehicle truly still: ZED says 0.5 rad the whole time; pixhawk
    # vehicle-yaw drifts +0.002 rad/s from 0.3 rad
    for i in range(100):
        t = i * 0.1
        pix_vehicle = 0.3 + 0.002 * t
        f.update_pix(hc.wrap(pix_vehicle - math.pi), t)   # board frame
        f.update_zed(0.5, t)
    # fused heading pinned near the ZED-stable value
    assert f.heading(10.0) == pytest.approx(0.5, abs=0.01)


def test_fusion_falls_back_when_zed_stale():
    f = hc.HeadingFusion(zed_stale_s=1.0)
    f.update_pix(hc.wrap(0.3 - math.pi), 0.0)
    f.update_zed(0.5, 0.0)
    assert f.zed_healthy(0.5) is True
    assert f.zed_healthy(2.0) is False
    # offset held: heading still defined, tracks gyro from here
    f.update_pix(hc.wrap(0.4 - math.pi), 2.0)
    assert f.heading(2.0) == pytest.approx(0.5 + 0.1, abs=1e-6)


def test_fusion_offset_wraps():
    f = hc.HeadingFusion()
    f.update_pix(hc.wrap(math.radians(179) - math.pi), 0.0)
    f.update_zed(math.radians(-179), 0.0)
    f.update_pix(hc.wrap(math.radians(179) - math.pi), 0.1)
    h = f.heading(0.1)
    assert h == pytest.approx(math.radians(-179), abs=1e-3)
```

- [ ] **Step 2: Run to verify failure** — missing `HeadingFusion`.

- [ ] **Step 3: Implement**

```python
class HeadingFusion:
    """Drift-corrected vehicle heading: gyro continuity + ZED stability.

    offset := circular-mean over window_s of (zed_heading - vehicle_yaw).
    heading := vehicle_yaw(latest pix) + offset. ZED stale -> freeze the
    offset (drift resumes at gyro rate, no step). A ZED tracking RESET
    (drift-reset plan) steps the ZED frame: the windowed mean re-converges
    within window_s; callers needing instant re-reference call reset().
    """

    def __init__(self, window_s=20.0, zed_stale_s=1.0):
        self.window_s = window_s
        self.zed_stale_s = zed_stale_s
        self._pairs = deque()           # (t, sin(off), cos(off))
        self._last_pix = None           # (t, vehicle_yaw)
        self._last_zed = None           # (t, heading)
        self._frozen_offset = None

    def reset(self):
        self._pairs.clear()
        self._frozen_offset = None

    def update_pix(self, pixhawk_yaw, t):
        self._last_pix = (t, vehicle_yaw(pixhawk_yaw))
        self._maybe_pair(t)

    def update_zed(self, zed_heading, t):
        self._last_zed = (t, zed_heading)
        self._maybe_pair(t)

    def zed_healthy(self, t):
        return (self._last_zed is not None
                and t - self._last_zed[0] <= self.zed_stale_s)

    def _maybe_pair(self, t):
        if self._last_pix is None or self._last_zed is None:
            return
        if abs(self._last_pix[0] - self._last_zed[0]) > self.zed_stale_s:
            return
        off = wrap(self._last_zed[1] - self._last_pix[1])
        self._pairs.append((t, math.sin(off), math.cos(off)))
        while self._pairs and t - self._pairs[0][0] > self.window_s:
            self._pairs.popleft()

    def _offset(self, t):
        if self.zed_healthy(t) and self._pairs:
            s = sum(p[1] for p in self._pairs)
            c = sum(p[2] for p in self._pairs)
            self._frozen_offset = math.atan2(s, c)
        return self._frozen_offset

    def heading(self, t):
        if self._last_pix is None:
            return None
        off = self._offset(t)
        if off is None:
            return self._last_pix[1]     # no ZED yet: raw gyro heading
        return wrap(self._last_pix[1] + off)
```

- [ ] **Step 4: Run tests** — all PASS; full suite green.

- [ ] **Step 5: Commit**

```bash
git add heading_common.py tests/test_heading_common.py
git commit -m "feat(heading): HeadingFusion — ZED-corrected gyro heading with stale fallback"
```

---

### Task 3: Adopt across scripts (kill hand-rolled signs)

**Files:**
- Modify: `submerge_forward_10ft.py` (x_cmd at ~line 266-269), `submerge_forward.py`, `diagnose_forward_veer.py` (`surge_phase` x_full), `gate_task.py`, `run_course.py`

- [ ] **Step 1: Sweep for hand-rolled conventions**

Run: `grep -rn -- "-int(.*1000)\|x=-\|yaw + math.pi\|yaw+math.pi" --include=*.py . | grep -v venv | grep -v install | grep -v tests`
Every hit that encodes "forward is -x" or "+π heading fix" migrates to `heading_common`.

- [ ] **Step 2: Migrate the surge scripts**

`submerge_forward_10ft.py`: add `import heading_common as hc`; replace `x_cmd=-int(args.forward_effort * 1000)` with `x_cmd=hc.forward_x(args.forward_effort)`; shrink the "Pixhawk is mounted facing backward" comments to a pointer: `# forward sign lives in heading_common`. Same in `submerge_forward.py`. In `diagnose_forward_veer.py.surge_phase`: `x_full = hc.forward_x(effort)`.

- [ ] **Step 3: Reference capture at the gate**

In `gate_task.py` / `run_course.py`, at the moment the sub is known to be lined up on the gate (pre-dive or post-centering — read the file for the exact hook), instantiate once and capture:

```python
gate_ref = hc.HeadingRef()
# ... at line-up, with the latest ATTITUDE.yaw in hand:
gate_ref.capture(hc.vehicle_yaw(attitude_yaw))
# ... later, to face the gate again:
err = gate_ref.relative(hc.vehicle_yaw(attitude_yaw))  # +ve = turned right
```

Where the scripts consume ZED heading (`field_common.HeadingMonitor`), leave them on ZED — `HeadingRef` over `vehicle_yaw` is the Pixhawk-side equivalent for when the ZED fix is down.

- [ ] **Step 4: Verify** — full pytest suite; import-check every touched script (`python3 -c "import gate_task"` etc.); `python3 diagnose_forward_veer.py --help` clean.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor(heading): scripts consume heading_common, no hand-rolled signs"
```

---

### Task 4: Water calibration + verification procedure (document only)

**Files:**
- Create: `docs/water-tests/2026-true-heading.md`

- [ ] **Step 1: Write it**

```markdown
# True-heading water verification

1. Sanity (dry, on the bench, FC on): run
   `python3 - <<'EOF'` snippet printing `hc.vehicle_yaw(ATTITUDE.yaw)` at
   1 Hz. Point the sub's NOSE at a landmark; rotate the sub 90° clockwise
   by hand: printed vehicle yaw must INCREASE by ~+90° (wrap allowed).
   If it decreases, the offset sign is wrong — stop, fix heading_common.
2. Reference drill (water): line up on the gate, capture HeadingRef, drive
   a lap of the pool manually, ask for `relative()`: it must read ~0 when
   visually re-aligned to the gate. Error >10° after ~5 min = gyro drift
   scale; note the °/min figure in this doc.
3. Fusion drill: same lap with the detector running (ZED healthy).
   HeadingFusion.heading() error re-aligned to gate should be <3° and
   NOT grow with time. Kill the detector mid-lap: heading keeps working,
   drift resumes at the gyro rate from step 2.
```

- [ ] **Step 2: Commit**

```bash
git add docs/water-tests/2026-true-heading.md
git commit -m "docs: true-heading water verification procedure"
```

---

## Self-Review Notes

- Spec coverage: "true direction relative to the Pixhawk" ✔ (`vehicle_yaw`, T1); usable absolute-ish heading despite gyro-only yaw ✔ (HeadingRef T1, HeadingFusion T2); adoption so it's THE convention ✔ (T3); field validation ✔ (T4). AHRS_ORIENTATION alternative considered and deliberately deferred (header) with the exact two-line change documented in the module docstring.
- Type consistency: all angles radians, all `t` seconds monotonic; `heading()` may return None — T3 consumers all have a ZED/timed fallback already.
- Interaction with drift-reset plan: `HeadingFusion.reset()` exists for the tracking-reset step; noted in class docstring.
