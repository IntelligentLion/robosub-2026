#!/usr/bin/env python3
"""Prequalification mission runner for RoboSub 2026.

Drives a fixed, vision-triggered sequence by publishing low-level
``auv_msgs/MovementCommand`` on ``movement_command`` (consumed by
``mavlink_thruster_control/thruster_node``). It listens to the same vision and
depth streams the autonomous controller uses, so no PID/localization stack is
required for the run — every state has a vision trigger plus timed/spatial
fallbacks so the run always completes.

Algorithm (RoboSub 2026 prequalification):
  1.  Submerge until the gate is detected, then pause at that depth.
  2.  Submerge a little more until the top of the gate leaves the frame, so the
      sub passes cleanly under the top bar instead of colliding with it.
  3.  Move forward through the gate.
  4.  Move forward until the vertical marker is detected, then pause.
  5.  Strafe right until the marker is toward the left of the sub.
  6.  Move forward until the marker is behind the sub.
  7.  Turn left until the marker is to the left of the sub.
  8.  Move forward until the marker is behind the sub.
  9.  Turn left until the marker is to the left of the sub.
  10. Strafe left until the gate is detected, then pause.
  11. Align (centre) on the gate.
  12. Move forward through the gate.
  13. Keep moving forward a little more to fully clear it.
  14. Resurface.

Fallbacks (so the run never stalls):
  * Submerge: if the gate is not seen after ``submerge_timeout_s`` OR after
    descending to ``max_depth_m``, hold depth and just drive forward through
    where the gate should be.
  * Forward-to-marker: if the marker is not seen after
    ``forward_to_marker_timeout_s`` OR after travelling ``max_forward_distance_m``
    (when pose is available), begin the around-the-marker maneuver anyway.
  * Every other state has a per-state safety timeout that advances the machine.

Subscribes:
  - vision/detections   (auv_msgs/ObjectDetectionArray) — camera detections
  - depth/sub_depth     (std_msgs/Float32)              — depth below surface
  - localization/pose   (geometry_msgs/PoseStamped)     — optional, used for
                                                          closed-loop turn caps
                                                          and distance fallbacks

Publishes:
  - movement_command    (auv_msgs/MovementCommand)      — thruster commands

Detection conventions (see vision/detector.py):
  position.x, position.y are the bbox centre normalised to [0, 1] in the image
  (0.5, 0.5 = centre). position.z is range-to-target in metres (-1 if unknown).
  bbox_width / bbox_height are normalised to [0, 1].
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from auv_msgs.msg import (
    MovementCommand,
    ObjectDetection,
    ObjectDetectionArray,
)


def _is_finite(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def _angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ─── Mission states ────────────────────────────────────────────────────────
S_SUBMERGE = 'submerge'                       # submerge until gate detected
S_SUBMERGE_CLEAR_TOP = 'submerge_clear_top'   # dive until gate top out of view
S_THROUGH_GATE_1 = 'through_gate_1'
S_FORWARD_TO_MARKER = 'forward_to_marker'
S_STRAFE_MARKER_LEFT = 'strafe_marker_left'
S_FORWARD_PAST_MARKER = 'forward_past_marker'
S_TURN_LEFT_1 = 'turn_left_1'
S_FORWARD_MARKER_BEHIND_1 = 'forward_marker_behind_1'
S_TURN_LEFT_2 = 'turn_left_2'
S_STRAFE_TO_GATE = 'strafe_to_gate'           # strafe left until gate detected
S_ALIGN_GATE = 'align_gate'
S_THROUGH_GATE_2 = 'through_gate_2'
S_FINAL_FORWARD = 'final_forward'             # a little more forward to clear
S_SURFACE = 'surface'
S_DONE = 'done'

# States that intentionally drive the vertical axis. Everywhere else, depth is
# actively held (closed-loop) so it does not change unless we mean it to.
_VERTICAL_STATES = (S_SUBMERGE, S_SUBMERGE_CLEAR_TOP, S_SURFACE)


class PrequalificationNode(Node):
    """Scripted prequalification state machine."""

    def __init__(self):
        super().__init__('prequalification_node')

        # ── Parameters ──────────────────────────────────────────────
        p = self.declare_parameter
        # Vision labels the trained detector emits for these objects.
        p('gate_label', 'gate')
        p('marker_label', 'marker')
        # Depth handling (metres below surface, positive down).
        p('depth_tol_m', 0.15)
        # Safety cap: stop descending past this even if the gate is never seen.
        p('max_depth_m', 1.5)
        # Closed-loop depth hold. Once the descent ends, every non-vertical
        # state actively holds the captured depth (re-commanding submerge/emerge
        # off depth/sub_depth error) so depth only changes while intentionally
        # submerging or surfacing. Set enable_depth_hold false for the old
        # passive (neutral-thrust) behaviour.
        p('enable_depth_hold', True)
        # Depth-hold SOURCE: who keeps the captured depth in the hold states.
        #   'baro' → the Pixhawk's ALT_HOLD holds depth on the pressure sensor;
        #            prequal only commands neutral vertical (depth_hold). Needs
        #            thruster_node flight_mode=ALT_HOLD (the default).
        #   'zed'  → prequal runs its own closed-loop P-controller below off
        #            depth/sub_depth (ZED-derived). Needs thruster_node
        #            flight_mode=MANUAL, or it fights ALT_HOLD's baro loop.
        p('depth_hold_source', 'baro')
        p('depth_hold_tol_m', 0.10)
        p('depth_hold_gain', 1.0)        # command speed per metre of error
        p('depth_hold_min_speed', 0.10)
        p('depth_hold_max_speed', 0.40)
        # Speeds, normalised 0.0–1.0.
        p('surge_speed', 0.35)
        p('strafe_speed', 0.30)
        p('turn_speed', 0.30)
        p('submerge_speed', 0.40)
        p('surface_speed', 0.45)
        # Vision gating.
        p('detection_conf', 0.50)
        p('detection_stale_s', 1.0)
        p('center_tol', 0.10)
        p('yaw_center_gain', 0.6)
        # Strafe-right target: marker considered "to the left" when its
        # normalised image centre-x drops below this.
        p('marker_left_threshold', 0.30)
        # Top of the gate is "out of view" once its bbox top edge
        # (position.y - bbox_height/2) rises to/above this normalised row
        # (0 = top of frame). Then dive a touch more for clearance.
        p('gate_top_clear_y', 0.05)
        p('gate_top_clear_extra_s', 1.5)
        # Gate is "passed/aligned and close" when its bbox fills this fraction
        # of the frame width, or its range drops below this many metres.
        p('gate_close_bbox', 0.45)
        p('gate_close_range_m', 1.2)
        # How long to keep surging straight ahead to clear a gate once aligned.
        p('gate_pass_duration_s', 5.0)
        # Final little nudge forward after the second gate transit.
        p('final_forward_duration_s', 3.0)
        # A tracked marker is declared "behind" after it has been seen in the
        # current state and then goes unseen for this long.
        p('marker_lost_s', 1.5)
        # Turns: rotate left until the marker is to the left of the sub; capped
        # at a ~90 deg sweep (closed-loop on pose yaw when available, else
        # timed) so a missed detection cannot spin the sub forever.
        p('use_pose_for_turns', True)
        p('turn_90_duration_s', 6.0)
        p('turn_yaw_tol_rad', 0.10)
        # Minimum sweep before a turn may terminate on the marker — stops an
        # instant exit if the marker is already in frame as the turn begins.
        p('turn_min_s', 1.0)
        # Forward-to-marker distance fallback (metres, needs pose).
        p('max_forward_distance_m', 8.0)
        # Per-state safety timeouts (seconds). On timeout the state advances so
        # the run never stalls in the water.
        p('submerge_timeout_s', 12.0)
        p('submerge_clear_top_timeout_s', 8.0)
        p('through_gate_timeout_s', 12.0)
        p('forward_to_marker_timeout_s', 30.0)
        p('strafe_timeout_s', 12.0)
        p('forward_past_marker_timeout_s', 12.0)
        p('turn_timeout_s', 12.0)
        p('forward_marker_behind_timeout_s', 15.0)
        p('strafe_to_gate_timeout_s', 15.0)
        p('align_gate_timeout_s', 20.0)
        p('final_forward_timeout_s', 8.0)
        p('surface_timeout_s', 12.0)
        p('control_hz', 10.0)
        # Set false to plan/dry-run without ever commanding the thrusters.
        p('publish_commands', True)

        g = self.get_parameter
        self.gate_label = g('gate_label').value
        self.marker_label = g('marker_label').value
        self.depth_tol_m = float(g('depth_tol_m').value)
        self.max_depth_m = float(g('max_depth_m').value)
        self.enable_depth_hold = bool(g('enable_depth_hold').value)
        self.depth_hold_source = str(g('depth_hold_source').value).lower()
        if self.depth_hold_source not in ('baro', 'zed'):
            self.get_logger().warn(
                f'Unknown depth_hold_source "{self.depth_hold_source}" — '
                f'using "baro" (Pixhawk ALT_HOLD).')
            self.depth_hold_source = 'baro'
        self.get_logger().info(
            f'Depth-hold source: {self.depth_hold_source} '
            + ('(Pixhawk ALT_HOLD baro — set thruster flight_mode=ALT_HOLD)'
               if self.depth_hold_source == 'baro'
               else '(ZED P-controller — set thruster flight_mode=MANUAL)'))
        self.depth_hold_tol_m = float(g('depth_hold_tol_m').value)
        self.depth_hold_gain = float(g('depth_hold_gain').value)
        self.depth_hold_min_speed = float(g('depth_hold_min_speed').value)
        self.depth_hold_max_speed = float(g('depth_hold_max_speed').value)
        self.surge_speed = float(g('surge_speed').value)
        self.strafe_speed = float(g('strafe_speed').value)
        self.turn_speed = float(g('turn_speed').value)
        self.submerge_speed = float(g('submerge_speed').value)
        self.surface_speed = float(g('surface_speed').value)
        self.detection_conf = float(g('detection_conf').value)
        self.detection_stale_s = float(g('detection_stale_s').value)
        self.center_tol = float(g('center_tol').value)
        self.yaw_center_gain = float(g('yaw_center_gain').value)
        self.marker_left_threshold = float(g('marker_left_threshold').value)
        self.gate_top_clear_y = float(g('gate_top_clear_y').value)
        self.gate_top_clear_extra_s = float(g('gate_top_clear_extra_s').value)
        self.gate_close_bbox = float(g('gate_close_bbox').value)
        self.gate_close_range_m = float(g('gate_close_range_m').value)
        self.gate_pass_duration_s = float(g('gate_pass_duration_s').value)
        self.final_forward_duration_s = \
            float(g('final_forward_duration_s').value)
        self.marker_lost_s = float(g('marker_lost_s').value)
        self.use_pose_for_turns = bool(g('use_pose_for_turns').value)
        self.turn_90_duration_s = float(g('turn_90_duration_s').value)
        self.turn_yaw_tol_rad = float(g('turn_yaw_tol_rad').value)
        self.turn_min_s = float(g('turn_min_s').value)
        self.max_forward_distance_m = float(g('max_forward_distance_m').value)
        self.control_hz = max(1.0, float(g('control_hz').value))
        self.publish_commands = bool(g('publish_commands').value)

        self._state_timeouts = {
            S_SUBMERGE: float(g('submerge_timeout_s').value),
            S_SUBMERGE_CLEAR_TOP: float(g('submerge_clear_top_timeout_s').value),
            S_THROUGH_GATE_1: float(g('through_gate_timeout_s').value),
            S_FORWARD_TO_MARKER: float(g('forward_to_marker_timeout_s').value),
            S_STRAFE_MARKER_LEFT: float(g('strafe_timeout_s').value),
            S_FORWARD_PAST_MARKER:
                float(g('forward_past_marker_timeout_s').value),
            S_TURN_LEFT_1: float(g('turn_timeout_s').value),
            S_FORWARD_MARKER_BEHIND_1:
                float(g('forward_marker_behind_timeout_s').value),
            S_TURN_LEFT_2: float(g('turn_timeout_s').value),
            S_STRAFE_TO_GATE: float(g('strafe_to_gate_timeout_s').value),
            S_ALIGN_GATE: float(g('align_gate_timeout_s').value),
            S_THROUGH_GATE_2: float(g('through_gate_timeout_s').value),
            S_FINAL_FORWARD: float(g('final_forward_timeout_s').value),
            S_SURFACE: float(g('surface_timeout_s').value),
        }

        # Where each state goes when its safety timeout fires.
        self._timeout_next = {
            S_SUBMERGE: S_THROUGH_GATE_1,        # hold depth, drive forward
            S_SUBMERGE_CLEAR_TOP: S_THROUGH_GATE_1,
            S_THROUGH_GATE_1: S_FORWARD_TO_MARKER,
            S_FORWARD_TO_MARKER: S_STRAFE_MARKER_LEFT,   # do the maneuver anyway
            S_STRAFE_MARKER_LEFT: S_FORWARD_PAST_MARKER,
            S_FORWARD_PAST_MARKER: S_TURN_LEFT_1,
            S_TURN_LEFT_1: S_FORWARD_MARKER_BEHIND_1,
            S_FORWARD_MARKER_BEHIND_1: S_TURN_LEFT_2,
            S_TURN_LEFT_2: S_STRAFE_TO_GATE,
            S_STRAFE_TO_GATE: S_ALIGN_GATE,
            S_ALIGN_GATE: S_THROUGH_GATE_2,
            S_THROUGH_GATE_2: S_FINAL_FORWARD,
            S_FINAL_FORWARD: S_SURFACE,
            S_SURFACE: S_DONE,
        }

        # ── State ───────────────────────────────────────────────────
        self._state = S_SUBMERGE
        self._state_start = self.get_clock().now()
        self._substate_start = None        # generic intra-state timer
        self._state_start_pos = None       # pose pos at state entry (for dist)

        # Vision
        self._detections = []
        self._det_stamp = self.get_clock().now()
        # Marker-tracking memory used by the "marker behind" states.
        self._marker_seen_this_state = False

        # Depth / pose
        self._depth_m = -1.0
        # Depth to hold once descending stops (captured live while submerging).
        self._hold_depth_m = None
        self._pose_yaw = 0.0
        self._pose_received = False
        self._pose_pos = (0.0, 0.0)
        self._pose_pos_received = False
        self._turn_start_yaw = None        # set on entering a turn state

        # ── ROS I/O ─────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self.create_subscription(
            ObjectDetectionArray, 'vision/detections', self._vision_cb, 10)
        self.create_subscription(
            Float32, 'depth/sub_depth', self._depth_cb, 10)
        self.create_subscription(
            PoseStamped, 'localization/pose', self._pose_cb, 10)

        self.create_timer(1.0 / self.control_hz, self._tick)

        self.get_logger().info(
            'Prequalification runner started — '
            f'gate="{self.gate_label}" marker="{self.marker_label}" '
            f'max_depth={self.max_depth_m:.2f}m '
            f'publish={self.publish_commands}')
        self.get_logger().info(f'State -> {self._state}')

    # ─── Callbacks ──────────────────────────────────────────────────

    def _vision_cb(self, msg: ObjectDetectionArray):
        try:
            self._detections = msg.detections
            self._det_stamp = self.get_clock().now()
        except Exception as e:  # noqa: BLE001 — never let a cb kill the node
            self.get_logger().error(f'Vision callback error: {e}')

    def _depth_cb(self, msg: Float32):
        try:
            if _is_finite(msg.data):
                self._depth_m = float(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Depth callback error: {e}')

    def _pose_cb(self, msg: PoseStamped):
        try:
            q = msg.pose.orientation
            if all(_is_finite(v) for v in (q.w, q.x, q.y, q.z)):
                self._pose_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self._pose_received = True
            pos = msg.pose.position
            if all(_is_finite(v) for v in (pos.x, pos.y)):
                self._pose_pos = (float(pos.x), float(pos.y))
                self._pose_pos_received = True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Pose callback error: {e}')

    # ─── Helpers ────────────────────────────────────────────────────

    def _send(self, command, speed=0.0, duration=0.0):
        if not self.publish_commands:
            return
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(max(0.0, min(1.0, speed)))
        msg.duration = float(max(0.0, duration))
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._send('stop')

    def _best_detection(self, label) -> ObjectDetection:
        """Highest-confidence fresh detection for ``label``, or None."""
        age = (self.get_clock().now() - self._det_stamp).nanoseconds / 1e9
        if age > self.detection_stale_s:
            return None
        best = None
        for d in self._detections:
            if d.label == label and d.confidence >= self.detection_conf:
                if best is None or d.confidence > best.confidence:
                    best = d
        return best

    def _elapsed_in_state(self):
        return (self.get_clock().now() - self._state_start).nanoseconds / 1e9

    def _elapsed_substate(self):
        if self._substate_start is None:
            self._substate_start = self.get_clock().now()
        return (self.get_clock().now()
                - self._substate_start).nanoseconds / 1e9

    def _distance_since_state(self):
        """Planar distance travelled since entering the current state, or None
        when pose is unavailable."""
        if not self._pose_pos_received or self._state_start_pos is None:
            return None
        dx = self._pose_pos[0] - self._state_start_pos[0]
        dy = self._pose_pos[1] - self._state_start_pos[1]
        return math.hypot(dx, dy)

    def _transition(self, new_state):
        self._stop()
        self._state = new_state
        self._state_start = self.get_clock().now()
        self._substate_start = None
        self._marker_seen_this_state = False
        self._turn_start_yaw = None
        self._state_start_pos = \
            self._pose_pos if self._pose_pos_received else None
        self.get_logger().info(f'State -> {new_state}')

    def _center_horizontally(self, det, surge_when_centered=0.0):
        """Yaw/strafe to centre on ``det``.

        Returns True when within ``center_tol`` of the image centre.  While
        centred, optionally surges forward at ``surge_when_centered``.
        """
        ex = det.position.x - 0.5
        if abs(ex) <= self.center_tol:
            if surge_when_centered > 0.0:
                self._send('surge_forward', surge_when_centered)
            else:
                self._stop()
            return True
        speed = min(self.turn_speed, max(0.10, abs(ex) * self.yaw_center_gain))
        if ex > 0:
            self._send('rotate_cw', speed)
        else:
            self._send('rotate_ccw', speed)
        return False

    def _turn_left_until_marker(self):
        """One control tick of a left (CCW) turn that stops once the marker is
        to the left of the sub.

        Terminates (returns True) on the first of:
          * marker detected with centre-x <= ``marker_left_threshold`` after a
            minimum ``turn_min_s`` sweep (the intended vision trigger), or
          * a ~90 deg sweep completes — closed-loop on pose yaw when available,
            else after ``turn_90_duration_s`` — as a hard cap so a missed
            detection cannot spin the sub indefinitely.
        """
        if self._substate_start is None:
            self._substate_start = self.get_clock().now()
            self._turn_start_yaw = \
                self._pose_yaw if self._pose_received else None
        elapsed = (self.get_clock().now()
                   - self._substate_start).nanoseconds / 1e9

        # Vision termination, after a minimum sweep.
        if elapsed >= self.turn_min_s:
            det = self._best_detection(self.marker_label)
            if det is not None and \
                    det.position.x <= self.marker_left_threshold:
                self.get_logger().info(
                    f'Turn left: marker now left (cx={det.position.x:.2f})')
                self._stop()
                return True

        # Pose cap at ~90 deg.
        if self.use_pose_for_turns and self._pose_received and \
                self._turn_start_yaw is not None:
            turned = abs(_angle_diff(self._pose_yaw, self._turn_start_yaw))
            if turned >= (math.pi / 2.0 - self.turn_yaw_tol_rad):
                self.get_logger().info('Turn left: 90 deg cap reached (pose)')
                self._stop()
                return True

        # Timed cap.
        if elapsed >= self.turn_90_duration_s:
            self.get_logger().info('Turn left: timed cap reached')
            self._stop()
            return True

        # CCW (left) is positive yaw → rotate_ccw.
        self._send('rotate_ccw', self.turn_speed)
        return False

    def _marker_behind(self):
        """True once the marker has been seen this state then lost for
        ``marker_lost_s`` — i.e. the sub has driven past it."""
        det = self._best_detection(self.marker_label)
        if det is not None:
            self._marker_seen_this_state = True
            self._substate_start = None
            return False
        if not self._marker_seen_this_state:
            return False
        if self._substate_start is None:
            self._substate_start = self.get_clock().now()
        lost = (self.get_clock().now()
                - self._substate_start).nanoseconds / 1e9
        return lost >= self.marker_lost_s

    # ─── Main control loop ──────────────────────────────────────────

    def _tick(self):
        try:
            if self._state == S_DONE:
                return

            # Per-state safety timeout: advance rather than stall.
            timeout = self._state_timeouts.get(self._state)
            if timeout is not None and self._elapsed_in_state() > timeout:
                self.get_logger().warn(
                    f'State "{self._state}" timed out after {timeout:.0f}s '
                    f'— advancing')
                self._advance_on_timeout()
                return

            handler = getattr(self, '_do_' + self._state, None)
            if handler is None:
                self.get_logger().error(
                    f'No handler for state "{self._state}" — surfacing')
                self._transition(S_SURFACE)
                return
            handler()

            # Depth bookkeeping. While intentionally moving the vertical axis,
            # track the live depth so we keep the depth we end up at. Otherwise
            # actively hold that depth on top of whatever the handler commanded
            # (the z axis is independent of surge/strafe/yaw at the thruster).
            if self._state in _VERTICAL_STATES:
                if self._depth_m >= 0.0:
                    self._hold_depth_m = self._depth_m
            elif self._state != S_DONE:
                self._apply_depth_hold()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Tick error: {e} — stopping')
            self._stop()

    def _apply_depth_hold(self):
        """Closed-loop hold of ``self._hold_depth_m`` via the vertical axis.

        The thruster node treats each axis independently and the z command is
        sticky, so issuing a submerge/emerge/depth_hold after the handler's
        surge/strafe/yaw corrects depth without disturbing the planar motion.
        """
        if self.depth_hold_source == 'baro':
            # Pixhawk ALT_HOLD owns depth from the baro — just hold neutral
            # vertical so we don't fight the autopilot's depth loop.
            self._send('depth_hold')
            return
        if not self.enable_depth_hold or self._hold_depth_m is None:
            return
        if self._depth_m < 0.0:
            return  # no depth feedback yet — leave the axis as-is
        err = self._hold_depth_m - self._depth_m  # +ve → need to go deeper
        if abs(err) <= self.depth_hold_tol_m:
            self._send('depth_hold')
            return
        speed = min(self.depth_hold_max_speed,
                    max(self.depth_hold_min_speed,
                        abs(err) * self.depth_hold_gain))
        self._send('submerge' if err > 0 else 'emerge', speed)

    def _advance_on_timeout(self):
        """Next state when a state times out (see ``_timeout_next``)."""
        nxt = self._timeout_next.get(self._state, S_SURFACE)
        self._transition(nxt)

    # ─── State handlers ─────────────────────────────────────────────

    def _do_submerge(self):
        # 1. Submerge until the gate is detected, then pause at that depth.
        det = self._best_detection(self.gate_label)
        if det is not None:
            self.get_logger().info(
                f'Gate detected at depth {self._depth_m:.2f}m — pausing')
            self._stop()
            self._transition(S_SUBMERGE_CLEAR_TOP)
            return
        # Depth-cap fallback: never seen the gate but at max depth → hold and go.
        if self._depth_m >= 0.0 and self._depth_m >= self.max_depth_m:
            self.get_logger().warn(
                f'Reached max depth {self.max_depth_m:.2f}m with no gate '
                f'— holding depth and driving forward')
            self._transition(S_THROUGH_GATE_1)
            return
        self._send('submerge', self.submerge_speed)

    def _do_submerge_clear_top(self):
        # 2. Dive a bit more until the top of the gate leaves the frame, plus a
        #    short extra so the sub passes cleanly under the top bar.
        # Safety: do not punch past the depth cap.
        if self._depth_m >= 0.0 and self._depth_m >= self.max_depth_m:
            self.get_logger().warn('Depth cap reached while clearing gate top')
            self._transition(S_THROUGH_GATE_1)
            return

        if self._substate_start is not None:
            # Top has cleared — committed to a short extra descent.
            if self._elapsed_substate() >= self.gate_top_clear_extra_s:
                self.get_logger().info('Gate top cleared — driving through')
                self._transition(S_THROUGH_GATE_1)
                return
            self._send('submerge', self.submerge_speed)
            return

        det = self._best_detection(self.gate_label)
        # Gate lost entirely, or its top edge has risen to the top of frame:
        # the top is out of view → commit the short extra descent.
        top_out = (det is None) or \
            ((det.position.y - det.bbox_height / 2.0) <= self.gate_top_clear_y)
        if top_out:
            self._substate_start = self.get_clock().now()
        self._send('submerge', self.submerge_speed)

    def _do_through_gate_1(self):
        # 3. Move forward through the gate.
        self._drive_through_gate(S_FORWARD_TO_MARKER)

    def _do_forward_to_marker(self):
        # 4. Move forward until the vertical marker is detected, then pause.
        det = self._best_detection(self.marker_label)
        if det is not None:
            self.get_logger().info('Vertical marker detected — pausing')
            self._stop()
            self._transition(S_STRAFE_MARKER_LEFT)
            return
        # Distance fallback: travelled far enough with no marker → maneuver.
        dist = self._distance_since_state()
        if dist is not None and dist >= self.max_forward_distance_m:
            self.get_logger().warn(
                f'No marker after {dist:.1f}m — beginning marker maneuver')
            self._transition(S_STRAFE_MARKER_LEFT)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_strafe_marker_left(self):
        # 5. Strafe right until the marker is toward the left of the sub.
        det = self._best_detection(self.marker_label)
        if det is not None and det.position.x <= self.marker_left_threshold:
            self.get_logger().info(
                f'Marker now left (cx={det.position.x:.2f})')
            self._transition(S_FORWARD_PAST_MARKER)
            return
        self._send('strafe_right', self.strafe_speed)

    def _do_forward_past_marker(self):
        # 6. Move forward until the marker is behind the sub.
        if self._marker_behind():
            self.get_logger().info('Reached back of marker')
            self._transition(S_TURN_LEFT_1)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_turn_left_1(self):
        # 7. Turn left until the marker is to the left of the sub.
        if self._turn_left_until_marker():
            self._transition(S_FORWARD_MARKER_BEHIND_1)

    def _do_forward_marker_behind_1(self):
        # 8. Move forward until the marker is behind the sub.
        if self._marker_behind():
            self.get_logger().info('Marker behind (leg 1)')
            self._transition(S_TURN_LEFT_2)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_turn_left_2(self):
        # 9. Turn left until the marker is to the left of the sub.
        if self._turn_left_until_marker():
            self._transition(S_STRAFE_TO_GATE)

    def _do_strafe_to_gate(self):
        # 10. Strafe left until the gate is detected, then pause.
        det = self._best_detection(self.gate_label)
        if det is not None:
            self.get_logger().info('Gate re-detected — pausing')
            self._stop()
            self._transition(S_ALIGN_GATE)
            return
        self._send('strafe_left', self.strafe_speed)

    def _do_align_gate(self):
        # 11. Align (centre) on the gate.
        det = self._best_detection(self.gate_label)
        if det is None:
            # Creep forward to bring the gate into view.
            self._send('surge_forward', self.surge_speed * 0.6)
            return
        if self._center_horizontally(det):
            self.get_logger().info('Gate aligned')
            self._transition(S_THROUGH_GATE_2)

    def _do_through_gate_2(self):
        # 12. Move forward through the gate.
        self._drive_through_gate(S_FINAL_FORWARD)

    def _do_final_forward(self):
        # 13. Keep moving forward a little more to fully clear the gate.
        if self._elapsed_in_state() >= self.final_forward_duration_s:
            self.get_logger().info('Cleared gate — surfacing')
            self._transition(S_SURFACE)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_surface(self):
        # 14. Resurface.
        if 0.0 <= self._depth_m <= self.depth_tol_m:
            self.get_logger().info('Surfaced — prequalification complete')
            self._stop()
            self._transition(S_DONE)
            return
        self._send('emerge', self.surface_speed)

    # ─── Shared gate-transit behaviour ──────────────────────────────

    def _drive_through_gate(self, next_state):
        """Centre on the gate, approach, then surge straight through.

        Once the gate is centred and close (large bbox or short range), commit
        to driving straight ahead for ``gate_pass_duration_s`` since the gate
        leaves the camera frame as the sub passes under/through it.
        """
        if self._substate_start is not None:
            # Committed: drive straight through for the pass duration.
            if self._elapsed_substate() >= self.gate_pass_duration_s:
                self.get_logger().info('Cleared the gate')
                self._transition(next_state)
                return
            self._send('surge_forward', self.surge_speed)
            return

        det = self._best_detection(self.gate_label)
        if det is None:
            # Lost sight before committing — keep creeping forward to reacquire.
            self._send('surge_forward', self.surge_speed * 0.6)
            return

        close = ((det.bbox_width >= self.gate_close_bbox) or
                 (0.0 < det.position.z <= self.gate_close_range_m))
        centered = self._center_horizontally(det, self.surge_speed)
        if centered and close:
            self.get_logger().info('Committing to gate transit')
            self._substate_start = self.get_clock().now()


def main():
    rclpy.init()
    node = None
    try:
        node = PrequalificationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        if node:
            node.get_logger().fatal(f'Unhandled exception: {e}')
        else:
            print(f'[prequalification_node] Fatal startup error: {e}')
    finally:
        if node:
            try:
                node._stop()
            except Exception:  # noqa: BLE001
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
