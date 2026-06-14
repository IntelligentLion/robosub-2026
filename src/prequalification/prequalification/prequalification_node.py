#!/usr/bin/env python3
"""Prequalification mission runner for RoboSub 2026.

Drives a fixed, vision-triggered sequence by publishing low-level
``auv_msgs/MovementCommand`` on ``movement_command`` (consumed by
``mavlink_thruster_control/thruster_node``). It listens to the same vision and
depth streams the autonomous controller uses, so no PID/localization stack is
required for the run — every state has a vision trigger plus a timed fallback
so the run always completes.

Algorithm (RoboSub 2026 prequalification):
  1.  Submerge to the target depth.
  2.  Detect the gate.
  3.  Move through the gate.
  4.  Move forward until the vertical marker is detected.
  5.  Strafe right until the marker is toward the left of the sub.
  6.  Move forward toward the back of the marker.
  7.  Turn left 90 degrees.
  8.  Move forward until the marker is behind the sub.
  9.  Turn left 90 degrees to face the gate.
  10. Move forward until the marker is behind the sub again.
  11. Align to the gate.
  12. Move through the gate.
  13. Resurface.

Subscribes:
  - vision/detections   (auv_msgs/ObjectDetectionArray) — camera detections
  - depth/sub_depth     (std_msgs/Float32)              — depth below surface
  - localization/pose   (geometry_msgs/PoseStamped)     — optional, for closed
                                                          loop 90 deg turns

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
S_SUBMERGE = 'submerge'
S_DETECT_GATE = 'detect_gate'
S_THROUGH_GATE_1 = 'through_gate_1'
S_FORWARD_TO_MARKER = 'forward_to_marker'
S_STRAFE_MARKER_LEFT = 'strafe_marker_left'
S_FORWARD_PAST_MARKER = 'forward_past_marker'
S_TURN_LEFT_1 = 'turn_left_1'
S_FORWARD_MARKER_BEHIND_1 = 'forward_marker_behind_1'
S_TURN_LEFT_2 = 'turn_left_2'
S_FORWARD_MARKER_BEHIND_2 = 'forward_marker_behind_2'
S_ALIGN_GATE = 'align_gate'
S_THROUGH_GATE_2 = 'through_gate_2'
S_SURFACE = 'surface'
S_DONE = 'done'


class PrequalificationNode(Node):
    """Scripted prequalification state machine."""

    def __init__(self):
        super().__init__('prequalification_node')

        # ── Parameters ──────────────────────────────────────────────
        p = self.declare_parameter
        # Vision labels the trained detector emits for these objects.
        p('gate_label', 'gate')
        p('marker_label', 'marker')
        # Depth target for the run (metres below surface, positive down).
        p('target_depth_m', 1.0)
        p('depth_tol_m', 0.15)
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
        # Gate is "passed/aligned and close" when its bbox fills this fraction
        # of the frame width, or its range drops below this many metres.
        p('gate_close_bbox', 0.45)
        p('gate_close_range_m', 1.2)
        # How long to keep surging straight ahead to clear a gate once aligned.
        p('gate_pass_duration_s', 5.0)
        # A tracked marker is declared "behind" after it has been seen in the
        # current state and then goes unseen for this long.
        p('marker_lost_s', 1.5)
        # 90-degree turn: closed-loop on yaw if pose is available, else timed.
        p('use_pose_for_turns', True)
        p('turn_90_duration_s', 6.0)
        p('turn_yaw_tol_rad', 0.10)
        # Per-state safety timeouts (seconds). On timeout the state advances so
        # the run never stalls in the water.
        p('submerge_timeout_s', 12.0)
        p('detect_gate_timeout_s', 30.0)
        p('forward_to_marker_timeout_s', 30.0)
        p('strafe_timeout_s', 12.0)
        p('forward_past_marker_timeout_s', 12.0)
        p('forward_marker_behind_timeout_s', 15.0)
        p('align_gate_timeout_s', 20.0)
        p('surface_timeout_s', 12.0)
        p('control_hz', 10.0)
        # Set false to plan/dry-run without ever commanding the thrusters.
        p('publish_commands', True)

        g = self.get_parameter
        self.gate_label = g('gate_label').value
        self.marker_label = g('marker_label').value
        self.target_depth_m = float(g('target_depth_m').value)
        self.depth_tol_m = float(g('depth_tol_m').value)
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
        self.gate_close_bbox = float(g('gate_close_bbox').value)
        self.gate_close_range_m = float(g('gate_close_range_m').value)
        self.gate_pass_duration_s = float(g('gate_pass_duration_s').value)
        self.marker_lost_s = float(g('marker_lost_s').value)
        self.use_pose_for_turns = bool(g('use_pose_for_turns').value)
        self.turn_90_duration_s = float(g('turn_90_duration_s').value)
        self.turn_yaw_tol_rad = float(g('turn_yaw_tol_rad').value)
        self.control_hz = max(1.0, float(g('control_hz').value))
        self.publish_commands = bool(g('publish_commands').value)

        self._state_timeouts = {
            S_SUBMERGE: float(g('submerge_timeout_s').value),
            S_DETECT_GATE: float(g('detect_gate_timeout_s').value),
            S_THROUGH_GATE_1: self.gate_pass_duration_s + 5.0,
            S_FORWARD_TO_MARKER: float(g('forward_to_marker_timeout_s').value),
            S_STRAFE_MARKER_LEFT: float(g('strafe_timeout_s').value),
            S_FORWARD_PAST_MARKER:
                float(g('forward_past_marker_timeout_s').value),
            S_TURN_LEFT_1: self.turn_90_duration_s + 4.0,
            S_FORWARD_MARKER_BEHIND_1:
                float(g('forward_marker_behind_timeout_s').value),
            S_TURN_LEFT_2: self.turn_90_duration_s + 4.0,
            S_FORWARD_MARKER_BEHIND_2:
                float(g('forward_marker_behind_timeout_s').value),
            S_ALIGN_GATE: float(g('align_gate_timeout_s').value),
            S_THROUGH_GATE_2: self.gate_pass_duration_s + 5.0,
            S_SURFACE: float(g('surface_timeout_s').value),
        }

        # ── State ───────────────────────────────────────────────────
        self._state = S_SUBMERGE
        self._state_start = self.get_clock().now()
        self._substate_start = None        # generic intra-state timer

        # Vision
        self._detections = []
        self._det_stamp = self.get_clock().now()
        # Marker-tracking memory used by the "marker behind" states.
        self._marker_seen_this_state = False

        # Depth / pose
        self._depth_m = -1.0
        self._pose_yaw = 0.0
        self._pose_received = False
        self._turn_target_yaw = None       # set on entering a turn state

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
            f'depth={self.target_depth_m:.2f}m '
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
            if not all(_is_finite(v) for v in (q.w, q.x, q.y, q.z)):
                return
            self._pose_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._pose_received = True
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

    def _transition(self, new_state):
        self._stop()
        self._state = new_state
        self._state_start = self.get_clock().now()
        self._substate_start = None
        self._marker_seen_this_state = False
        self._turn_target_yaw = None
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

    def _turn_left_90(self):
        """One control tick of a left (CCW) 90 degree turn.

        Closed-loop on yaw when pose is available, otherwise timed.  Returns
        True once the turn is complete.
        """
        if self.use_pose_for_turns and self._pose_received:
            if self._turn_target_yaw is None:
                self._turn_target_yaw = _angle_diff(
                    self._pose_yaw + math.pi / 2.0, 0.0)
                self.get_logger().info(
                    f'Turn left 90: {math.degrees(self._pose_yaw):.0f} -> '
                    f'{math.degrees(self._turn_target_yaw):.0f} deg')
            err = _angle_diff(self._turn_target_yaw, self._pose_yaw)
            if abs(err) <= self.turn_yaw_tol_rad:
                self._stop()
                return True
            # CCW (left) is positive yaw → rotate_ccw.
            self._send('rotate_ccw', self.turn_speed)
            return False

        # Timed fallback.
        if self._substate_start is None:
            self._substate_start = self.get_clock().now()
        elapsed = (self.get_clock().now()
                   - self._substate_start).nanoseconds / 1e9
        if elapsed >= self.turn_90_duration_s:
            self._stop()
            return True
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
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Tick error: {e} — stopping')
            self._stop()

    def _advance_on_timeout(self):
        """Best-effort next state when a state times out."""
        order = [
            S_SUBMERGE, S_DETECT_GATE, S_THROUGH_GATE_1, S_FORWARD_TO_MARKER,
            S_STRAFE_MARKER_LEFT, S_FORWARD_PAST_MARKER, S_TURN_LEFT_1,
            S_FORWARD_MARKER_BEHIND_1, S_TURN_LEFT_2,
            S_FORWARD_MARKER_BEHIND_2, S_ALIGN_GATE, S_THROUGH_GATE_2,
            S_SURFACE, S_DONE,
        ]
        try:
            nxt = order[order.index(self._state) + 1]
        except (ValueError, IndexError):
            nxt = S_SURFACE
        self._transition(nxt)

    # ─── State handlers ─────────────────────────────────────────────

    def _do_submerge(self):
        # 1. Submerge to target depth.
        if self._depth_m >= 0.0 and \
                self._depth_m >= self.target_depth_m - self.depth_tol_m:
            self.get_logger().info(
                f'Reached depth {self._depth_m:.2f}m')
            self._transition(S_DETECT_GATE)
            return
        self._send('submerge', self.submerge_speed)

    def _do_detect_gate(self):
        # 2. Detect the gate, then centre on it.
        self._send('depth_hold')
        det = self._best_detection(self.gate_label)
        if det is None:
            # Hold depth and wait; gate is straight ahead at the start.
            return
        if self._center_horizontally(det):
            self.get_logger().info('Gate detected and centred')
            self._transition(S_THROUGH_GATE_1)

    def _do_through_gate_1(self):
        # 3. Move through the gate.
        self._drive_through_gate(S_FORWARD_TO_MARKER)

    def _do_forward_to_marker(self):
        # 4. Move forward until the vertical marker is detected.
        det = self._best_detection(self.marker_label)
        if det is not None:
            self.get_logger().info('Vertical marker detected')
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
        # 6. Move forward toward the back of the marker (until we pass it).
        if self._marker_behind():
            self.get_logger().info('Reached back of marker')
            self._transition(S_TURN_LEFT_1)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_turn_left_1(self):
        # 7. Turn left 90 degrees.
        if self._turn_left_90():
            self._transition(S_FORWARD_MARKER_BEHIND_1)

    def _do_forward_marker_behind_1(self):
        # 8. Move forward until the marker is behind the sub.
        if self._marker_behind():
            self.get_logger().info('Marker behind (leg 1)')
            self._transition(S_TURN_LEFT_2)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_turn_left_2(self):
        # 9. Turn left 90 degrees to face the gate.
        if self._turn_left_90():
            self._transition(S_FORWARD_MARKER_BEHIND_2)

    def _do_forward_marker_behind_2(self):
        # 10. Move forward until the marker is behind the sub again.
        if self._marker_behind():
            self.get_logger().info('Marker behind (leg 2)')
            self._transition(S_ALIGN_GATE)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_align_gate(self):
        # 11. Align to the gate.
        det = self._best_detection(self.gate_label)
        if det is None:
            # Creep forward to bring the gate into view.
            self._send('surge_forward', self.surge_speed * 0.6)
            return
        if self._center_horizontally(det):
            self.get_logger().info('Gate aligned')
            self._transition(S_THROUGH_GATE_2)

    def _do_through_gate_2(self):
        # 12. Move through the gate.
        self._drive_through_gate(S_SURFACE)

    def _do_surface(self):
        # 13. Resurface.
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
            elapsed = (self.get_clock().now()
                       - self._substate_start).nanoseconds / 1e9
            if elapsed >= self.gate_pass_duration_s:
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
