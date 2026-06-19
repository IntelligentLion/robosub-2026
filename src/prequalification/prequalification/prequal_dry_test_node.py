#!/usr/bin/env python3
"""Prequalification DRY-TEST runner for RoboSub 2026 (logic-only, no thrusters).

This is a *bench* version of ``prequalification_node`` for validating the
prequalification decision logic with nothing but a camera in your hand. The
trained model currently emits two classes, ``pointed_nose`` and ``round_nose``,
so for this dry test they stand in for the real prequal objects:

    pointed_nose  ->  FULL GATE       (the gate you swim through)
    round_nose    ->  VERTICAL MARKER (the pole you go around)

You hold the camera and point it at each object to drive the state machine. To
simulate motion you physically move the camera: forward = surge, up = surface,
down = submerge, left/right = strafe, rotating = yaw. Nothing is ever driven —
the node still publishes the *real* ``movement_command`` topic so the print-only
thruster stand-in in ``dry_test.py --camera`` shows exactly what the sub would
do, but no Pixhawk is required.

This now mirrors the **full** ``prequalification_node`` state machine (same 14
state names, so the logs line up) instead of the old collapsed flow. There is
no depth sensor or pose on a hand-held test, so the states that depend on those
fall back to timed behaviour:

  * ``submerge`` advances on gate detection, else on ``submerge_timeout_s``.
  * ``submerge_clear_top`` watches the gate bbox top edge leave the frame
    (point the camera so the gate rides high), then a short timed extra.
  * the ``turn_left_*`` states stop when the marker swings to the left of the
    frame, capped by ``turn_90_duration_s`` (no pose to close the loop on).
  * ``through_gate_*``, ``final_forward`` and ``surface`` are timed.

Every state also has a per-state safety timeout, exactly like the real node, so
the dry test never stalls if you cannot reproduce a trigger by hand.

Subscribes:  vision/detections (auv_msgs/ObjectDetectionArray)
Publishes:   movement_command  (auv_msgs/MovementCommand)

Run (camera-only dry test, three terminals):
  A:  python3 dry_test.py --camera                 # print-only thrusters + dets
  B:  ros2 run vision detector ...                 # real camera + detector
  C:  ros2 run prequalification prequalification_dry_test
      (or: python3 src/prequalification/prequalification/prequal_dry_test_node.py)
"""

import rclpy
from rclpy.node import Node

from auv_msgs.msg import (
    MovementCommand,
    ObjectDetection,
    ObjectDetectionArray,
)


# ─── Mission states (mirror prequalification_node) ──────────────────────────
S_SUBMERGE = 'submerge'
S_SUBMERGE_CLEAR_TOP = 'submerge_clear_top'
S_THROUGH_GATE_1 = 'through_gate_1'
S_FORWARD_TO_MARKER = 'forward_to_marker'
S_STRAFE_MARKER_LEFT = 'strafe_marker_left'
S_FORWARD_PAST_MARKER = 'forward_past_marker'
S_TURN_LEFT_1 = 'turn_left_1'
S_FORWARD_MARKER_BEHIND_1 = 'forward_marker_behind_1'
S_TURN_LEFT_2 = 'turn_left_2'
S_STRAFE_TO_GATE = 'strafe_to_gate'
S_ALIGN_GATE = 'align_gate'
S_THROUGH_GATE_2 = 'through_gate_2'
S_FINAL_FORWARD = 'final_forward'
S_SURFACE = 'surface'
S_DONE = 'done'


class PrequalDryTestNode(Node):
    """Camera-driven, print-only prequalification logic tester."""

    def __init__(self):
        super().__init__('prequal_dry_test_node')

        p = self.declare_parameter
        # Class names the detector emits, in their dry-test roles.
        p('gate_label', 'pointed_nose')      # plays the FULL GATE
        p('marker_label', 'round_nose')      # plays the VERTICAL MARKER
        # Speeds, normalised 0.0-1.0.
        p('surge_speed', 0.35)
        p('strafe_speed', 0.30)
        p('turn_speed', 0.30)
        p('submerge_speed', 0.40)
        p('surface_speed', 0.45)
        # Vision gating.
        p('detection_conf', 0.50)
        p('detection_stale_s', 1.0)
        p('center_tol', 0.12)
        p('yaw_center_gain', 0.6)
        # Marker is "to the left" when its normalised image centre-x < this.
        p('marker_left_threshold', 0.30)
        # Gate top is "out of view" once its bbox top edge
        # (centre_y - height/2) reaches this normalised row (0 = top of frame).
        p('gate_top_clear_y', 0.05)
        p('gate_top_clear_extra_s', 1.5)
        # Gate is "close enough to commit to transit" when its bbox fills this
        # fraction of the frame, or its range drops below this many metres, or
        # it has simply been centred continuously for align_commit_dwell_s
        # (range/bbox may be unavailable on a hand-held dry test).
        p('gate_close_bbox', 0.45)
        p('gate_close_range_m', 1.2)
        p('align_commit_dwell_s', 1.5)
        # Timed phases (no depth feedback on a hand-held dry test).
        p('gate_pass_duration_s', 5.0)
        p('final_forward_duration_s', 3.0)
        p('surface_duration_s', 5.0)
        # A tracked marker is "behind" after being seen this state then unseen
        # for this long.
        p('marker_lost_s', 1.5)
        # Turn-left states cap (no pose to close the loop on a dry test).
        p('turn_90_duration_s', 6.0)
        p('turn_min_s', 1.0)
        # Per-state safety timeouts (seconds) — advance rather than stall.
        p('submerge_timeout_s', 30.0)
        p('submerge_clear_top_timeout_s', 10.0)
        p('through_gate_timeout_s', 12.0)
        p('forward_to_marker_timeout_s', 30.0)
        p('strafe_timeout_s', 20.0)
        p('forward_past_marker_timeout_s', 20.0)
        p('turn_timeout_s', 12.0)
        p('forward_marker_behind_timeout_s', 20.0)
        p('strafe_to_gate_timeout_s', 30.0)
        p('align_gate_timeout_s', 30.0)
        p('final_forward_timeout_s', 8.0)
        p('surface_timeout_s', 12.0)
        p('control_hz', 10.0)
        # Set false to log the flow without ever touching movement_command.
        p('publish_commands', True)

        g = self.get_parameter
        self.gate_label = g('gate_label').value
        self.marker_label = g('marker_label').value
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
        self.align_commit_dwell_s = float(g('align_commit_dwell_s').value)
        self.gate_pass_duration_s = float(g('gate_pass_duration_s').value)
        self.final_forward_duration_s = \
            float(g('final_forward_duration_s').value)
        self.surface_duration_s = float(g('surface_duration_s').value)
        self.marker_lost_s = float(g('marker_lost_s').value)
        self.turn_90_duration_s = float(g('turn_90_duration_s').value)
        self.turn_min_s = float(g('turn_min_s').value)
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
        self._timeout_next = {
            S_SUBMERGE: S_THROUGH_GATE_1,
            S_SUBMERGE_CLEAR_TOP: S_THROUGH_GATE_1,
            S_THROUGH_GATE_1: S_FORWARD_TO_MARKER,
            S_FORWARD_TO_MARKER: S_STRAFE_MARKER_LEFT,
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
        self._substate_start = None     # generic intra-state timer
        self._marker_seen_this_state = False
        self._aligned_since = None      # gate continuously centred since...

        # Vision
        self._detections = []
        self._det_stamp = self.get_clock().now()

        # ── ROS I/O ─────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self.create_subscription(
            ObjectDetectionArray, 'vision/detections', self._vision_cb, 10)
        self.create_timer(1.0 / self.control_hz, self._tick)

        self._banner([
            'PREQUALIFICATION DRY TEST — logic only, print-only thrusters',
            f'  GATE   role  <- detector class "{self.gate_label}"  '
            '(point the camera at the POINTED NOSE)',
            f'  MARKER role  <- detector class "{self.marker_label}"  '
            '(point the camera at the ROUND NOSE)',
            f'  publish_commands={self.publish_commands}',
            'Move the camera to simulate motion: forward=surge, up=surface,',
            'down=submerge, left/right=strafe, rotate=yaw.',
        ])
        self.get_logger().info(f'State -> {self._state}')

    # ─── Callbacks ──────────────────────────────────────────────────

    def _vision_cb(self, msg: ObjectDetectionArray):
        try:
            self._detections = msg.detections
            self._det_stamp = self.get_clock().now()
        except Exception as e:  # noqa: BLE001 — never let a cb kill the node
            self.get_logger().error(f'Vision callback error: {e}')

    # ─── Helpers ────────────────────────────────────────────────────

    def _banner(self, lines):
        width = max(len(s) for s in lines) + 2
        bar = '=' * width
        self.get_logger().info('\n' + bar)
        for s in lines:
            self.get_logger().info(' ' + s)
        self.get_logger().info(bar)

    def _send(self, command, speed=0.0):
        if not self.publish_commands:
            return
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(max(0.0, min(1.0, speed)))
        msg.duration = 0.0
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

    def _transition(self, new_state):
        self._stop()
        self._state = new_state
        self._state_start = self.get_clock().now()
        self._substate_start = None
        self._marker_seen_this_state = False
        self._aligned_since = None
        self.get_logger().info(f'State -> {new_state}')

    def _center_horizontally(self, det, surge_when_centered=0.0):
        """Yaw to centre on ``det`` using its normalised image centre-x.

        Returns True when within ``center_tol`` of the image centre. While
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
        # Target right of centre -> rotate CW to bring it to centre.
        self._send('rotate_cw' if ex > 0 else 'rotate_ccw', speed)
        return False

    def _turn_left_until_marker(self):
        """One tick of a left (CCW) turn that stops once the marker is to the
        left of the frame, after a minimum sweep, capped at a timed ~90 deg
        (no pose on a hand-held dry test)."""
        elapsed = self._elapsed_substate()
        if elapsed >= self.turn_min_s:
            det = self._best_detection(self.marker_label)
            if det is not None and \
                    det.position.x <= self.marker_left_threshold:
                self.get_logger().info(
                    f'Turn left: marker now left (cx={det.position.x:.2f})')
                self._stop()
                return True
        if elapsed >= self.turn_90_duration_s:
            self.get_logger().info('Turn left: timed cap reached')
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
            timeout = self._state_timeouts.get(self._state)
            if timeout is not None and self._elapsed_in_state() > timeout:
                self.get_logger().warn(
                    f'State "{self._state}" timed out after {timeout:.0f}s '
                    f'— advancing')
                self._transition(self._timeout_next.get(self._state, S_SURFACE))
                return
            handler = getattr(self, '_do_' + self._state, None)
            if handler is None:
                self.get_logger().error(
                    f'No handler for "{self._state}" — surfacing')
                self._transition(S_SURFACE)
                return
            handler()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Tick error: {e} — stopping')
            self._stop()

    # ─── State handlers ─────────────────────────────────────────────

    def _do_submerge(self):
        # 1. Descend (move camera DOWN) until the GATE is detected, then pause.
        det = self._best_detection(self.gate_label)
        if det is not None:
            self._banner([
                f'PHASE 1 — POINTED NOSE detected (class "{self.gate_label}")',
                'Reading it as the FULL GATE. Pausing, then diving until the',
                'gate top leaves the frame (point so the gate rides HIGH).',
            ])
            self._stop()
            self._transition(S_SUBMERGE_CLEAR_TOP)
            return
        self._send('submerge', self.submerge_speed)

    def _do_submerge_clear_top(self):
        # 2. Dive a bit more until the gate top edge leaves the frame, then a
        #    short timed extra so the sub would pass under the top bar.
        if self._substate_start is not None:
            if self._elapsed_substate() >= self.gate_top_clear_extra_s:
                self.get_logger().info('Gate top cleared — driving through')
                self._transition(S_THROUGH_GATE_1)
                return
            self._send('submerge', self.submerge_speed)
            return
        det = self._best_detection(self.gate_label)
        top_out = (det is None) or \
            ((det.position.y - det.bbox_height / 2.0) <= self.gate_top_clear_y)
        if top_out:
            self._substate_start = self.get_clock().now()
        self._send('submerge', self.submerge_speed)

    def _do_through_gate_1(self):
        # 3. Centre on the GATE, then drive straight through (timed).
        self._drive_through_gate(S_FORWARD_TO_MARKER, banner=[
            'GATE cleared. AUV moving FORWARD. Waiting on the VERTICAL MARKER',
            f'(point the camera at the ROUND NOSE = "{self.marker_label}").',
        ])

    def _do_forward_to_marker(self):
        # 4. Surge forward until the MARKER (round_nose) appears, then pause.
        det = self._best_detection(self.marker_label)
        if det is not None:
            self._banner([
                f'PHASE 2 — ROUND NOSE detected (class "{self.marker_label}")',
                'Reading it as the VERTICAL MARKER. Beginning the',
                'around-the-marker maneuver.',
            ])
            self._stop()
            self._transition(S_STRAFE_MARKER_LEFT)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_strafe_marker_left(self):
        # 5. Strafe right until the marker is toward the left of the frame.
        det = self._best_detection(self.marker_label)
        if det is not None and det.position.x <= self.marker_left_threshold:
            self.get_logger().info(
                f'Marker now left (cx={det.position.x:.2f})')
            self._transition(S_FORWARD_PAST_MARKER)
            return
        self._send('strafe_right', self.strafe_speed)

    def _do_forward_past_marker(self):
        # 6. Surge forward until the marker is behind (seen then lost).
        if self._marker_behind():
            self.get_logger().info('Reached back of marker')
            self._transition(S_TURN_LEFT_1)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_turn_left_1(self):
        # 7. Turn left until the marker is to the left of the frame.
        if self._turn_left_until_marker():
            self._transition(S_FORWARD_MARKER_BEHIND_1)

    def _do_forward_marker_behind_1(self):
        # 8. Surge forward until the marker is behind.
        if self._marker_behind():
            self.get_logger().info('Marker behind (leg 1)')
            self._transition(S_TURN_LEFT_2)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_turn_left_2(self):
        # 9. Turn left until the marker is to the left of the frame.
        if self._turn_left_until_marker():
            self._transition(S_STRAFE_TO_GATE)

    def _do_strafe_to_gate(self):
        # 10. Strafe left until the GATE is re-detected, then pause.
        det = self._best_detection(self.gate_label)
        if det is not None:
            self._banner([
                'PHASE 3 — POINTED NOSE detected again '
                f'(class "{self.gate_label}")',
                'Reading it as the FULL GATE. Centering, then driving',
                'FORWARD THROUGH the Gate.',
            ])
            self._stop()
            self._transition(S_ALIGN_GATE)
            return
        self._send('strafe_left', self.strafe_speed)

    def _do_align_gate(self):
        # 11. Re-centre on the GATE.
        det = self._best_detection(self.gate_label)
        if det is None:
            # Creep forward to bring the gate back into view.
            self._send('surge_forward', self.surge_speed * 0.6)
            return
        if self._center_horizontally(det):
            self.get_logger().info('Gate aligned')
            self._transition(S_THROUGH_GATE_2)

    def _do_through_gate_2(self):
        # 12. Drive straight through the GATE (timed).
        self._drive_through_gate(S_FINAL_FORWARD, banner=[
            'PHASE 4 — AUV has moved THROUGH the Gate.',
            'Nudging forward to clear, then resurfacing.',
        ])

    def _do_final_forward(self):
        # 13. Surge a little more to fully clear the gate (move camera fwd).
        if self._elapsed_in_state() >= self.final_forward_duration_s:
            self.get_logger().info('Cleared gate — surfacing')
            self._transition(S_SURFACE)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_surface(self):
        # 14. Timed ascent — move the camera UP to simulate it.
        if self._elapsed_in_state() >= self.surface_duration_s:
            self._banner([
                'PHASE 5 — AUV has resurfaced.',
                '===== PREQUALIFICATION MISSION COMPLETE =====',
            ])
            self._stop()
            self._transition(S_DONE)
            return
        self._send('emerge', self.surface_speed)

    # ─── Shared gate-transit behaviour ──────────────────────────────

    def _drive_through_gate(self, next_state, banner=None):
        """Centre on the GATE, commit when close (or after a steady-centre
        dwell, since range/bbox may be unavailable hand-held), then drive
        straight through for ``gate_pass_duration_s``."""
        if self._substate_start is not None:
            if self._elapsed_substate() >= self.gate_pass_duration_s:
                if banner:
                    self._banner(banner)
                else:
                    self.get_logger().info('Cleared the gate')
                self._transition(next_state)
                return
            self._send('surge_forward', self.surge_speed)
            return

        det = self._best_detection(self.gate_label)
        if det is None:
            # Lost sight before committing — creep forward to reacquire.
            self._send('surge_forward', self.surge_speed * 0.6)
            self._aligned_since = None
            return
        centered = self._center_horizontally(det, self.surge_speed)
        close = ((det.bbox_width >= self.gate_close_bbox) or
                 (0.0 < det.position.z <= self.gate_close_range_m))
        if not centered:
            self._aligned_since = None
            return
        # Centred: commit when close, or after a short steady-centre dwell.
        if self._aligned_since is None:
            self._aligned_since = self.get_clock().now()
        dwell = (self.get_clock().now()
                 - self._aligned_since).nanoseconds / 1e9
        if close or dwell >= self.align_commit_dwell_s:
            self.get_logger().info('Centred on the Gate — committing to transit')
            self._substate_start = self.get_clock().now()


def main():
    rclpy.init()
    node = None
    try:
        node = PrequalDryTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        if node:
            node.get_logger().fatal(f'Unhandled exception: {e}')
        else:
            print(f'[prequal_dry_test_node] Fatal startup error: {e}')
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
