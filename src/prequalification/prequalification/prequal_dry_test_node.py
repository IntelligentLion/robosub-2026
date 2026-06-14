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

Mapped mission flow (the five phases you asked for)
---------------------------------------------------
  1. Point camera at the POINTED NOSE  -> read as the FULL GATE.
       => "centering toward the Gate and moving forward, waiting on the
           Vertical Marker."
  2. Point camera at the ROUND NOSE    -> read as the VERTICAL MARKER.
       => "moving around the Marker, waiting on the Gate."
  3. Point camera at the POINTED NOSE again -> the FULL GATE again.
       => "centering toward the Gate and moving forward through the Gate."
  4. Drive-through completes.
       => "moved through the Gate, resurfacing."
  5. Surface completes.
       => "MISSION COMPLETE."

This mirrors the real prequalification algorithm (submerge, detect gate, through
gate, find vertical marker, circle it, turn back, realign to gate, through gate,
resurface) — the marker-circling maneuver is collapsed into a single
"moving around the Marker" phase whose exit is re-detecting the gate.

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


# ─── Mission states (collapsed dry-test flow) ───────────────────────────────
S_SUBMERGE = 'submerge'                 # brief timed descent (move camera down)
S_DETECT_GATE_1 = 'detect_gate_1'       # find + centre on the GATE (pointed)
S_FORWARD_TO_MARKER = 'forward_to_marker'   # surge; wait for the MARKER (round)
S_AROUND_MARKER = 'around_marker'       # circle the MARKER; wait for the GATE
S_ALIGN_GATE_2 = 'align_gate_2'         # re-centre on the GATE
S_THROUGH_GATE = 'through_gate'         # timed drive through the GATE
S_SURFACE = 'surface'                   # timed ascent (move camera up)
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
        # Gate is "close enough to commit to transit" when its bbox fills this
        # fraction of the frame, or its range drops below this many metres, or
        # it has simply been centred continuously for align_commit_dwell_s
        # (range/bbox may be unavailable on a hand-held dry test).
        p('gate_close_bbox', 0.45)
        p('gate_close_range_m', 1.2)
        p('align_commit_dwell_s', 1.5)
        # Timed phases (no depth feedback on a hand-held dry test).
        p('submerge_duration_s', 3.0)
        p('gate_pass_duration_s', 5.0)
        p('surface_duration_s', 5.0)
        # Minimum time spent "around the marker" before the gate can re-trigger
        # — stops an instant skip if both objects are briefly in frame.
        p('min_around_s', 3.0)
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
        self.gate_close_bbox = float(g('gate_close_bbox').value)
        self.gate_close_range_m = float(g('gate_close_range_m').value)
        self.align_commit_dwell_s = float(g('align_commit_dwell_s').value)
        self.submerge_duration_s = float(g('submerge_duration_s').value)
        self.gate_pass_duration_s = float(g('gate_pass_duration_s').value)
        self.surface_duration_s = float(g('surface_duration_s').value)
        self.min_around_s = float(g('min_around_s').value)
        self.control_hz = max(1.0, float(g('control_hz').value))
        self.publish_commands = bool(g('publish_commands').value)

        # ── State ───────────────────────────────────────────────────
        self._state = S_SUBMERGE
        self._state_start = self.get_clock().now()
        self._substate_start = None     # generic intra-state timer
        self._gate_seen = False         # logged the gate-detect banner yet?
        self._marker_seen = False
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

    # ─── Main control loop ──────────────────────────────────────────

    def _tick(self):
        try:
            if self._state == S_DONE:
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
        # Brief timed descent — move the camera DOWN to simulate it.
        if self._elapsed_in_state() >= self.submerge_duration_s:
            self.get_logger().info('Reached run depth (timed)')
            self._transition(S_DETECT_GATE_1)
            return
        self._send('submerge', self.submerge_speed)

    def _do_detect_gate_1(self):
        # PHASE 1: find the GATE (pointed_nose) and centre on it.
        det = self._best_detection(self.gate_label)
        if det is None:
            self._stop()
            return
        if not self._gate_seen:
            self._gate_seen = True
            self._banner([
                f'PHASE 1 — POINTED NOSE detected (class "{self.gate_label}")',
                'Reading it as the FULL GATE. Centering on the Gate...',
            ])
        if self._center_horizontally(det):
            self._banner([
                'GATE centred. AUV is centering toward the Gate and moving',
                'FORWARD. Waiting on the VERTICAL MARKER',
                f'(point the camera at the ROUND NOSE = "{self.marker_label}").',
            ])
            self._transition(S_FORWARD_TO_MARKER)

    def _do_forward_to_marker(self):
        # PHASE 2a: surge forward until the MARKER (round_nose) appears.
        det = self._best_detection(self.marker_label)
        if det is not None:
            self._marker_seen = True
            self._banner([
                f'PHASE 2 — ROUND NOSE detected (class "{self.marker_label}")',
                'Reading it as the VERTICAL MARKER. AUV is moving AROUND the',
                'Marker. Waiting on the GATE',
                f'(point the camera at the POINTED NOSE = "{self.gate_label}").',
            ])
            self._transition(S_AROUND_MARKER)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_around_marker(self):
        # PHASE 2b: circle the MARKER until the GATE (pointed_nose) re-appears.
        # The real algorithm strafes right, drives past, turns left 90, drives,
        # turns left 90 to re-face the gate. We replay that as a looping
        # maneuver so the thruster console shows realistic motion; the phase
        # exits when the gate is re-detected.
        t = self._elapsed_in_state()
        if t >= self.min_around_s:
            gate = self._best_detection(self.gate_label)
            if gate is not None:
                self._banner([
                    'PHASE 3 — POINTED NOSE detected again '
                    f'(class "{self.gate_label}")',
                    'Reading it as the FULL GATE. AUV is centering toward the',
                    'Gate and will move FORWARD THROUGH the Gate.',
                ])
                self._transition(S_ALIGN_GATE_2)
                return

        # Looping "around the marker" maneuver: strafe right, surge, yaw left.
        phase = t % 6.0
        if phase < 2.0:
            self._send('strafe_right', self.strafe_speed)
        elif phase < 4.0:
            self._send('surge_forward', self.surge_speed)
        else:
            self._send('rotate_ccw', self.turn_speed)

    def _do_align_gate_2(self):
        # PHASE 3: re-centre on the GATE, then commit to driving through.
        det = self._best_detection(self.gate_label)
        if det is None:
            # Creep forward to bring the gate back into view.
            self._send('surge_forward', self.surge_speed * 0.6)
            self._aligned_since = None
            return
        centered = self._center_horizontally(det, self.surge_speed)
        close = ((det.bbox_width >= self.gate_close_bbox) or
                 (0.0 < det.position.z <= self.gate_close_range_m))
        if not centered:
            self._aligned_since = None
            return
        # Centred: commit when close, or after a short steady-centre dwell
        # (range/bbox may be unavailable on a hand-held dry test).
        if self._aligned_since is None:
            self._aligned_since = self.get_clock().now()
        dwell = (self.get_clock().now()
                 - self._aligned_since).nanoseconds / 1e9
        if close or dwell >= self.align_commit_dwell_s:
            self.get_logger().info('Centred on the Gate — committing to transit')
            self._transition(S_THROUGH_GATE)

    def _do_through_gate(self):
        # PHASE 3 (cont.): drive straight through the GATE for a fixed time
        # (the gate leaves the frame as the sub passes through it).
        if self._elapsed_substate() >= self.gate_pass_duration_s:
            self._banner([
                'PHASE 4 — AUV has moved THROUGH the Gate.',
                'Resurfacing (move the camera UP to simulate ascent).',
            ])
            self._transition(S_SURFACE)
            return
        self._send('surge_forward', self.surge_speed)

    def _do_surface(self):
        # PHASE 4 (cont.): timed ascent — move the camera UP to simulate it.
        if self._elapsed_in_state() >= self.surface_duration_s:
            self._banner([
                'PHASE 5 — AUV has resurfaced.',
                '===== PREQUALIFICATION MISSION COMPLETE =====',
            ])
            self._stop()
            self._transition(S_DONE)
            return
        self._send('emerge', self.surface_speed)


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
