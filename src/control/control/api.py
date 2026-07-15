"""Auv — the operator façade. One API, used identically by mission scripts,
BehaviorTree action nodes, and interactive pool operation.

It holds no MAVLink and no control state: it publishes intent to motion_node
and watches submerge/state come back. Everything that actually decides anything
lives in motion_node and the pure controllers under it.

    from control.api import Auv

    with Auv() as auv:
        auv.submerge_to_depth(target_depth=2.0)
        auv.move_forward(speed=0.4, duration=10)   # depth+heading+attitude held
        auv.stop()

Speeds are normalized 0.0–1.0, matching MovementCommand. The original brief
wrote raw MAVLink units (dive_speed=-300, speed=400); this uses the project's
existing normalized convention rather than introducing a second one.
    
The descent rate lives on motion_node (`dive_speed`), not here — see
submerge_to_depth.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, String

from auv_msgs.msg import MovementCommand


class SubmergeError(RuntimeError):
    """The dive could not be completed. `reason` carries the autopilot's own
    explanation where there is one (e.g. "Depth sensor is not connected.")."""


class Auv:
    """Blocking operator API over motion_node's topics."""

    def __init__(self, node=None, spin_hz=50.0):
        self._owns_node = node is None
        if self._owns_node:
            if not rclpy.ok():
                rclpy.init()
            node = Node('auv_api')
        self._node = node
        self._spin_period = 1.0 / float(spin_hz)

        self._state = ''
        self._yaw = None                 # latest heading, radians, REP-103 CCW+
        self._cmd_pub = self._node.create_publisher(
            MovementCommand, 'motion/cmd', 10)
        self._submerge_pub = self._node.create_publisher(
            Float32, 'motion/submerge', 10)
        self._node.create_subscription(
            String, 'submerge/state', self._on_state, 10)
        # Same source motion_node's heading lock watches — closed-loop turn().
        self._node.create_subscription(
            Vector3Stamped, 'imu/rpy', self._on_yaw, 10)

    # ─── plumbing ───────────────────────────────────────────────────

    def _on_state(self, msg: String):
        self._state = msg.data

    def _on_yaw(self, msg: Vector3Stamped):
        self._yaw = float(msg.vector.z)

    def _spin(self, seconds):
        """Pump callbacks for `seconds`, so state updates actually arrive."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=self._spin_period)

    def _publish_axes(self, surge=0.0, strafe=0.0, yaw_rate=0.0):
        msg = MovementCommand()
        msg.command = 'axes'
        msg.surge = float(surge)
        msg.strafe = float(strafe)
        msg.yaw_rate = float(yaw_rate)
        # heave / roll / pitch stay 0: ALT_HOLD owns depth and self-levels, and
        # motion_node ignores them on this topic anyway.
        self._cmd_pub.publish(msg)

    # ─── the API ────────────────────────────────────────────────────

    @property
    def state(self):
        """Latest submerge/state string ('hold', 'diving', 'failed: …')."""
        return self._state

    def submerge_to_depth(self, target_depth, timeout=60.0):
        """Dive to `target_depth` metres and hold there. Blocks until the sub is
        at depth with ALT_HOLD confirmed and the heading captured.

        The descent rate is motion_node's `dive_speed` parameter — it is the
        node that runs DepthController, and a per-call rate here would have to
        be silently ignored or race the running dive. Set it where it lives:
        `ros2 param set /motion_node dive_speed 0.3`, or via the launch arg.

        Raises SubmergeError if the dive fails — a failed preflight, an ALT_HOLD
        the autopilot refuses (dead Bar02), a vehicle that never arms, or a dive
        that times out. The reason is the autopilot's own where it has one.
        """
        if not math.isfinite(target_depth) or target_depth <= 0.0:
            raise ValueError(
                f'target_depth must be a positive depth below the surface '
                f'(got {target_depth})')

        self._state = ''
        # motion_node's subscription may not be matched yet on a fresh node; a
        # Float32 published into the void is silently lost, so wait for the
        # connection rather than diving into a topic nobody is listening on.
        self._await_subscriber(self._submerge_pub, timeout=5.0)
        self._submerge_pub.publish(Float32(data=float(target_depth)))

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._spin(0.05)
            if self._state == 'hold':
                return
            if self._state.startswith('failed'):
                raise SubmergeError(self._state.split(':', 1)[-1].strip())
        raise SubmergeError(
            f'timed out after {timeout:.0f}s waiting to reach '
            f'{target_depth:.2f} m (last state: {self._state or "none"})')

    def _await_subscriber(self, publisher, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return
            self._spin(0.05)
        raise SubmergeError(
            'motion_node is not subscribed to motion/submerge — is it running, '
            'and is it inhibited by another movement_command publisher?')

    def move_forward(self, speed, duration):
        """Drive forward for `duration` seconds. Depth, heading and attitude are
        held automatically; only the forward channel comes from you."""
        self._move(surge=abs(float(speed)), duration=duration)

    def move_backward(self, speed, duration):
        self._move(surge=-abs(float(speed)), duration=duration)

    def move_right(self, speed, duration):
        self._move(strafe=abs(float(speed)), duration=duration)

    def move_left(self, speed, duration):
        self._move(strafe=-abs(float(speed)), duration=duration)

    def _move(self, surge=0.0, strafe=0.0, duration=0.0):
        deadline = time.monotonic() + float(duration)
        while time.monotonic() < deadline:
            # Re-published every cycle: motion_node holds the last operator
            # intent, and a stream keeps it honest if a message is dropped.
            self._publish_axes(surge=surge, strafe=strafe)
            self._spin(0.1)
            if self._state.startswith('failed'):
                self.stop()
                raise SubmergeError(self._state.split(':', 1)[-1].strip())
        self.stop()

    def turn(self, yaw_rate, duration=None, degrees=None):
        """Deliberately change heading. While yaw_rate is non-zero the heading
        lock stands down; when you stop, it re-captures the NEW heading and
        holds that.

        Two ways to bound the turn:
          • duration — spin for that many seconds (open-loop, original behaviour).
          • degrees  — spin until the heading has actually moved this many
            degrees (closed-loop off imu/rpy), then stop. `degrees` is a positive
            magnitude; the sign of yaw_rate picks the direction. When degrees is
            given, `duration` (default 60 s) is only a safety timeout.

        Supply exactly one of duration / degrees.
        """
        yaw_rate = float(yaw_rate)
        if degrees is None:
            if duration is None:
                raise ValueError('turn() needs either duration or degrees')
            deadline = time.monotonic() + float(duration)
            while time.monotonic() < deadline:
                self._publish_axes(yaw_rate=yaw_rate)
                self._spin(0.1)
            self.stop()
            return

        if yaw_rate == 0.0:
            raise ValueError('turn(degrees=...) needs a non-zero yaw_rate for direction')
        target = math.radians(abs(float(degrees)))
        timeout = 60.0 if duration is None else float(duration)

        # Need a heading fix before we can measure how far we've come.
        deadline = time.monotonic() + timeout
        while self._yaw is None and time.monotonic() < deadline:
            self._spin(0.1)
        if self._yaw is None:
            self.stop()
            raise SubmergeError('no heading on imu/rpy — cannot turn by degrees')

        direction = 1.0 if yaw_rate > 0.0 else -1.0
        prev = self._yaw
        travelled = 0.0                  # progress toward target, radians
        while travelled < target and time.monotonic() < deadline:
            self._publish_axes(yaw_rate=yaw_rate)
            self._spin(0.1)
            # Unwrap each step so ±pi rollover doesn't reset progress, and count
            # it in the commanded direction so sensor jitter cancels instead of
            # inflating progress the way abs() would.
            step = (self._yaw - prev + math.pi) % (2.0 * math.pi) - math.pi
            prev = self._yaw
            travelled += step * direction
        self.stop()

    def stop(self):
        """Stop translating. Depth hold and heading hold remain active."""
        msg = MovementCommand()
        msg.command = 'stop'
        self._cmd_pub.publish(msg)
        self._spin(0.1)

    def surface(self):
        """Release the dive: motion_node stops and unlocks. The vehicle stays in
        ALT_HOLD holding its current depth — this does NOT ascend. Disarm (or
        command an ascent yourself) to actually come up."""
        self._submerge_pub.publish(Float32(data=0.0))
        self._spin(0.2)

    # ─── lifecycle ──────────────────────────────────────────────────

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        if self._owns_node:
            self._node.destroy_node()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
