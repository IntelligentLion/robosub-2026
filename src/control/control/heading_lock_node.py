#!/usr/bin/env python3
# src/control/control/heading_lock_node.py
"""heading_lock_node — drive straight on the ZED 2i IMU yaw.

Subscribes:
  - imu/rpy           (geometry_msgs/Vector3Stamped) — vector.z = yaw rad,
                       REP-103 CCW+ (orientation_node); topic via yaw_topic
  - heading_lock/cmd  (std_msgs/Float32) — data > 0: lock current yaw, drive
                       forward at that speed (clamped to max_forward_speed);
                       repeat while locked = speed change, target kept;
                       data <= 0 or non-finite: stop + unlock

Publishes:
  - movement_command  (auv_msgs/MovementCommand) — 'axes' (surge+yaw_rate)
                       every tick while active; 'stop' on unlock/abort
  - heading_lock/{current_yaw,target_yaw,error,pid_output} (Float32, rad)
  - heading_lock/motor1..motor4 (Float32) — commanded INTENT into the mixer
                       (motor2=motor4=base+corr left, motor1=motor3=base-corr
                       right), NOT measured PWM (that belongs to ArduSub)

Control law lives in control.heading_lock (pure, unit-tested); this file is
wiring only: staleness detection (node-clock arrival age, immune to source
clock skew), live-tunable params, debug topics.

Safety: yaw stale > stale_timeout_s -> correction zeroed; still stale after
grace_s -> stop + unlock (blind forward is how the veer-right symptom hits
walls). Any tick exception -> stop + unlock. heave stays 0 so ALT_HOLD keeps
owning depth.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from auv_msgs.msg import MovementCommand

from control.heading_lock import HeadingLock, LockState
from control.pid import PID

DEBUG_TOPICS = ('current_yaw', 'target_yaw', 'error', 'pid_output',
                'motor1', 'motor2', 'motor3', 'motor4')


def _pf(value, default):
    """Total float coercion for ROS param reads (matches autonomous_controller)."""
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


class HeadingLockNode(Node):
    def __init__(self):
        super().__init__('heading_lock_node')
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)     # PD start; raise at the pool
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('i_limit', 0.3)
        self.declare_parameter('max_yaw_authority', 0.4)
        self.declare_parameter('max_forward_speed', 0.6)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('grace_s', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('yaw_topic', 'imu/rpy')

        def p(name, default):
            return _pf(self.get_parameter(name).value, default)

        self._pid = PID(kp=p('kp', 1.2), ki=p('ki', 0.0), kd=p('kd', 0.3),
                        limit=1.0, i_limit=p('i_limit', 0.3))
        self._lock = HeadingLock(
            self._pid,
            max_yaw_authority=p('max_yaw_authority', 0.4),
            grace_s=p('grace_s', 1.0))
        self._stale_timeout_s = p('stale_timeout_s', 0.5)
        self._max_forward_speed = p('max_forward_speed', 0.6)

        self._last_yaw = None          # (arrival_monotonic_s, yaw_rad)
        self._last_tick = time.monotonic()
        self._warned_stale = False

        self.add_on_set_parameters_callback(self._on_params)

        self._cmd_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self._dbg = {
            name: self.create_publisher(Float32, f'heading_lock/{name}', 10)
            for name in DEBUG_TOPICS}
        yaw_topic = str(self.get_parameter('yaw_topic').value)
        self.create_subscription(
            Vector3Stamped, yaw_topic, self._on_yaw, 10)
        self.create_subscription(
            Float32, 'heading_lock/cmd', self._on_cmd, 10)
        rate_hz = max(1.0, p('rate_hz', 20.0))
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'heading_lock_node up — yaw from "{yaw_topic}", '
            f'{rate_hz:.0f} Hz, authority ±{self._lock.max_yaw_authority}')

    # ─── inputs ─────────────────────────────────────────────────────

    def _on_yaw(self, msg: Vector3Stamped):
        if math.isfinite(msg.vector.z):
            self._last_yaw = (time.monotonic(), msg.vector.z)

    def _fresh_yaw(self, now_s):
        """Latest yaw, or None if stale/never seen (arrival-time based)."""
        if self._last_yaw is None:
            return None
        arrival, yaw = self._last_yaw
        if now_s - arrival > self._stale_timeout_s:
            return None
        return yaw

    def _on_cmd(self, msg: Float32):
        speed = msg.data
        if not math.isfinite(speed) or speed <= 0.0:
            if self._lock.state is not LockState.IDLE:
                self.get_logger().info('cmd <= 0 — stop + unlock')
            self._lock.stop()
            self._publish_stop()
            return
        speed = min(speed, self._max_forward_speed)
        if self._lock.state in (LockState.LOCKED, LockState.STALE_GRACE):
            self._lock.set_base_speed(speed)     # speed change, target kept
            return
        yaw = self._fresh_yaw(time.monotonic())
        if yaw is None:
            self.get_logger().error(
                'cmd refused — no fresh yaw to lock '
                '(is orientation_node publishing?)')
            self._publish_stop()
            return
        self._lock.start(yaw, speed)
        self._warned_stale = False
        self.get_logger().info(
            f'heading LOCKED at {math.degrees(self._lock.target_yaw):+.1f}° '
            f'— forward {speed:.2f}')

    # ─── control tick ───────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if self._lock.state is LockState.IDLE:
            return
        try:
            yaw = self._fresh_yaw(now)
            surge, yaw_rate, state = self._lock.update(yaw, now, dt)

            if state is LockState.ABORTED:
                self.get_logger().error(
                    f'yaw stale > {self._lock.grace_s:.1f}s grace — '
                    'STOP + unlock')
                self._lock.stop()
                self._publish_stop()
                return
            if state is LockState.STALE_GRACE and not self._warned_stale:
                self.get_logger().warn(
                    'yaw stale — correction zeroed, forward continues '
                    f'{self._lock.grace_s:.1f}s grace')
                self._warned_stale = True
            elif state is LockState.LOCKED:
                self._warned_stale = False

            out = MovementCommand()
            out.command = 'axes'
            out.surge = float(surge)
            out.yaw_rate = float(yaw_rate)
            self._cmd_pub.publish(out)
            self._publish_debug(yaw, surge, yaw_rate)
        except Exception as e:
            self.get_logger().error(f'tick error: {e} — stop + unlock')
            self._lock.stop()
            self._publish_stop()

    # ─── outputs ────────────────────────────────────────────────────

    def _publish_stop(self):
        msg = MovementCommand()
        msg.command = 'stop'
        self._cmd_pub.publish(msg)

    def _publish_debug(self, yaw, surge, yaw_rate):
        left = surge + yaw_rate       # motors 2 (FL) & 4 (RL) intent
        right = surge - yaw_rate      # motors 1 (FR) & 3 (RR) intent
        values = {
            'current_yaw': yaw if yaw is not None else float('nan'),
            'target_yaw': self._lock.target_yaw,
            'error': self._lock.last_error,
            'pid_output': yaw_rate,
            'motor1': right, 'motor2': left,
            'motor3': right, 'motor4': left,
        }
        for name, v in values.items():
            self._dbg[name].publish(Float32(data=float(v)))

    # ─── live tuning ────────────────────────────────────────────────

    def _on_params(self, params):
        for prm in params:
            name, val = prm.name, prm.value
            if name in ('kp', 'ki', 'kd'):
                self._pid.set_gains(**{name: _pf(val, 0.0)})
            elif name == 'i_limit':
                self._pid.set_gains(i_limit=_pf(val, 0.3))
            elif name == 'max_yaw_authority':
                self._lock.max_yaw_authority = _pf(val, 0.4)
            elif name == 'grace_s':
                self._lock.grace_s = _pf(val, 1.0)
            elif name == 'stale_timeout_s':
                self._stale_timeout_s = _pf(val, 0.5)
            elif name == 'max_forward_speed':
                self._max_forward_speed = _pf(val, 0.6)
            # rate_hz / yaw_topic changes need a restart; accepted silently
        return SetParametersResult(successful=True)

    def destroy_node(self):
        # Best effort: leave the thruster node commanding neutral.
        try:
            self._publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadingLockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
