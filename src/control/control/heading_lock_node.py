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

Params (all live-tunable via `ros2 param set` except rate_hz/yaw_topic, which
require a restart and are rejected by the validation callback):
  - kp, ki, kd, i_limit         PID gains (>= 0, finite)
  - max_yaw_authority           correction clamp, 0 < x <= 1
  - max_forward_speed           surge clamp, 0 < x <= 1
  - stale_timeout_s             per-sample staleness age, 0 < x <= 2.0
  - grace_s                     continuous-stale abort timer, 0 < x <= 5.0
  - stale_window_s              sliding window for the stale DUTY CYCLE
                                 abort below, 0 < x <= 30.0 (default 3.0)
  - stale_duty_abort            abort if the fraction of stale ticks over
                                 stale_window_s exceeds this, 0 < x <= 1
                                 (default 0.5)
  - rate_hz, yaw_topic           restart-only (rejected if set live)

The timing ceilings above are SAFETY bounds, not taste — see
TIMING_PARAM_MAX: without a finite upper bound, one `ros2 param set` can
silently disable the staleness/abort contract entirely.

Safety: yaw stale > stale_timeout_s -> correction zeroed; still stale after
grace_s -> stop + unlock (blind forward is how the veer-right symptom hits
walls). Any tick exception -> stop + unlock. heave stays 0 so ALT_HOLD keeps
owning depth.

Degraded-source safety (stale DUTY CYCLE): a source that never goes fully
dead but keeps dropping under stale_timeout_s (e.g. a ZED brownout arriving
every 0.5-1.0s) never trips the grace_s abort above — each fresh sample
resets the continuous-stale clock, so the node would otherwise drive blind
forever with yaw_rate pinned near 0. This node tracks the fraction of stale
ticks over the trailing stale_window_s and aborts (stop + unlock) once that
fraction exceeds stale_duty_abort, independent of heading_lock.py's own
grace_s path (which still fires faster for a fully-dead source).
"""
import math
import time
from collections import deque

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

# Declared defaults, kept in one place so both __init__ and the
# on-set-parameters validator use the SAME fallback (never a bare 0.0 —
# a None/invalid value silently coerced to 0.0 would e.g. disable kp).
PARAM_DEFAULTS = {
    'kp': 1.2, 'ki': 0.0, 'kd': 0.3, 'i_limit': 0.3,
    'max_yaw_authority': 0.4, 'max_forward_speed': 0.6,
    'stale_timeout_s': 0.5, 'grace_s': 1.0,
    'stale_window_s': 3.0, 'stale_duty_abort': 0.5,
    'rate_hz': 20.0, 'yaw_topic': 'imu/rpy',
}

# Params that require a node restart to take effect; rejected if set live.
RESTART_ONLY_PARAMS = ('rate_hz', 'yaw_topic')

# Upper bounds on the timing params. These are SAFETY ceilings, not taste:
# a lower bound alone leaves the staleness/abort contract disabled by a
# single `ros2 param set` at the pool.
#   stale_timeout_s huge -> _fresh_yaw() never returns None -> the sub
#     steers on an arbitrarily old yaw sample and NOTHING ever goes stale,
#     so neither the grace_s abort nor the duty-cycle abort can ever fire.
#   grace_s huge -> arbitrarily long blind forward (yaw_rate pinned at 0)
#     after the source dies — this is how the veer-right symptom hits walls.
#   stale_window_s huge -> the duty-cycle window never fills, so the
#     degraded-source abort never fires.
# Ceilings are generous enough for any legitimate pool tuning; the point is
# that they are finite.
TIMING_PARAM_MAX = {
    'stale_timeout_s': 2.0,
    'grace_s': 5.0,
    'stale_window_s': 30.0,
}


def _pf(value, default):
    """Total float coercion for ROS param reads (matches autonomous_controller)."""
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


def _validate_param(name, value):
    """Return None if `value` is acceptable for param `name`, else a
    human-readable rejection reason. Validates the WHOLE incoming batch
    before anything is applied — see _on_params."""
    if name in RESTART_ONLY_PARAMS:
        return 'requires node restart'

    if name in ('kp', 'ki', 'kd', 'i_limit'):
        try:
            v = float(value)
        except (TypeError, ValueError):
            return f'{name} must be a finite number (got {value!r})'
        if not math.isfinite(v) or v < 0.0:
            return f'{name} must be finite and >= 0 (got {value!r})'
        return None

    if name in ('max_yaw_authority', 'max_forward_speed', 'stale_duty_abort'):
        try:
            v = float(value)
        except (TypeError, ValueError):
            return f'{name} must be a finite number (got {value!r})'
        if not (math.isfinite(v) and 0.0 < v <= 1.0):
            return f'{name} must satisfy 0 < {name} <= 1 (got {value!r})'
        return None

    if name in TIMING_PARAM_MAX:
        hi = TIMING_PARAM_MAX[name]
        try:
            v = float(value)
        except (TypeError, ValueError):
            return f'{name} must be a finite number (got {value!r})'
        if not (math.isfinite(v) and 0.0 < v <= hi):
            return (f'{name} must satisfy 0 < {name} <= {hi} '
                    f'(got {value!r})')
        return None

    return None  # unknown/undeclared param names: nothing to validate here


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
        self.declare_parameter('stale_window_s', 3.0)
        self.declare_parameter('stale_duty_abort', 0.5)
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
        self._stale_window_s = p('stale_window_s', 3.0)
        self._stale_duty_abort = p('stale_duty_abort', 0.5)

        self._last_yaw = None          # (arrival_monotonic_s, yaw_rad)
        self._last_tick = time.monotonic()
        self._warned_stale = False
        self._stale_samples = deque()  # (tick_monotonic_s, was_stale bool)
        self._stale_window_started_at = None  # anchor for "window is full"

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
        now = time.monotonic()
        yaw = self._fresh_yaw(now)
        if yaw is None:
            self.get_logger().error(
                'cmd refused — no fresh yaw to lock '
                '(is orientation_node publishing?)')
            self._publish_stop()
            return
        self._lock.start(yaw, speed)
        self._warned_stale = False
        self._stale_samples.clear()
        self._stale_window_started_at = now
        self.get_logger().info(
            f'heading LOCKED at {math.degrees(self._lock.target_yaw):+.1f}° '
            f'— forward {speed:.2f}')

    # ─── control tick ───────────────────────────────────────────────

    def _record_stale_sample(self, now, was_stale):
        """Append (now, was_stale) and evict entries older than
        stale_window_s. Called every tick while not IDLE."""
        self._stale_samples.append((now, was_stale))
        cutoff = now - self._stale_window_s
        while self._stale_samples and self._stale_samples[0][0] < cutoff:
            self._stale_samples.popleft()

    def _stale_duty_fraction(self, now):
        """Fraction of stale ticks in the trailing stale_window_s, or None
        if the window hasn't accumulated a full stale_window_s of history
        yet (avoids aborting on a partially-filled window right after
        lock).

        "Full" is measured against _stale_window_started_at (set at lock
        time), NOT against the deque's own span — the sliding eviction in
        _record_stale_sample always keeps the oldest sample's age just
        under stale_window_s by construction, so comparing span against
        the same threshold would never be true and the check would never
        fire.
        """
        if self._stale_window_started_at is None:
            return None
        if now - self._stale_window_started_at < self._stale_window_s:
            return None
        if not self._stale_samples:
            return None
        stale_count = sum(1 for _, s in self._stale_samples if s)
        return stale_count / len(self._stale_samples)

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if self._lock.state is LockState.IDLE:
            return
        try:
            yaw = self._fresh_yaw(now)
            self._record_stale_sample(now, yaw is None)
            duty_fraction = self._stale_duty_fraction(now)
            if duty_fraction is not None and duty_fraction > self._stale_duty_abort:
                self.get_logger().error(
                    f'yaw stale {duty_fraction:.0%} of last '
                    f'{self._stale_window_s:.1f}s (> {self._stale_duty_abort:.0%} '
                    'duty-cycle threshold) — STOP + unlock')
                self._lock.stop()
                self._publish_stop()
                return

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
            'error': self._lock.last_error if yaw is not None else float('nan'),
            'pid_output': yaw_rate,
            'motor1': right, 'motor2': left,
            'motor3': right, 'motor4': left,
        }
        for name, v in values.items():
            self._dbg[name].publish(Float32(data=float(v)))

    # ─── live tuning ────────────────────────────────────────────────

    def _on_params(self, params):
        """Validate the WHOLE incoming batch first; on the first violation
        reject the batch and apply NOTHING (partial application would leave
        e.g. a validated kp alongside a rejected, unchanged max_yaw_authority
        — silently inconsistent). Only once every param passes do we apply
        any of them. rate_hz/yaw_topic are always rejected: they only take
        effect at construction time, so accepting them live would silently
        lie about having applied the change.
        """
        for prm in params:
            reason = _validate_param(prm.name, prm.value)
            if reason is not None:
                return SetParametersResult(successful=False, reason=reason)

        for prm in params:
            name, val = prm.name, prm.value
            default = PARAM_DEFAULTS.get(name, 0.0)
            if name in ('kp', 'ki', 'kd'):
                self._pid.set_gains(**{name: _pf(val, default)})
            elif name == 'i_limit':
                self._pid.set_gains(i_limit=_pf(val, default))
            elif name == 'max_yaw_authority':
                self._lock.max_yaw_authority = _pf(val, default)
            elif name == 'grace_s':
                self._lock.grace_s = _pf(val, default)
            elif name == 'stale_timeout_s':
                self._stale_timeout_s = _pf(val, default)
            elif name == 'max_forward_speed':
                self._max_forward_speed = _pf(val, default)
            elif name == 'stale_window_s':
                self._stale_window_s = _pf(val, default)
            elif name == 'stale_duty_abort':
                self._stale_duty_abort = _pf(val, default)
            # rate_hz / yaw_topic are rejected above, never reached here.
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
