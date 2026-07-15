#!/usr/bin/env python3
"""motion_node — THE centralized movement node.

Sole publisher of `movement_command`. Everything that wants the sub to move
goes through here. That is not a style preference: heading_lock_node,
autonomous_controller and the standalone field_common scripts all published
that topic directly, so any two running at once fought over the thrusters with
no arbiter. On startup and periodically this node counts publishers on
movement_command and INHIBITS itself if it is not alone.

Division of labour, top to bottom:

  ArduSub (ALT_HOLD)  depth, roll, pitch      — its controllers, not ours
  SubmergeController  preflight/mode/arm/dive sequencing
  DepthController     the dive only
  HeadingController   yaw — the one gap ArduSub has no mode for
  MotionController    which axis may come from where

Subscribes
  imu/rpy          (Vector3Stamped)  vector.z = yaw, REP-103 CCW+; `yaw_topic`
  pixhawk/depth    (Float32)         metres, +down, NaN when unavailable
  pixhawk/mode     (String)          the mode the vehicle is ACTUALLY in
  pixhawk/armed    (Bool)
  motion/cmd       (MovementCommand) operator surge/strafe only
  motion/submerge  (Float32)         target depth m; <= 0 aborts + releases

Publishes
  movement_command (MovementCommand) 'axes' while active, 'stop' on abort/idle
  submerge/state   (String)          phase, or the failure reason
  heading/{current,target,error,yaw_correction}, depth/{current,target},
  motion/{forward_cmd,vertical_cmd}  (Float32) — for rqt_plot and RViz

Safety, all in one place. The three loss paths deliberately differ, because the
right answer differs:
  yaw stale            → correction 0 immediately, forward continues grace_s,
                         then stop. Depth hold is untouched (heave stays 0).
  yaw degraded         → stale-duty-cycle abort, latched until acknowledged.
  depth stale/NaN      → stop movement. Stay in ALT_HOLD — the autopilot may
                         still be holding. Never dive.
  mode leaves ALT_HOLD → depth hold is GONE. Stop movement.
  tick exception       → stop + unlock.
"""
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from auv_msgs.msg import MovementCommand
from auv_msgs.srv import SetFlightMode

from control.depth_controller import DepthController
from control.heading_lock import HeadingLock, LockState
from control.motion import MotionController
from control.pid import PID
from control.submerge import Effects, SubmergeController, SubmergeState

DEBUG_TOPICS = ('heading/current', 'heading/target', 'heading/error',
                'heading/yaw_correction', 'depth/current', 'depth/target',
                'motion/forward_cmd', 'motion/vertical_cmd')

# THE source of truth for every declared param's default. __init__ declares from
# this dict and reads its fallbacks from it, and _on_params applies from it — so
# a default lives in exactly one place and cannot drift between the declaration,
# the initial read, and the live-tune path.
PARAM_DEFAULTS = {
    # No 'forward_speed' here: surge is the OPERATOR's axis, and it arrives on
    # motion/cmd. A default surge parameter on this node would be read by
    # nothing, and reads like an autostart that does not exist. The mission's
    # forward speed lives on forward_hold_mission; the safety clamp on it is
    # max_forward_speed, below.
    'target_depth': 2.0, 'dive_speed': 0.3,
    'depth_tolerance_m': 0.15, 'min_heave': 0.12, 'depth_timeout': 30.0,
    'phase_timeout_s': 15.0,
    'heading_kp': 1.2, 'heading_ki': 0.0, 'heading_kd': 0.3, 'i_limit': 0.3,
    'max_yaw_correction': 0.4, 'max_forward_speed': 0.6,
    'stale_timeout_s': 0.5, 'grace_s': 1.0, 'depth_stale_timeout_s': 2.0,
    'stale_window_s': 3.0, 'stale_duty_abort': 0.5,
    'control_rate_hz': 20.0, 'yaw_topic': 'imu/rpy',
}

# Params that only take effect at construction; rejected if set live, rather
# than accepted while silently not applying.
RESTART_ONLY_PARAMS = ('control_rate_hz', 'yaw_topic')

# Valid range per param, as (low, high, low_inclusive, high_inclusive).
#
# CANONICAL RATIONALE (ported from heading_lock_node — do not restate it
# elsewhere). These are SAFETY bounds, not taste. A missing or wrong-sided bound
# lets a single `ros2 param set` at the pool silently disable the
# staleness/abort contract while reporting success:
#   max_yaw_correction <= 0 -> the clamp becomes max(hi, min(lo, c)) = a
#     CONSTANT full-authority yaw, i.e. the sub spins regardless of error.
#   max_forward_speed > 1  -> surge published outside MovementCommand's
#     documented [-1, 1].
#   stale_timeout_s too big -> _fresh_yaw() never returns None, so the sub
#     steers on an arbitrarily old yaw sample and NOTHING ever goes stale:
#     neither the grace abort nor the duty abort can fire again.
#   grace_s too big -> arbitrarily long blind forward (yaw_rate pinned at 0)
#     after the source dies — this is how the veer-right symptom hits walls.
#   stale_window_s too big -> the duty window never fills, so the
#     degraded-source abort never fires.
#   stale_duty_abort >= 1 -> the abort test is `fraction > stale_duty_abort` and
#     fraction is bounded above by 1.0, so a value of exactly 1.0 makes the duty
#     abort UNREACHABLE even at 100% stale ticks. Hence the strict upper bound
#     here, unlike the other unit-interval params.
#   depth_stale_timeout_s too big -> the sub keeps driving on a stale depth
#     after the Bar02 drops off I2C, which is exactly when ALT_HOLD has already
#     stopped holding.
#   min_heave <= 0 -> a small dive_speed lands inside ArduSub's throttle
#     deadzone and becomes no command at all: a dive that never starts and never
#     reports why.
# Ceilings are generous enough for any legitimate pool tuning; the point is that
# they are finite and on the correct side.
PARAM_BOUNDS = {
    'target_depth':          (0.0, 30.0,     False, True),
    'dive_speed':            (0.0, 1.0,      False, True),
    'depth_tolerance_m':     (0.0, 1.0,      False, True),
    'min_heave':             (0.0, 1.0,      False, True),
    'depth_timeout':         (0.0, 300.0,    False, True),
    'phase_timeout_s':       (0.0, 120.0,    False, True),
    'heading_kp':            (0.0, math.inf, True,  True),
    'heading_ki':            (0.0, math.inf, True,  True),
    'heading_kd':            (0.0, math.inf, True,  True),
    'i_limit':               (0.0, math.inf, True,  True),
    'max_yaw_correction':    (0.0, 1.0,      False, True),
    'max_forward_speed':     (0.0, 1.0,      False, True),
    'stale_duty_abort':      (0.0, 1.0,      False, False),   # strict: see above
    'stale_timeout_s':       (0.0, 2.0,      False, True),
    'depth_stale_timeout_s': (0.0, 5.0,      False, True),
    'grace_s':               (0.0, 5.0,      False, True),
    'stale_window_s':        (0.0, 30.0,     False, True),
}


def _pf(value, default):
    """Total float coercion for ROS param reads."""
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


def _validate_param(name, value):
    """None if acceptable, else a human-readable rejection reason naming the
    bound (so a tuner at the pool sees WHY the set was refused)."""
    if name in RESTART_ONLY_PARAMS:
        return 'requires node restart'

    bounds = PARAM_BOUNDS.get(name)
    if bounds is None:
        return None                    # unknown/undeclared: nothing to check

    lo, hi, lo_inc, hi_inc = bounds
    try:
        v = float(value)
    except (TypeError, ValueError):
        return f'{name} must be a finite number (got {value!r})'
    if not math.isfinite(v):
        return f'{name} must be finite (got {value!r})'

    lo_ok = (v >= lo) if lo_inc else (v > lo)
    hi_ok = (v <= hi) if hi_inc else (v < hi)
    if lo_ok and hi_ok:
        return None

    lo_op = '<=' if lo_inc else '<'
    hi_op = '<=' if hi_inc else '<'
    hi_txt = '' if hi == math.inf else f' {hi_op} {hi}'
    return f'{name} must satisfy {lo} {lo_op} {name}{hi_txt} (got {value!r})'


class _GatewayEffects(Effects):
    """SubmergeController's side effects, wired to the gateway's services.

    Request/poll rather than blocking calls: this runs inside a ROS timer
    callback, and waiting on a service future in one deadlocks the executor.
    """

    def __init__(self, node):
        self._node = node

    def request_preflight(self):
        self._node._request_preflight()

    def preflight_result(self):
        return self._node._preflight_result

    def request_mode(self, name):
        self._node._request_mode(name)

    def mode_result(self):
        return self._node._mode_result

    def is_armed(self):
        return self._node._armed


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        def p(name):
            return _pf(self.get_parameter(name).value, PARAM_DEFAULTS[name])

        # NOT `self._clock`: rclpy.Node already owns that attribute and
        # create_timer() dereferences it. Shadowing it makes every timer
        # construction fail with 'builtin_function_or_method has no
        # attribute handle'.
        self._now = time.monotonic            # injectable for tests

        self._pid = PID(kp=p('heading_kp'), ki=p('heading_ki'),
                        kd=p('heading_kd'), limit=1.0, i_limit=p('i_limit'))
        self._heading = HeadingLock(
            self._pid,
            max_yaw_authority=p('max_yaw_correction'),
            grace_s=p('grace_s'))
        self._depth_ctl = DepthController(
            tolerance_m=p('depth_tolerance_m'),
            min_heave=p('min_heave'),
            timeout_s=p('depth_timeout'))
        self._mixer = MotionController(max_surge=p('max_forward_speed'))
        self._submerge = SubmergeController(
            self._depth_ctl, self._heading, _GatewayEffects(self),
            phase_timeout_s=p('phase_timeout_s'))

        self._stale_timeout_s = p('stale_timeout_s')
        self._depth_stale_timeout_s = p('depth_stale_timeout_s')
        self._max_forward_speed = p('max_forward_speed')
        self._stale_window_s = p('stale_window_s')
        self._stale_duty_abort = p('stale_duty_abort')
        self._default_dive_speed = p('dive_speed')

        self._last_yaw = None                 # (arrival_s, yaw_rad)
        self._last_depth = None               # (arrival_s, depth_m)
        self._mode = 'UNKNOWN'
        self._armed = False
        self._op_surge = 0.0
        self._op_strafe = 0.0
        self._op_yaw = None
        self._last_tick = self._now()
        self._warned_stale = False
        self._stale_samples = deque()
        self._stale_window_started_at = None
        self._duty_aborted = False
        self._inhibited = False

        self._preflight_result = None         # None = pending; (ok, reason)
        self._mode_result = None
        self._mode_requests = []

        # MUST stay AFTER the declare_parameter calls: in rclpy Humble,
        # declare_parameter invokes registered on-set callbacks with
        # raise_on_failure=True, so registering first would make _validate_param's
        # unconditional rejection of the restart-only params raise at
        # construction and break `--ros-args -p yaw_topic:=...`.
        self.add_on_set_parameters_callback(self._on_params)

        self._cmd_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self._state_pub = self.create_publisher(String, 'submerge/state', 10)
        self._dbg = {t: self.create_publisher(Float32, t, 10)
                     for t in DEBUG_TOPICS}

        yaw_topic = str(self.get_parameter('yaw_topic').value)
        self.create_subscription(Vector3Stamped, yaw_topic, self._on_yaw, 10)
        self.create_subscription(Float32, 'pixhawk/depth', self._on_depth, 10)
        self.create_subscription(String, 'pixhawk/mode', self._on_mode, 10)
        self.create_subscription(Bool, 'pixhawk/armed', self._on_armed, 10)
        self.create_subscription(
            MovementCommand, 'motion/cmd', self._on_cmd, 10)
        self.create_subscription(
            Float32, 'motion/submerge', self._on_submerge, 10)
        self.create_subscription(
            Float32, 'motion/heading', self._on_heading, 10)

        self._preflight_cli = self.create_client(Trigger, 'pixhawk/preflight')
        self._mode_cli = self.create_client(SetFlightMode, 'pixhawk/set_mode')

        rate_hz = max(1.0, p('control_rate_hz'))
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(2.0, self._check_sole_publisher)

        self.get_logger().info(
            f'motion_node up — yaw from "{yaw_topic}", {rate_hz:.0f} Hz, '
            f'yaw authority ±{self._heading.max_yaw_authority}')

    # ─── sole-publisher guard ───────────────────────────────────────

    def _count_movement_publishers(self):
        return self.count_publishers('movement_command')

    def _check_sole_publisher(self):
        """Two publishers on movement_command means two things fighting over the
        thrusters with no arbiter. Refuse to be one of them."""
        others = self._count_movement_publishers() - 1      # minus our own
        if others > 0 and not self._inhibited:
            self._inhibited = True
            self.get_logger().error(
                f'{others} other publisher(s) on movement_command — INHIBITED. '
                'motion_node must be the only one. Stop autonomous_controller / '
                'any field_common script before running this.')
            self._abort('another movement_command publisher is active')
        elif others <= 0 and self._inhibited:
            self._inhibited = False
            self.get_logger().info(
                'movement_command is ours alone again — re-enabled')

    # ─── inputs ─────────────────────────────────────────────────────

    def _on_yaw(self, msg: Vector3Stamped):
        if math.isfinite(msg.vector.z):
            self._last_yaw = (self._now(), msg.vector.z)

    def _on_depth(self, msg: Float32):
        if math.isfinite(msg.data):
            self._last_depth = (self._now(), float(msg.data))

    def _on_mode(self, msg: String):
        self._mode = msg.data

    def _on_armed(self, msg: Bool):
        self._armed = bool(msg.data)

    def _fresh_yaw(self, now_s):
        """Latest yaw, or None if stale/never seen. Arrival-time based, so it is
        immune to clock skew on the publishing side."""
        if self._last_yaw is None:
            return None
        arrival, yaw = self._last_yaw
        return None if (now_s - arrival) > self._stale_timeout_s else yaw

    def _fresh_depth(self, now_s):
        if self._last_depth is None:
            return None
        arrival, depth = self._last_depth
        return (None if (now_s - arrival) > self._depth_stale_timeout_s
                else depth)

    def _on_cmd(self, msg: MovementCommand):
        """Operator intent. Only surge and strafe are honoured."""
        if msg.command == 'stop':
            self._op_surge = 0.0
            self._op_strafe = 0.0
            self._op_yaw = None
            return
        if msg.heave != 0.0:
            self.get_logger().warn(
                'motion/cmd heave ignored — ALT_HOLD owns depth')
        if msg.roll_rate != 0.0 or msg.pitch_rate != 0.0:
            self.get_logger().warn(
                'motion/cmd roll/pitch ignored — ALT_HOLD self-levels')
        self._op_surge = float(msg.surge)
        self._op_strafe = float(msg.strafe)
        # yaw_rate == 0 means "not steering, let the lock hold"; anything else is
        # a deliberate heading change and overrides the lock while it persists.
        self._op_yaw = float(msg.yaw_rate) if msg.yaw_rate != 0.0 else None

    def _on_heading(self, msg: Float32):
        """Command an ABSOLUTE heading (rad, REP-103 CCW+) for the lock to slew
        to and hold. Only honoured while HOLDING — a heading command mid-dive
        would fight the sequencing, and one while idle drives nothing. This is
        the deliberate-slew path the auto-tuner uses for a clean step response;
        operator yaw_rate on motion/cmd still overrides it while active."""
        target = float(msg.data)
        if not math.isfinite(target):
            self.get_logger().warn('motion/heading non-finite — ignored')
            return
        if self._inhibited:
            self.get_logger().warn(
                'motion/heading ignored — another movement_command publisher '
                'is active')
            return
        if self._submerge.state is not SubmergeState.HOLD:
            self.get_logger().warn(
                f'motion/heading ignored — not HOLDING (state '
                f'{self._submerge.state.value})')
            return
        # Drop any operator-yaw override so the PID actually drives to the new
        # target instead of the mixer re-capturing current heading each tick.
        self._op_yaw = None
        self._heading.set_target(target)
        self.get_logger().info(f'heading target → {target:.3f} rad')

    def _on_submerge(self, msg: Float32):
        target = float(msg.data)
        if not math.isfinite(target) or target <= 0.0:
            self.get_logger().info('submerge <= 0 — abort + release')
            self._submerge.stop()
            self._duty_aborted = False              # the operator ack
            self._op_surge = self._op_strafe = 0.0
            self._op_yaw = None
            self._publish_stop()
            return
        if self._inhibited:
            self.get_logger().error(
                'submerge refused — another movement_command publisher is active')
            return
        # A latched duty abort means the yaw source was measurably degraded, not
        # merely momentarily late. Re-arming here would hand a repeating
        # publisher a fresh (partially-filled, therefore un-abortable) stale
        # window every time: stop / drive / stop, forever. Require the ack.
        if self._duty_aborted:
            self.get_logger().error(
                'submerge refused — yaw source degraded, duty-cycle abort '
                'latched. Send motion/submerge <= 0 to acknowledge first.')
            return
        now = self._now()
        self._preflight_result = None
        self._mode_result = None
        self._mode_requests = []
        self._stale_samples.clear()
        self._stale_window_started_at = now
        self._warned_stale = False
        self._submerge.start(target, self._default_dive_speed, now)
        self.get_logger().info(f'submerging to {target:.2f} m')

    # ─── gateway service calls (request / poll, never block) ─────────

    def _request_preflight(self):
        if not self._preflight_cli.service_is_ready():
            self.get_logger().warn(
                'pixhawk/preflight not available — is thruster_node up?')
            return
        fut = self._preflight_cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_preflight_done)

    def _on_preflight_done(self, future):
        try:
            r = future.result()
            self._preflight_result = (bool(r.success), r.message)
        except Exception as exc:
            self._preflight_result = (False, f'preflight call failed: {exc}')

    def _request_mode(self, name):
        self._mode_requests.append(name)
        if not self._mode_cli.service_is_ready():
            self.get_logger().warn(
                'pixhawk/set_mode not available — is thruster_node up?')
            return
        req = SetFlightMode.Request()
        req.mode = name
        fut = self._mode_cli.call_async(req)
        fut.add_done_callback(self._on_mode_done)

    def _on_mode_done(self, future):
        try:
            r = future.result()
            self._mode_result = (bool(r.success), r.reason)
        except Exception as exc:
            self._mode_result = (False, f'set_mode call failed: {exc}')

    # ─── control tick ───────────────────────────────────────────────

    def _record_stale_sample(self, now, was_stale):
        self._stale_samples.append((now, was_stale))
        cutoff = now - self._stale_window_s
        while self._stale_samples and self._stale_samples[0][0] < cutoff:
            self._stale_samples.popleft()

    def _stale_duty_fraction(self, now):
        """Fraction of stale ticks in the trailing stale_window_s, or None if the
        window has not accumulated a full stale_window_s of history yet.

        A source that never dies but keeps dropping under stale_timeout_s (a ZED
        or I2C brownout arriving every 0.5-1.0 s) never trips the grace abort —
        each fresh sample resets the continuous-stale clock — so it would
        otherwise drive blind forever with yaw pinned near 0. This is the check
        that catches it.

        "Full" is measured against _stale_window_started_at, NOT the deque's own
        span: the sliding eviction always keeps the oldest sample just under
        stale_window_s old by construction, so comparing the span against the
        same threshold would never be true and the check would never fire.
        """
        if self._stale_window_started_at is None:
            return None
        if now - self._stale_window_started_at < self._stale_window_s:
            return None
        if not self._stale_samples:
            return None
        stale = sum(1 for _, s in self._stale_samples if s)
        return stale / len(self._stale_samples)

    def _tick(self):
        now = self._now()
        dt = now - self._last_tick
        self._last_tick = now

        if self._inhibited:
            return
        if self._submerge.state in (SubmergeState.IDLE, SubmergeState.FAILED):
            return

        try:
            yaw = self._fresh_yaw(now)
            depth = self._fresh_depth(now)

            heave, sub_state = self._submerge.update(depth, yaw, now)

            if sub_state is SubmergeState.FAILED:
                self.get_logger().error(
                    f'submerge FAILED — {self._submerge.failure_reason}')
                self._publish_state()
                self._publish_stop()
                return

            if sub_state is not SubmergeState.HOLD:
                # Still sequencing. The operator has no authority yet: a forward
                # command mid-dive would carry us away from the dive point.
                axes = self._mixer.mix(0.0, 0.0, yaw_correction=0.0, heave=heave)
                self._publish_axes(axes)
                self._publish_state()
                self._publish_debug(yaw, depth, axes)
                return

            # ── HOLD: ALT_HOLD owns depth/roll/pitch; we own yaw ──
            if depth is None:
                self.get_logger().error(
                    'depth lost — stopping movement. Staying in ALT_HOLD; the '
                    'autopilot holds depth if it still can.')
                self._publish_stop()
                self._publish_state()
                return

            if self._mode != 'ALT_HOLD':
                self.get_logger().error(
                    f'vehicle is in {self._mode}, not ALT_HOLD — depth hold is '
                    'NOT active. Stopping movement.')
                self._publish_stop()
                self._publish_state()
                return

            self._record_stale_sample(now, yaw is None)
            duty = self._stale_duty_fraction(now)
            if duty is not None and duty > self._stale_duty_abort:
                self.get_logger().error(
                    f'yaw stale {duty:.0%} of last {self._stale_window_s:.1f}s '
                    f'(> {self._stale_duty_abort:.0%} duty-cycle threshold) — '
                    'STOP + unlock')
                self._duty_aborted = True
                self._abort('yaw source degraded (duty-cycle abort)')
                return

            if self._op_yaw is not None and yaw is not None:
                # Operator is deliberately turning. The mixer routes yaw from
                # operator_yaw while they steer; keep the lock CAPTURED on the
                # current heading every tick so that when they release, it holds
                # the NEW heading. Without this the target stays where it was
                # captured at dive time, and release drives the nose back to the
                # pre-turn heading — the overshoot-then-oscillate symptom.
                self._heading.start(yaw, base_speed=self._op_surge)
                yaw_correction, lock_state = 0.0, LockState.LOCKED
            else:
                self._heading.set_base_speed(self._op_surge)
                _, yaw_correction, lock_state = self._heading.update(yaw, now, dt)

            if lock_state is LockState.ABORTED:
                self.get_logger().error(
                    f'yaw stale > {self._heading.grace_s:.1f}s grace — STOP')
                self._abort('yaw source lost')
                return
            if lock_state is LockState.STALE_GRACE and not self._warned_stale:
                self.get_logger().warn(
                    'yaw stale — correction zeroed, forward continues '
                    f'{self._heading.grace_s:.1f}s grace')
                self._warned_stale = True
            elif lock_state is LockState.LOCKED:
                self._warned_stale = False

            axes = self._mixer.mix(
                operator_surge=self._op_surge,
                operator_strafe=self._op_strafe,
                yaw_correction=yaw_correction,
                heave=0.0,                        # ALT_HOLD owns depth now
                operator_yaw=self._op_yaw)
            self._publish_axes(axes)
            self._publish_state()
            self._publish_debug(yaw, depth, axes)

        except Exception as exc:
            self.get_logger().error(f'tick error: {exc} — stop + unlock')
            self._abort(f'tick error: {exc}')

    # ─── outputs ────────────────────────────────────────────────────

    def _abort(self, reason):
        self._submerge.abort(reason)
        self._op_surge = 0.0
        self._op_strafe = 0.0
        self._op_yaw = None
        self._publish_state()
        self._publish_stop()

    def _publish_stop(self):
        msg = MovementCommand()
        msg.command = 'stop'
        self._cmd_pub.publish(msg)

    def _publish_axes(self, axes):
        msg = MovementCommand()
        msg.command = 'axes'
        msg.surge = float(axes.surge)
        msg.strafe = float(axes.strafe)
        msg.heave = float(axes.heave)
        msg.yaw_rate = float(axes.yaw_rate)
        # roll_rate / pitch_rate stay 0: ALT_HOLD self-levels, and commanding
        # them would fight the autopilot's attitude controller.
        self._cmd_pub.publish(msg)

    def _publish_state(self):
        state = self._submerge.state.value
        if self._submerge.state is SubmergeState.FAILED:
            state = f'failed: {self._submerge.failure_reason}'
        self._state_pub.publish(String(data=state))

    def _publish_debug(self, yaw, depth, axes):
        nan = float('nan')
        values = {
            'heading/current': yaw if yaw is not None else nan,
            'heading/target': self._heading.target_yaw,
            'heading/error': (self._heading.last_error if yaw is not None
                              else nan),
            'heading/yaw_correction': axes.yaw_rate,
            'depth/current': depth if depth is not None else nan,
            'depth/target': self._submerge.target_depth,
            'motion/forward_cmd': axes.surge,
            'motion/vertical_cmd': axes.heave,
        }
        for topic, v in values.items():
            self._dbg[topic].publish(Float32(data=float(v)))

    # ─── live tuning ────────────────────────────────────────────────

    def _on_params(self, params):
        """Validate the WHOLE incoming batch first; on the first violation reject
        the batch and apply NOTHING. Partial application would leave e.g. a
        validated kp alongside a rejected, unchanged max_yaw_correction —
        silently inconsistent."""
        for prm in params:
            reason = _validate_param(prm.name, prm.value)
            if reason is not None:
                return SetParametersResult(successful=False, reason=reason)

        for prm in params:
            name = prm.name
            v = _pf(prm.value, PARAM_DEFAULTS.get(name, 0.0))
            if name == 'heading_kp':
                self._pid.set_gains(kp=v)
            elif name == 'heading_ki':
                self._pid.set_gains(ki=v)
            elif name == 'heading_kd':
                self._pid.set_gains(kd=v)
            elif name == 'i_limit':
                self._pid.set_gains(i_limit=v)
            elif name == 'max_yaw_correction':
                self._heading.max_yaw_authority = v
            elif name == 'grace_s':
                self._heading.grace_s = v
            elif name == 'stale_timeout_s':
                self._stale_timeout_s = v
            elif name == 'depth_stale_timeout_s':
                self._depth_stale_timeout_s = v
            elif name == 'max_forward_speed':
                self._max_forward_speed = v
                self._mixer.max_surge = v
            elif name == 'stale_window_s':
                self._stale_window_s = v
            elif name == 'stale_duty_abort':
                self._stale_duty_abort = v
            elif name == 'dive_speed':
                self._default_dive_speed = v
            elif name == 'depth_tolerance_m':
                self._depth_ctl.tolerance_m = v
            elif name == 'min_heave':
                self._depth_ctl.min_heave = v
            elif name == 'depth_timeout':
                self._depth_ctl.timeout_s = v
            elif name == 'phase_timeout_s':
                self._submerge.phase_timeout_s = v
            # control_rate_hz / yaw_topic are rejected above, never reached here.
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
    node = MotionNode()
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
