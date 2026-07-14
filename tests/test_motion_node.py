"""motion_node wiring: safety aborts, axis authority, sole-publisher guard.
Callbacks invoked directly; no spinning, no hardware."""
import pytest
import rclpy
from auv_msgs.msg import MovementCommand
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32, String

from control.motion_node import MotionNode, _validate_param
from control.submerge import SubmergeState


def setup_module(_m):
    rclpy.init()


def teardown_module(_m):
    rclpy.shutdown()


class FakePublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class FakeClock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


def make_node(clock=None):
    node = MotionNode()
    node._cmd_pub = FakePublisher()
    node._state_pub = FakePublisher()
    node._now = clock or FakeClock()
    node._last_tick = node._now()
    node._armed = True
    node._count_movement_publishers = lambda: 1     # we are alone

    # Stub the gateway services. The verdict must land in response to the
    # REQUEST, not be pre-set: _on_submerge clears stale results on every new
    # dive, exactly as it should.
    node.fake_preflight = (True, '')
    node.fake_mode = (True, '')

    def preflight():
        node._preflight_result = node.fake_preflight

    def set_mode(name):
        node._mode_requests.append(name)
        node._mode_result = node.fake_mode

    node._request_preflight = preflight
    node._request_mode = set_mode
    return node


def feed_yaw(node, yaw, t):
    node._last_yaw = (t, yaw)


def feed_depth(node, depth, t):
    node._last_depth = (t, float(depth))


def feed_mode(node, mode):
    node._on_mode(String(data=mode))


def last_cmd(node):
    assert node._cmd_pub.msgs, 'nothing published'
    return node._cmd_pub.msgs[-1]


def operator(node, surge=0.0, strafe=0.0, yaw_rate=0.0):
    cmd = MovementCommand()
    cmd.command = 'axes'
    cmd.surge = surge
    cmd.strafe = strafe
    cmd.yaw_rate = yaw_rate
    node._on_cmd(cmd)


def dive_to_hold(node, clock, target=2.0):
    node._on_submerge(Float32(data=target))
    feed_mode(node, 'ALT_HOLD')
    for depth in (0.0, 0.5, 1.0, 1.5, 2.0, 2.0, 2.0):
        t = clock.advance(0.05)
        feed_yaw(node, 0.7, t)
        feed_depth(node, depth, t)
        node._tick()
        if node._submerge.state is SubmergeState.HOLD:
            return
    raise AssertionError(f'never reached HOLD (stuck in {node._submerge.state})')


# ── axis authority ────────────────────────────────────────────────────

def test_forward_is_the_only_axis_the_operator_supplies():
    clock = FakeClock()
    node = make_node(clock)
    dive_to_hold(node, clock)
    operator(node, surge=0.4)

    t = clock.advance(0.05)
    feed_yaw(node, 0.9, t)          # drifted 0.2 rad off the 0.7 rad target
    feed_depth(node, 2.0, t)
    node._tick()

    out = last_cmd(node)
    assert out.command == 'axes'
    assert out.surge == pytest.approx(0.4)
    assert out.heave == 0.0         # ALT_HOLD owns depth now
    assert out.roll_rate == 0.0     # ALT_HOLD self-levels
    assert out.pitch_rate == 0.0
    assert out.yaw_rate != 0.0      # the lock is steering, not the operator
    node.destroy_node()


def test_heave_stays_zero_in_hold_so_alt_hold_keeps_owning_depth():
    clock = FakeClock()
    node = make_node(clock)
    dive_to_hold(node, clock)
    for _ in range(5):
        t = clock.advance(0.05)
        feed_yaw(node, 0.7, t)
        feed_depth(node, 2.4, t)    # well below target — do NOT correct it
        node._tick()
    axes = [m for m in node._cmd_pub.msgs if m.command == 'axes']
    assert axes[-1].heave == 0.0
    node.destroy_node()


# ── safety: heading loss ──────────────────────────────────────────────

def test_lost_heading_zeroes_the_correction_immediately():
    clock = FakeClock()
    node = make_node(clock)
    dive_to_hold(node, clock)
    operator(node, surge=0.4)

    clock.advance(node._stale_timeout_s + 0.1)     # yaw goes stale
    feed_depth(node, 2.0, clock.t)
    node._tick()

    out = last_cmd(node)
    assert out.yaw_rate == 0.0                     # never steer blind
    assert out.surge == pytest.approx(0.4)         # forward rides out the grace
    node.destroy_node()


def test_lost_heading_past_grace_stops_but_stays_in_alt_hold():
    clock = FakeClock()
    node = make_node(clock)
    dive_to_hold(node, clock)
    operator(node, surge=0.4)

    for _ in range(40):
        clock.advance(0.1)
        feed_depth(node, 2.0, clock.t)             # depth fine; only yaw gone
        node._tick()

    assert last_cmd(node).command == 'stop'
    # No further mode request: depth hold must survive a heading failure.
    assert node._mode_requests == ['ALT_HOLD']
    node.destroy_node()


# ── safety: depth loss ────────────────────────────────────────────────

def test_lost_depth_stops_movement_but_does_not_leave_alt_hold():
    clock = FakeClock()
    node = make_node(clock)
    dive_to_hold(node, clock)
    operator(node, surge=0.4)

    clock.advance(node._depth_stale_timeout_s + 0.1)
    feed_yaw(node, 0.7, clock.t)
    node._tick()

    assert last_cmd(node).command == 'stop'
    assert node._mode_requests == ['ALT_HOLD']     # no MANUAL, no second set
    node.destroy_node()


# ── safety: mode loss ─────────────────────────────────────────────────

def test_dropping_out_of_alt_hold_stops_movement():
    # ArduSub forced us back to MANUAL — the classic "mode 19" spam when the
    # Bar02 drops off I2C. Depth hold is gone; do not keep driving.
    clock = FakeClock()
    node = make_node(clock)
    dive_to_hold(node, clock)
    operator(node, surge=0.4)

    feed_mode(node, 'MANUAL')
    t = clock.advance(0.05)
    feed_yaw(node, 0.7, t)
    feed_depth(node, 2.0, t)
    node._tick()

    assert last_cmd(node).command == 'stop'
    node.destroy_node()


# ── the dive itself ───────────────────────────────────────────────────

def test_no_forward_thrust_during_the_dive():
    clock = FakeClock()
    node = make_node(clock)
    node._on_submerge(Float32(data=2.0))
    feed_mode(node, 'ALT_HOLD')
    operator(node, surge=0.5)              # operator jumps the gun

    for depth in (0.0, 0.3, 0.6):
        t = clock.advance(0.05)
        feed_yaw(node, 0.7, t)
        feed_depth(node, depth, t)
        node._tick()

    axes = [m for m in node._cmd_pub.msgs if m.command == 'axes']
    assert axes, 'expected axes output during the dive'
    assert all(m.surge == 0.0 for m in axes)
    assert any(m.heave > 0.0 for m in axes)
    node.destroy_node()


def test_failed_submerge_publishes_stop_and_the_reason():
    clock = FakeClock()
    node = make_node(clock)
    node.fake_preflight = (False, 'MOT_3_DIRECTION = -1 but backup says +1')
    node._on_submerge(Float32(data=2.0))
    for _ in range(5):
        t = clock.advance(0.05)
        feed_yaw(node, 0.7, t)
        feed_depth(node, 0.0, t)
        node._tick()
    assert node._submerge.state is SubmergeState.FAILED
    assert last_cmd(node).command == 'stop'
    assert any('MOT_3_DIRECTION' in m.data for m in node._state_pub.msgs)
    node.destroy_node()


def test_dive_never_starts_without_depth_data():
    # No depth = no dive. Descending blind with no way to know when to stop is
    # the failure this whole design exists to prevent.
    clock = FakeClock()
    node = make_node(clock)
    node._on_submerge(Float32(data=2.0))
    feed_mode(node, 'ALT_HOLD')
    for _ in range(6):
        t = clock.advance(0.05)
        feed_yaw(node, 0.7, t)
        node._tick()                       # depth never fed
    axes = [m for m in node._cmd_pub.msgs if m.command == 'axes']
    assert all(m.heave == 0.0 for m in axes)
    node.destroy_node()


# ── sole-publisher guard ──────────────────────────────────────────────

def test_refuses_to_command_when_another_movement_publisher_exists():
    # Two publishers on movement_command means two things fighting over the
    # thrusters. That is the exact overlap this node exists to eliminate.
    clock = FakeClock()
    node = make_node(clock)
    node._count_movement_publishers = lambda: 2
    node._check_sole_publisher()

    assert node._inhibited
    assert last_cmd(node).command == 'stop'

    node._on_submerge(Float32(data=2.0))           # refused while inhibited
    assert node._submerge.state is not SubmergeState.PREFLIGHT
    node.destroy_node()


def test_sole_publisher_check_ignores_our_own_publisher():
    node = make_node()
    node._count_movement_publishers = lambda: 1
    node._check_sole_publisher()
    assert not node._inhibited
    node.destroy_node()


# ── param validation (ported from heading_lock_node) ──────────────────

def test_zero_max_yaw_correction_is_rejected():
    # clamp(c, -0, +0) inverts into max(hi, min(lo, c)) = constant full
    # authority: the sub would spin regardless of error. Must not be settable.
    assert _validate_param('max_yaw_correction', 0.0) is not None
    assert _validate_param('max_yaw_correction', 0.4) is None


def test_stale_duty_abort_of_one_is_rejected():
    # The test is `fraction > threshold` and fraction maxes at 1.0, so exactly
    # 1.0 makes the degraded-source abort unreachable even at 100% stale ticks.
    assert _validate_param('stale_duty_abort', 1.0) is not None
    assert _validate_param('stale_duty_abort', 0.5) is None


def test_restart_only_params_are_rejected_live():
    assert _validate_param('control_rate_hz', 30.0) is not None
    assert _validate_param('yaw_topic', 'imu/rpy') is not None
