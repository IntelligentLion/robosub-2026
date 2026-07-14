"""heading_lock_node wiring tests: cmd handling, speed clamp, staleness.
Callbacks invoked directly; no spinning, no hardware."""
import math

import pytest
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.parameter import Parameter
from std_msgs.msg import Float32

from control.heading_lock import LockState
from control.heading_lock_node import HeadingLockNode


def make_node():
    return HeadingLockNode()


def setup_module(_m):
    rclpy.init()


def teardown_module(_m):
    rclpy.shutdown()


def yaw_msg(yaw):
    m = Vector3Stamped()
    m.vector.z = yaw
    return m


class FakePublisher:
    """Stand-in for an rclpy Publisher: records every published message."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class FakeClock:
    """Controllable time.monotonic() replacement — no sleeping in tests."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


def stale_script(pattern, target=0.0):
    """Build a _fresh_yaw(now) replacement that consumes one bool per call:
    True -> stale (returns None), False -> fresh (returns `target`)."""
    it = iter(pattern)

    def _fresh(now):
        return None if next(it) else target
    return _fresh


def test_cmd_without_yaw_is_refused():
    node = make_node()
    try:
        node._on_cmd(Float32(data=0.4))
        assert node._lock.state is LockState.IDLE
    finally:
        node.destroy_node()


def test_cmd_locks_and_clamps_speed():
    node = make_node()
    try:
        node._on_yaw(yaw_msg(0.3))
        node._on_cmd(Float32(data=5.0))          # way over max_forward_speed
        assert node._lock.state is LockState.LOCKED
        assert node._lock.target_yaw == pytest.approx(0.3)
        assert node._lock.base_speed == 0.6      # max_forward_speed default
    finally:
        node.destroy_node()


def test_second_cmd_updates_speed_without_relock():
    node = make_node()
    try:
        node._on_yaw(yaw_msg(0.3))
        node._on_cmd(Float32(data=0.3))
        node._on_yaw(yaw_msg(0.9))               # sub has drifted
        node._on_cmd(Float32(data=0.5))          # speed change only
        assert node._lock.target_yaw == pytest.approx(0.3)   # NOT re-captured
        assert node._lock.base_speed == 0.5
    finally:
        node.destroy_node()


def test_zero_and_nan_cmd_unlock():
    node = make_node()
    try:
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.4))
        node._on_cmd(Float32(data=0.0))
        assert node._lock.state is LockState.IDLE
        node._on_cmd(Float32(data=0.4))
        node._on_cmd(Float32(data=float('nan')))
        assert node._lock.state is LockState.IDLE
    finally:
        node.destroy_node()


def test_declared_params_have_spec_defaults():
    node = make_node()
    try:
        for name, want in [('kp', 1.2), ('ki', 0.0), ('kd', 0.3),
                           ('i_limit', 0.3),
                           ('max_yaw_authority', 0.4),
                           ('max_forward_speed', 0.6),
                           ('stale_timeout_s', 0.5), ('grace_s', 1.0),
                           ('stale_window_s', 3.0),
                           ('stale_duty_abort', 0.5),
                           ('rate_hz', 20.0)]:
            assert node.get_parameter(name).value == want, name
        assert node.get_parameter('yaw_topic').value == 'imu/rpy'
    finally:
        node.destroy_node()


# ─── published-message contract (Important 2) ──────────────────────────

def test_refused_lock_publishes_stop():
    node = make_node()
    try:
        fake = FakePublisher()
        node._cmd_pub = fake
        node._on_cmd(Float32(data=0.4))     # no fresh yaw -> refused
        assert len(fake.msgs) == 1
        assert fake.msgs[0].command == 'stop'
    finally:
        node.destroy_node()


def test_locked_tick_publishes_axes_with_pid_correction():
    node = make_node()
    try:
        fake = FakePublisher()
        node._cmd_pub = fake
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        node._on_yaw(yaw_msg(0.1))          # introduce heading error
        fake.msgs.clear()
        node._tick()
        assert len(fake.msgs) == 1
        msg = fake.msgs[0]
        assert msg.command == 'axes'
        assert msg.heave == 0.0
        assert msg.surge == pytest.approx(0.5)
        # First tick after start(): PID is freshly reset, so kd contributes
        # nothing (prev_error == error) and ki=0 by default -> output is
        # exactly kp * error, deterministic regardless of wall-clock dt.
        expected = node._pid.kp * 0.1
        assert msg.yaw_rate == pytest.approx(expected)
        assert msg.yaw_rate != 0.0
    finally:
        node.destroy_node()


def test_stale_grace_tick_publishes_axes_zero_yaw_rate():
    node = make_node()
    try:
        fake = FakePublisher()
        node._cmd_pub = fake
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        arrival, yaw = node._last_yaw
        node._last_yaw = (arrival - 10.0, yaw)     # force staleness
        fake.msgs.clear()
        node._tick()
        assert len(fake.msgs) == 1
        msg = fake.msgs[0]
        assert msg.command == 'axes'
        assert msg.yaw_rate == 0.0
        assert msg.surge == pytest.approx(0.5)
        assert msg.heave == 0.0
        assert node._lock.state is LockState.STALE_GRACE
    finally:
        node.destroy_node()


def test_post_grace_tick_stops_then_next_tick_is_silent():
    node = make_node()
    try:
        fake = FakePublisher()
        node._cmd_pub = fake
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        arrival, yaw = node._last_yaw
        node._last_yaw = (arrival - 10.0, yaw)     # force staleness
        node._lock.grace_s = 0.0                   # grace already exhausted
        fake.msgs.clear()
        node._tick()
        assert len(fake.msgs) == 1
        assert fake.msgs[0].command == 'stop'
        assert node._lock.state is LockState.IDLE

        fake.msgs.clear()
        node._tick()                                # ABORTED is latched IDLE
        assert fake.msgs == []
    finally:
        node.destroy_node()


def test_tick_exception_stops_and_idles(monkeypatch):
    node = make_node()
    try:
        fake = FakePublisher()
        node._cmd_pub = fake
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))

        def _boom(*_a, **_k):
            raise RuntimeError('boom')

        monkeypatch.setattr(node._lock, 'update', _boom)
        fake.msgs.clear()
        node._tick()
        assert len(fake.msgs) == 1
        assert fake.msgs[0].command == 'stop'
        assert node._lock.state is LockState.IDLE
    finally:
        node.destroy_node()


# ─── on-set-parameters validation (Important 1) ────────────────────────

def test_on_params_rejects_negative_max_yaw_authority_and_mutates_nothing():
    node = make_node()
    try:
        before = node._lock.max_yaw_authority
        result = node._on_params([Parameter('max_yaw_authority', value=-0.1)])
        assert result.successful is False
        assert node._lock.max_yaw_authority == before
    finally:
        node.destroy_node()


def test_on_params_batch_is_all_or_nothing():
    """One valid + one invalid param in the same batch: NEITHER applies."""
    node = make_node()
    try:
        before_kp = node._pid.kp
        before_authority = node._lock.max_yaw_authority
        result = node._on_params([
            Parameter('kp', value=99.0),               # valid on its own
            Parameter('max_yaw_authority', value=-1.0),  # invalid
        ])
        assert result.successful is False
        assert node._pid.kp == before_kp
        assert node._lock.max_yaw_authority == before_authority
    finally:
        node.destroy_node()


def test_on_params_rejects_huge_stale_timeout_is_still_positive_but_zero_rejected():
    node = make_node()
    try:
        before = node._stale_timeout_s
        result = node._on_params([Parameter('stale_timeout_s', value=0.0)])
        assert result.successful is False
        assert node._stale_timeout_s == before
    finally:
        node.destroy_node()


def test_on_params_rejects_zero_grace_s():
    node = make_node()
    try:
        before = node._lock.grace_s
        result = node._on_params([Parameter('grace_s', value=0.0)])
        assert result.successful is False
        assert node._lock.grace_s == before
    finally:
        node.destroy_node()


def test_on_params_rejects_max_forward_speed_over_one():
    node = make_node()
    try:
        before = node._max_forward_speed
        result = node._on_params([Parameter('max_forward_speed', value=1.5)])
        assert result.successful is False
        assert node._max_forward_speed == before
    finally:
        node.destroy_node()


def test_on_params_rejects_negative_gains():
    node = make_node()
    try:
        for name in ('kp', 'ki', 'kd', 'i_limit'):
            result = node._on_params([Parameter(name, value=-0.5)])
            assert result.successful is False, name
    finally:
        node.destroy_node()


def test_on_params_rejects_rate_hz_and_yaw_topic_live_change():
    node = make_node()
    try:
        result = node._on_params([Parameter('rate_hz', value=5.0)])
        assert result.successful is False
        assert 'restart' in result.reason

        result2 = node._on_params([Parameter('yaw_topic', value='other_topic')])
        assert result2.successful is False
        assert 'restart' in result2.reason
    finally:
        node.destroy_node()


@pytest.mark.parametrize('name, limit, attr', [
    # A huge stale_timeout_s means _fresh_yaw() never returns None, so the
    # whole staleness/abort contract is silently disabled and the sub steers
    # on an arbitrarily old yaw sample. A huge grace_s means arbitrarily long
    # blind forward. A huge stale_window_s means the duty-cycle window never
    # fills and that abort never fires. All three need a finite ceiling.
    ('stale_timeout_s', 2.0, lambda n: n._stale_timeout_s),
    ('grace_s', 5.0, lambda n: n._lock.grace_s),
    ('stale_window_s', 30.0, lambda n: n._stale_window_s),
])
def test_on_params_enforces_timing_upper_bounds(name, limit, attr):
    node = make_node()
    try:
        # Accepted exactly AT the bound.
        result = node._on_params([Parameter(name, value=limit)])
        assert result.successful is True, f'{name} should accept {limit}'
        assert attr(node) == pytest.approx(limit)

        # Rejected just past it, and nothing is mutated.
        before = attr(node)
        over = limit + 0.1
        result = node._on_params([Parameter(name, value=over)])
        assert result.successful is False, f'{name} should reject {over}'
        assert str(limit) in result.reason, (
            f'reason should name the bound so a tuner sees why: {result.reason}')
        assert attr(node) == pytest.approx(before)
    finally:
        node.destroy_node()


def test_on_params_rejects_staleness_disabling_timeout():
    """The concrete harm: `ros2 param set ... stale_timeout_s 999` must not
    silently disable the staleness/abort contract."""
    node = make_node()
    try:
        result = node._on_params([Parameter('stale_timeout_s', value=999.0)])
        assert result.successful is False
        assert node._stale_timeout_s == 0.5      # default, untouched
    finally:
        node.destroy_node()


def test_on_params_rejects_bad_stale_duty_params():
    node = make_node()
    try:
        assert node._on_params(
            [Parameter('stale_window_s', value=0.0)]).successful is False
        assert node._on_params(
            [Parameter('stale_duty_abort', value=0.0)]).successful is False
        assert node._on_params(
            [Parameter('stale_duty_abort', value=1.5)]).successful is False
    finally:
        node.destroy_node()


def test_on_params_applies_valid_batch():
    node = make_node()
    try:
        result = node._on_params([
            Parameter('kp', value=2.0),
            Parameter('max_yaw_authority', value=0.6),
            Parameter('stale_window_s', value=5.0),
            Parameter('stale_duty_abort', value=0.8),
        ])
        assert result.successful is True
        assert node._pid.kp == 2.0
        assert node._lock.max_yaw_authority == 0.6
        assert node._stale_window_s == 5.0
        assert node._stale_duty_abort == 0.8
    finally:
        node.destroy_node()


# ─── stale duty-cycle abort (Important 3) ───────────────────────────────

def test_low_stale_duty_keeps_driving(monkeypatch):
    node = make_node()
    try:
        clock = FakeClock(1000.0)
        monkeypatch.setattr('control.heading_lock_node.time.monotonic', clock)
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        fake = FakePublisher()
        node._cmd_pub = fake

        # 20 Hz for 4s = 80 ticks; one stale tick per 20 (~5% duty).
        pattern = [False] * 80
        for i in range(0, 80, 20):
            pattern[i] = True
        node._fresh_yaw = stale_script(pattern, target=0.0)

        for _ in range(80):
            clock.advance(0.05)
            node._tick()

        assert node._lock.state in (LockState.LOCKED, LockState.STALE_GRACE)
        assert fake.msgs[-1].command == 'axes'
    finally:
        node.destroy_node()


def test_high_stale_duty_triggers_duty_cycle_abort(monkeypatch):
    node = make_node()
    try:
        clock = FakeClock(2000.0)
        monkeypatch.setattr('control.heading_lock_node.time.monotonic', clock)
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        fake = FakePublisher()
        node._cmd_pub = fake

        # Bursts of 7 stale + 3 fresh ticks (0.35s stale run, well under the
        # 1.0s grace_s) so the EXISTING grace path can't fire on its own —
        # only the ~70% duty-cycle fraction should trigger the abort.
        pattern = []
        for _ in range(12):
            pattern.extend([True] * 7 + [False] * 3)
        node._fresh_yaw = stale_script(pattern, target=0.0)

        aborted = False
        for _ in range(len(pattern)):
            clock.advance(0.05)
            node._tick()
            if node._lock.state is LockState.IDLE:
                aborted = True
                break

        assert aborted
        assert fake.msgs[-1].command == 'stop'
    finally:
        node.destroy_node()


def test_fully_dead_source_grace_abort_fires_before_duty_window_fills(monkeypatch):
    node = make_node()
    try:
        clock = FakeClock(3000.0)
        monkeypatch.setattr('control.heading_lock_node.time.monotonic', clock)
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        fake = FakePublisher()
        node._cmd_pub = fake
        node._fresh_yaw = lambda now: None      # yaw never arrives again

        aborted_at = None
        for i in range(80):                      # 4s at 20 Hz
            clock.advance(0.05)
            node._tick()
            if node._lock.state is LockState.IDLE:
                aborted_at = (i + 1) * 0.05
                break

        assert aborted_at is not None
        assert aborted_at < node._stale_window_s   # grace_s (1.0s) beat the 3.0s window
        assert fake.msgs[-1].command == 'stop'
    finally:
        node.destroy_node()


# ─── debug topic contract (Minor) ───────────────────────────────────────

def test_error_topic_publishes_nan_when_stale():
    node = make_node()
    try:
        node._dbg['error'] = FakePublisher()
        node._dbg['current_yaw'] = FakePublisher()
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.5))
        arrival, yaw = node._last_yaw
        node._last_yaw = (arrival - 10.0, yaw)     # force staleness
        node._tick()
        assert math.isnan(node._dbg['error'].msgs[-1].data)
        assert math.isnan(node._dbg['current_yaw'].msgs[-1].data)
    finally:
        node.destroy_node()


def test_heave_always_zero_on_axes_publish():
    node = make_node()
    try:
        fake = FakePublisher()
        node._cmd_pub = fake
        node._on_yaw(yaw_msg(0.2))
        node._on_cmd(Float32(data=0.4))
        for _ in range(3):
            node._tick()
        axes_msgs = [m for m in fake.msgs if m.command == 'axes']
        assert axes_msgs, 'expected at least one axes publish'
        assert all(m.heave == 0.0 for m in axes_msgs)
    finally:
        node.destroy_node()
