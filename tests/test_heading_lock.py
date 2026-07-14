"""HeadingLock control-law tests. The sign convention here is THE contract:
imu/rpy yaw is REP-103 CCW-positive, MovementCommand.yaw_rate is CW-positive,
error = wrap(current - target) feeds the PID directly."""
import math

import pytest

from control.heading_lock import HeadingLock, LockState, wrap
from control.pid import PID


def make_lock(kp=1.0, ki=0.0, kd=0.0, authority=0.4, grace_s=1.0):
    return HeadingLock(PID(kp=kp, ki=ki, kd=kd, limit=1.0, i_limit=0.3),
                       max_yaw_authority=authority, grace_s=grace_s)


def test_wrap():
    assert wrap(math.radians(340)) == pytest.approx(math.radians(-20))
    assert wrap(math.radians(-340)) == pytest.approx(math.radians(20))
    assert wrap(0.0) == 0.0


def test_start_captures_target_and_locks():
    lock = make_lock()
    assert lock.state is LockState.IDLE
    lock.start(0.7, base_speed=0.3)
    assert lock.state is LockState.LOCKED
    assert lock.target_yaw == pytest.approx(0.7)
    assert lock.base_speed == pytest.approx(0.3)


def test_idle_update_is_neutral():
    lock = make_lock()
    assert lock.update(0.5, now_s=0.0, dt_s=0.05) == (0.0, 0.0, LockState.IDLE)


def test_cw_drift_gets_ccw_correction():
    # REP-103: clockwise drift DECREASES yaw. Correction must be negative
    # (CCW on the CW-positive yaw_rate axis) -> mixer raises right pair.
    lock = make_lock(kp=1.0)
    lock.start(0.0, base_speed=0.3)
    surge, yaw_rate, state = lock.update(-0.1, now_s=0.0, dt_s=0.05)
    assert state is LockState.LOCKED
    assert surge == pytest.approx(0.3)          # forward speed untouched
    assert yaw_rate == pytest.approx(-0.1)      # kp * wrap(-0.1 - 0.0)
    assert lock.last_error == pytest.approx(-0.1)


def test_ccw_drift_gets_cw_correction():
    lock = make_lock(kp=1.0)
    lock.start(0.0, base_speed=0.3)
    _, yaw_rate, _ = lock.update(0.1, now_s=0.0, dt_s=0.05)
    assert yaw_rate == pytest.approx(0.1)


def test_error_wraps_across_pi():
    # target +170deg, current -170deg: shortest path is +20deg CCW of target
    # -> error +20deg -> CW (positive) correction, NOT -340deg.
    lock = make_lock(kp=1.0)
    lock.start(math.radians(170), base_speed=0.3)
    _, yaw_rate, _ = lock.update(math.radians(-170), now_s=0.0, dt_s=0.05)
    assert yaw_rate == pytest.approx(math.radians(20), abs=1e-9)


def test_correction_clamped_to_authority():
    lock = make_lock(kp=100.0, authority=0.4)
    lock.start(0.0, base_speed=0.3)
    _, yaw_rate, _ = lock.update(-0.5, now_s=0.0, dt_s=0.05)
    assert yaw_rate == -0.4
    _, yaw_rate, _ = lock.update(0.5, now_s=0.1, dt_s=0.05)
    assert yaw_rate == 0.4


def test_stale_grace_then_abort():
    lock = make_lock(grace_s=1.0)
    lock.start(0.0, base_speed=0.3)
    # stale: correction zeroed, forward continues
    surge, yaw_rate, state = lock.update(None, now_s=10.0, dt_s=0.05)
    assert (surge, yaw_rate, state) == (0.3, 0.0, LockState.STALE_GRACE)
    surge, yaw_rate, state = lock.update(None, now_s=10.9, dt_s=0.05)
    assert state is LockState.STALE_GRACE
    # past grace: abort, everything neutral
    surge, yaw_rate, state = lock.update(None, now_s=11.0, dt_s=0.05)
    assert (surge, yaw_rate, state) == (0.0, 0.0, LockState.ABORTED)
    # aborted latches until stop(): even a fresh sample stays aborted
    assert lock.update(0.0, now_s=11.1, dt_s=0.05) == (
        0.0, 0.0, LockState.ABORTED)


def test_recovery_within_grace_keeps_original_target():
    lock = make_lock(kp=1.0, grace_s=1.0)
    lock.start(0.5, base_speed=0.3)
    _, _, state = lock.update(None, now_s=0.0, dt_s=0.05)
    assert state is LockState.STALE_GRACE
    _, yaw_rate, state = lock.update(0.4, now_s=0.5, dt_s=0.05)
    assert state is LockState.LOCKED
    assert lock.target_yaw == pytest.approx(0.5)     # NOT re-locked at 0.4
    assert yaw_rate == pytest.approx(-0.1)


def test_second_stale_episode_gets_fresh_grace_budget():
    lock = make_lock(grace_s=1.0)
    lock.start(0.0, base_speed=0.3)
    lock.update(None, now_s=0.0, dt_s=0.05)            # grace starts
    lock.update(0.0, now_s=0.5, dt_s=0.05)             # recovers
    _, _, state = lock.update(None, now_s=5.0, dt_s=0.05)
    assert state is LockState.STALE_GRACE              # new episode, new clock
    _, _, state = lock.update(None, now_s=5.9, dt_s=0.05)
    assert state is LockState.STALE_GRACE


def test_set_base_speed_does_not_relock():
    lock = make_lock()
    lock.start(0.5, base_speed=0.3)
    lock.set_base_speed(0.6)
    surge, _, _ = lock.update(0.5, now_s=0.0, dt_s=0.05)
    assert surge == pytest.approx(0.6)
    assert lock.target_yaw == pytest.approx(0.5)


def test_stop_resets_integrator():
    lock = make_lock(kp=0.0, ki=1.0)
    lock.start(0.0, base_speed=0.3)
    for i in range(20):                                # wind the integral up
        lock.update(0.3, now_s=i * 0.05, dt_s=0.05)
    lock.stop()
    assert lock.state is LockState.IDLE
    lock.start(0.0, base_speed=0.3)
    _, yaw_rate, _ = lock.update(0.0, now_s=100.0, dt_s=0.05)
    assert yaw_rate == 0.0                             # no windup carryover


def test_start_rejects_nonfinite_yaw():
    lock = make_lock()
    with pytest.raises(ValueError):
        lock.start(float('nan'), base_speed=0.3)
