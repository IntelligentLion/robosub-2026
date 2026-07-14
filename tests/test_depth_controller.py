"""DepthController: the dive only. Pure — no ROS, no MAVLink, no sleeping."""
import pytest

from control.depth_controller import DepthController, DiveState


def make(**kw):
    kw.setdefault('tolerance_m', 0.15)
    kw.setdefault('min_heave', 0.12)
    kw.setdefault('timeout_s', 30.0)
    return DepthController(**kw)


def test_idle_before_start():
    dc = make()
    heave, state = dc.update(0.0, 0.0)
    assert state is DiveState.IDLE
    assert heave == 0.0


def test_dives_downward_toward_the_target():
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heave, state = dc.update(0.2, 1.0)
    assert state is DiveState.DIVING
    assert heave == pytest.approx(0.3)      # +heave is DOWN


def test_at_depth_within_tolerance_and_stops_thrusting():
    dc = make(tolerance_m=0.15)
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heave, state = dc.update(1.90, 5.0)     # 0.10 m short — inside tolerance
    assert state is DiveState.AT_DEPTH
    assert heave == 0.0


def test_at_depth_latches_and_does_not_resume_diving():
    # Once ALT_HOLD owns depth, a bit of overshoot/bob must NOT restart the
    # dive — that would fight the autopilot's own depth controller.
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    dc.update(2.0, 5.0)
    heave, state = dc.update(1.5, 6.0)      # drifted well above target
    assert state is DiveState.AT_DEPTH
    assert heave == 0.0


def test_min_heave_floor_beats_the_alt_hold_throttle_deadzone():
    # ArduSub treats z within ±THR_DZ (100 of 1000) of neutral as NO COMMAND.
    # A dive_speed of 0.05 would therefore do nothing at all while looking like
    # it should. Floor the magnitude instead of silently no-op'ing.
    dc = make(min_heave=0.12)
    dc.start(target_depth_m=2.0, dive_speed=0.05, now_s=0.0)
    heave, state = dc.update(0.0, 1.0)
    assert state is DiveState.DIVING
    assert heave == pytest.approx(0.12)


def test_min_heave_does_not_inflate_an_adequate_dive_speed():
    dc = make(min_heave=0.12)
    dc.start(target_depth_m=2.0, dive_speed=0.4, now_s=0.0)
    heave, _ = dc.update(0.0, 1.0)
    assert heave == pytest.approx(0.4)


def test_negative_dive_speed_is_treated_as_a_descent_magnitude():
    # The brief wrote dive_speed=-300 (raw MAVLink, negative = down). Our
    # convention is +heave = down. Accept either sign; never ascend by accident.
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=-0.3, now_s=0.0)
    heave, _ = dc.update(0.0, 1.0)
    assert heave == pytest.approx(0.3)


def test_missing_depth_never_dives_blind():
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heave, state = dc.update(None, 1.0)
    assert state is DiveState.NO_DEPTH_DATA
    assert heave == 0.0


def test_nan_depth_is_treated_as_missing():
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heave, state = dc.update(float('nan'), 1.0)
    assert state is DiveState.NO_DEPTH_DATA
    assert heave == 0.0


def test_depth_recovers_after_a_dropout():
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    dc.update(None, 1.0)
    heave, state = dc.update(0.5, 2.0)
    assert state is DiveState.DIVING
    assert heave == pytest.approx(0.3)


def test_timeout_stops_the_dive():
    dc = make(timeout_s=10.0)
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    heave, state = dc.update(0.5, 10.5)
    assert state is DiveState.TIMEOUT
    assert heave == 0.0


def test_timeout_latches():
    dc = make(timeout_s=10.0)
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    dc.update(0.5, 10.5)
    heave, state = dc.update(1.99, 11.0)    # even reaching depth won't undo it
    assert state is DiveState.TIMEOUT
    assert heave == 0.0


def test_stale_depth_does_not_hold_the_dive_open_forever():
    # A dropout mid-dive still counts against the timeout — otherwise a dead
    # sensor could keep us diving indefinitely.
    dc = make(timeout_s=10.0)
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    dc.update(None, 5.0)
    _, state = dc.update(None, 10.5)
    assert state is DiveState.TIMEOUT


def test_stop_returns_to_idle():
    dc = make()
    dc.start(target_depth_m=2.0, dive_speed=0.3, now_s=0.0)
    dc.stop()
    heave, state = dc.update(0.0, 1.0)
    assert state is DiveState.IDLE
    assert heave == 0.0


def test_start_rejects_a_non_finite_target():
    dc = make()
    with pytest.raises(ValueError):
        dc.start(target_depth_m=float('nan'), dive_speed=0.3, now_s=0.0)
