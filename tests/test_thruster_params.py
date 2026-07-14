"""Preflight param comparison. Pure — no MAVLink, no vehicle."""
from mavlink_thruster_control.thruster_params import (
    ALL_MOTORS, compare, expected_params)


def test_all_eight_motors_are_checked():
    assert ALL_MOTORS == (1, 2, 3, 4, 5, 6, 7, 8)


def test_expected_params_cover_direction_function_and_reversed():
    exp = expected_params([1])
    assert exp['MOT_1_DIRECTION'] == -1.0
    assert exp['SERVO1_FUNCTION'] == 33.0
    assert exp['SERVO1_REVERSED'] == 0.0
    assert exp['SERVO1_TRIM'] == 1500.0
    assert exp['SERVO1_MIN'] == 1100.0
    assert exp['SERVO1_MAX'] == 1900.0


def test_matching_params_pass():
    exp = expected_params(ALL_MOTORS)
    ok, problems = compare(dict(exp), exp)
    assert ok
    assert problems == []


def test_flipped_horizontal_direction_is_caught():
    # A flipped horizontal turns a pure forward command into a yaw torque —
    # the sub spins instead of driving straight (2026-07-13 incident).
    exp = expected_params([1, 2, 3, 4])
    live = dict(exp)
    live['MOT_3_DIRECTION'] = -1.0        # backup says +1
    ok, problems = compare(live, exp)
    assert not ok
    assert any('MOT_3_DIRECTION' in p for p in problems)


def test_flipped_vertical_direction_is_caught():
    # A flipped vertical fights the other three on the heave axis: the sub
    # rolls and refuses to descend.
    exp = expected_params([5, 6, 7, 8])
    live = dict(exp)
    live['MOT_5_DIRECTION'] = 1.0         # backup says -1
    ok, problems = compare(live, exp)
    assert not ok
    assert any('MOT_5_DIRECTION' in p for p in problems)


def test_unreadable_param_fails_closed():
    # No response from the autopilot means we cannot CONFIRM the config. That
    # is a failure, not a pass — "unknown" must never read as "fine".
    exp = expected_params([1])
    live = dict(exp)
    live['SERVO1_REVERSED'] = None
    ok, problems = compare(live, exp)
    assert not ok
    assert any('NO RESPONSE' in p for p in problems)


def test_missing_param_key_fails_closed():
    exp = expected_params([1])
    live = dict(exp)
    del live['MOT_1_DIRECTION']
    ok, problems = compare(live, exp)
    assert not ok


def test_small_float_noise_is_tolerated():
    # PARAM_VALUE arrives as a float32; exact equality would false-alarm.
    exp = expected_params([1])
    live = dict(exp)
    live['SERVO1_TRIM'] = 1500.0001
    ok, _ = compare(live, exp)
    assert ok
