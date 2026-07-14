"""MotionController: which axis is allowed to come from where. Pure."""
import pytest

from control.motion import Axes, MotionController


def test_operator_owns_surge_and_strafe():
    mc = MotionController()
    axes = mc.mix(operator_surge=0.4, operator_strafe=-0.2,
                  yaw_correction=0.0, heave=0.0)
    assert axes.surge == pytest.approx(0.4)
    assert axes.strafe == pytest.approx(-0.2)


def test_yaw_comes_from_the_heading_lock_not_the_operator():
    # The point of the whole feature: the pilot does not steer.
    mc = MotionController()
    axes = mc.mix(operator_surge=0.4, operator_strafe=0.0,
                  yaw_correction=0.15, heave=0.0)
    assert axes.yaw_rate == pytest.approx(0.15)


def test_explicit_operator_yaw_overrides_the_lock():
    # An intentional heading change must still be possible. When the operator
    # asks for yaw their value wins and the correction is discarded — summing
    # the two would make the lock fight the turn it was just told to make.
    mc = MotionController()
    axes = mc.mix(operator_surge=0.0, operator_strafe=0.0,
                  yaw_correction=0.3, operator_yaw=-0.5, heave=0.0)
    assert axes.yaw_rate == pytest.approx(-0.5)


def test_operator_yaw_of_zero_is_still_an_override_not_a_default():
    # 0.0 is a real command ("hold this rate"), distinct from None ("I'm not
    # steering, use the lock"). Conflating them would silently disable the
    # heading lock for anyone who passed 0.
    mc = MotionController()
    axes = mc.mix(operator_surge=0.0, operator_strafe=0.0,
                  yaw_correction=0.3, operator_yaw=0.0, heave=0.0)
    assert axes.yaw_rate == pytest.approx(0.0)


def test_roll_and_pitch_are_never_commanded():
    # ALT_HOLD self-levels. Commanding roll/pitch would fight its attitude
    # controller. Axes has no roll/pitch field at all — the message defaults to
    # 0 and nothing can set it.
    assert Axes._fields == ('surge', 'strafe', 'heave', 'yaw_rate')


def test_heave_passes_through_during_the_dive():
    mc = MotionController()
    axes = mc.mix(operator_surge=0.0, operator_strafe=0.0,
                  yaw_correction=0.0, heave=0.3)
    assert axes.heave == pytest.approx(0.3)


def test_every_axis_is_clamped_to_the_movement_command_range():
    mc = MotionController()
    axes = mc.mix(operator_surge=5.0, operator_strafe=-9.0,
                  yaw_correction=3.0, heave=-4.0)
    assert axes.surge == 1.0
    assert axes.strafe == -1.0
    assert axes.yaw_rate == 1.0
    assert axes.heave == -1.0


def test_max_surge_limits_the_operator_but_not_the_heading_lock():
    mc = MotionController(max_surge=0.5)
    axes = mc.mix(operator_surge=0.9, operator_strafe=0.0,
                  yaw_correction=0.8, heave=0.0)
    assert axes.surge == pytest.approx(0.5)
    assert axes.yaw_rate == pytest.approx(0.8)


def test_non_finite_input_becomes_zero_rather_than_propagating():
    # A NaN reaching MANUAL_CONTROL is undefined behaviour at the autopilot.
    mc = MotionController()
    axes = mc.mix(operator_surge=float('nan'), operator_strafe=0.2,
                  yaw_correction=float('inf'), heave=0.0)
    assert axes.surge == 0.0
    assert axes.yaw_rate == 0.0
    assert axes.strafe == pytest.approx(0.2)


def test_stop_axes_are_all_zero():
    assert MotionController.STOP == Axes(0.0, 0.0, 0.0, 0.0)
