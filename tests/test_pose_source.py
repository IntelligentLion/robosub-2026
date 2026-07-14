"""PoseSource: dead-reckoning honesty + the ZED relay. Pure."""
import math

import pytest

from control.pose_source import DeadReckonPose, ZedOdomPose


def test_no_fix_until_depth_arrives():
    dr = DeadReckonPose()
    assert dr.pose() is None
    dr.update(surge_cmd=0.5, strafe_cmd=0.0, yaw_rad=0.0, depth_m=None, dt=0.1)
    assert dr.pose() is None
    dr.update(surge_cmd=0.0, strafe_cmd=0.0, yaw_rad=0.0, depth_m=2.0, dt=0.1)
    assert dr.pose() is not None


def test_dead_reckoning_always_declares_itself_an_estimate():
    # The single most important property in this file. A drifting position
    # rendered as if it were measured is worse than no position at all.
    assert DeadReckonPose().is_estimate() is True
    assert ZedOdomPose().is_estimate() is False


def test_forward_at_zero_yaw_moves_along_x():
    dr = DeadReckonPose(surge_scale=1.0)
    dr.update(0.0, 0.0, yaw_rad=0.0, depth_m=1.0, dt=0.1)
    dr.update(0.5, 0.0, yaw_rad=0.0, depth_m=1.0, dt=1.0)
    x, y, z, yaw = dr.pose()
    assert x == pytest.approx(0.5)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(1.0)      # depth is MEASURED, not integrated


def test_forward_at_ninety_degrees_moves_along_y():
    dr = DeadReckonPose(surge_scale=1.0)
    dr.update(0.0, 0.0, yaw_rad=math.pi / 2, depth_m=1.0, dt=0.1)
    dr.update(0.5, 0.0, yaw_rad=math.pi / 2, depth_m=1.0, dt=1.0)
    x, y, _, _ = dr.pose()
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.5)


def test_depth_is_measured_not_integrated():
    dr = DeadReckonPose()
    dr.update(0.0, 0.0, yaw_rad=0.0, depth_m=2.5, dt=0.1)
    assert dr.pose()[2] == pytest.approx(2.5)
    dr.update(0.0, 0.0, yaw_rad=0.0, depth_m=1.0, dt=0.1)
    assert dr.pose()[2] == pytest.approx(1.0)


def test_a_huge_dt_does_not_teleport_the_estimate():
    # A stalled node resuming must not integrate one giant step and fling the
    # marker across the map.
    dr = DeadReckonPose(surge_scale=1.0)
    dr.update(0.0, 0.0, 0.0, depth_m=1.0, dt=0.1)
    dr.update(1.0, 0.0, 0.0, depth_m=1.0, dt=45.0)
    assert dr.pose()[0] == pytest.approx(0.0)


def test_stale_yaw_holds_the_last_heading_rather_than_jumping_to_zero():
    dr = DeadReckonPose(surge_scale=1.0)
    dr.update(0.0, 0.0, yaw_rad=math.pi / 2, depth_m=1.0, dt=0.1)
    dr.update(1.0, 0.0, yaw_rad=None, depth_m=1.0, dt=1.0)
    x, y, _, yaw = dr.pose()
    assert yaw == pytest.approx(math.pi / 2)
    assert y == pytest.approx(1.0)      # kept going the way it was pointing


def test_reset_returns_to_origin_and_clears_the_fix():
    dr = DeadReckonPose(surge_scale=1.0)
    dr.update(0.0, 0.0, 0.0, depth_m=1.0, dt=0.1)
    dr.update(1.0, 0.0, 0.0, depth_m=1.0, dt=1.0)
    dr.reset()
    assert dr.pose() is None


def test_zed_relay_reports_what_it_is_given():
    z = ZedOdomPose()
    assert z.pose() is None
    z.set_pose(1.0, 2.0, 3.0, 0.5)
    assert z.pose() == pytest.approx((1.0, 2.0, 3.0, 0.5))
