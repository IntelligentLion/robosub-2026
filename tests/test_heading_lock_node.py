"""heading_lock_node wiring tests: cmd handling, speed clamp, staleness.
Callbacks invoked directly; no spinning, no hardware."""
import pytest
import rclpy
from geometry_msgs.msg import Vector3Stamped
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
                           ('max_yaw_authority', 0.4),
                           ('max_forward_speed', 0.6),
                           ('stale_timeout_s', 0.5), ('grace_s', 1.0),
                           ('rate_hz', 20.0)]:
            assert node.get_parameter(name).value == want, name
        assert node.get_parameter('yaw_topic').value == 'imu/rpy'
    finally:
        node.destroy_node()
