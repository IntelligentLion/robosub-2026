"""thruster_node gateway: MAVLink messages in, ROS telemetry out.

Fake MAVLink messages are fed straight to _on_mav_msg. simulate=True, so no
serial port is opened and no reader thread runs.
"""
import math

import pytest
import rclpy

from mavlink_thruster_control.thruster_node import ThrusterController


def setup_module(_m):
    rclpy.init()


def teardown_module(_m):
    rclpy.shutdown()


class FakePublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class FakeMsg:
    """Minimal stand-in for a pymavlink message object."""

    def __init__(self, mtype, **fields):
        self._type = mtype
        self.__dict__.update(fields)

    def get_type(self):
        return self._type


def make_node():
    node = ThrusterController(simulate=True)
    node._imu_pub = FakePublisher()
    node._depth_pub = FakePublisher()
    node._mode_pub = FakePublisher()
    node._armed_pub = FakePublisher()
    return node


def attitude(roll=0.0, pitch=0.0, yaw=0.0):
    return FakeMsg('ATTITUDE', roll=roll, pitch=pitch, yaw=yaw,
                   rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0)


def pressure2(hpa):
    return FakeMsg('SCALED_PRESSURE2', press_abs=hpa, temperature=1500)


def heartbeat(custom_mode=2, armed=True):
    base = 128 if armed else 0        # MAV_MODE_FLAG_SAFETY_ARMED
    return FakeMsg('HEARTBEAT', custom_mode=custom_mode, base_mode=base)


def test_attitude_publishes_imu_with_yaw_preserved():
    node = make_node()
    node._on_mav_msg(attitude(yaw=math.pi / 2))
    node._publish_telemetry()
    assert len(node._imu_pub.msgs) == 1
    q = node._imu_pub.msgs[0].orientation
    # yaw=pi/2 about z → quaternion (0, 0, sin(pi/4), cos(pi/4))
    assert q.z == pytest.approx(math.sin(math.pi / 4))
    assert q.w == pytest.approx(math.cos(math.pi / 4))
    node.destroy_node()


def test_depth_is_nan_until_surface_is_latched():
    node = make_node()
    node._on_mav_msg(pressure2(1013.25))
    node._publish_telemetry()
    assert math.isnan(node._depth_pub.msgs[0].data)
    node.destroy_node()


def test_depth_published_after_surface_latch():
    node = make_node()
    for _ in range(node.SURFACE_SAMPLES):
        node._on_mav_msg(pressure2(1013.25))
    node._on_mav_msg(pressure2(1111.32))       # ~1 m of fresh water
    node._publish_telemetry()
    assert node._depth_pub.msgs[-1].data == pytest.approx(1.0, abs=1e-2)
    node.destroy_node()


def test_hull_baro_alone_never_latches_a_surface():
    # SCALED_PRESSURE instance 0 is the FMU baro inside the hull. It must not
    # become a depth source: it would read a constant while the sub sinks.
    node = make_node()
    for _ in range(node.SURFACE_SAMPLES + 5):
        node._on_mav_msg(FakeMsg('SCALED_PRESSURE', press_abs=1013.25,
                                 temperature=1500))
    node._publish_telemetry()
    assert node._surface_hpa is None
    assert math.isnan(node._depth_pub.msgs[-1].data)
    node.destroy_node()


def test_insane_surface_latch_is_rejected():
    node = make_node()
    for _ in range(node.SURFACE_SAMPLES):
        node._on_mav_msg(pressure2(4000.0))    # implausible
    node._publish_telemetry()
    assert node._surface_hpa is None
    assert math.isnan(node._depth_pub.msgs[-1].data)
    node.destroy_node()


def test_heartbeat_publishes_live_mode_and_armed():
    node = make_node()
    node._on_mav_msg(heartbeat(custom_mode=2, armed=True))
    node._publish_telemetry()
    assert node._mode_pub.msgs[-1].data == 'ALT_HOLD'
    assert node._armed_pub.msgs[-1].data is True
    node.destroy_node()


def test_mode_topic_reports_actual_mode_not_requested_mode():
    # The whole point: the gateway asked for ALT_HOLD, but a dead Bar02 makes
    # ArduSub stay in MANUAL(19). The topic must say MANUAL.
    node = make_node()
    node.flight_mode_name = 'ALT_HOLD'
    node.flight_mode_id = 2
    node._on_mav_msg(heartbeat(custom_mode=19, armed=True))
    node._publish_telemetry()
    assert node._mode_pub.msgs[-1].data == 'MANUAL'
    node.destroy_node()


def test_unknown_custom_mode_is_reported_not_swallowed():
    node = make_node()
    node._on_mav_msg(heartbeat(custom_mode=77, armed=True))
    node._publish_telemetry()
    assert node._mode_pub.msgs[-1].data == 'UNKNOWN(77)'
    node.destroy_node()


def test_stale_heartbeat_reports_unknown_mode_and_disarmed():
    # No heartbeat = we do not know what mode we are in. Reporting the last
    # known mode would let motion_node keep driving on a dead link.
    node = make_node()
    node._on_mav_msg(heartbeat(custom_mode=2, armed=True))
    node._last_hb_time -= (node.HEARTBEAT_STALE_S + 1.0)
    node._publish_telemetry()
    assert node._mode_pub.msgs[-1].data == 'UNKNOWN'
    assert node._armed_pub.msgs[-1].data is False
    node.destroy_node()


def test_statustext_is_captured_for_the_set_mode_reason():
    node = make_node()
    node._on_mav_msg(FakeMsg('STATUSTEXT', severity=3,
                             text='Depth sensor is not connected.'))
    assert node._last_statustext == 'Depth sensor is not connected.'
    node.destroy_node()
