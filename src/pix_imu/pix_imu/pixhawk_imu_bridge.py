#!/usr/bin/env python3
"""pixhawk_imu_bridge — MAVLink (ArduSub) IMU -> sensor_msgs/Imu.

Connects to the Pixhawk over serial (pymavlink), requests ATTITUDE and
RAW_IMU streams, and republishes them as a single sensor_msgs/Imu on
`imu_topic` (default /pixhawk/imu/data). The imu package's generic
orientation/diagnostics/marker nodes then consume that topic exactly like
they consume the ZED — no ZED, no per-node MAVLink code.

Body frame is ArduPilot FRD (x fwd, y right, z down). orientation_node
zeroes orientation relative to a startup reference, so the tree reads
level at launch and rotation still displays correctly regardless of frame
convention. Publishing raw FRD keeps accel/gyro arrows consistent with it.

A background thread owns the single serial reader (one reader per port —
two threads recv'ing one port causes the "readiness to read but returned
no data" stall). ATTITUDE drives the publish; the latest RAW_IMU accel and
ATTITUDE body rates ride along in each message.
"""
import math
import sys
import threading

# pymavlink 2.4.49 add_message bug: a MAVLink1 packet (no instance field)
# stores a message with _instances=None; a later MAVLink2 packet of the same
# type then indexes _instances[i] -> TypeError, killing the recv path. Guard
# must be installed BEFORE any connection is created. (Same fix as
# field_common.py; this standalone node needs its own copy.)
from pymavlink import mavutil as _mavutil

_orig_add_message = _mavutil.add_message


def _safe_add_message(messages, mtype, msg):
    stored = messages.get(mtype)
    if (stored is not None
            and getattr(stored, '_instances', None) is None
            and msg._instance_field is not None
            and getattr(msg, msg._instance_field, None) is not None):
        del messages[mtype]
    _orig_add_message(messages, mtype, msg)


_mavutil.add_message = _safe_add_message

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from pymavlink import mavutil

G = 9.80665                       # m/s^2 (RAW_IMU accel is in mg)
ATTITUDE_MSG_ID = 30
RAW_IMU_MSG_ID = 27


def quat_from_euler(roll, pitch, yaw):
    """(x,y,z,w) from roll/pitch/yaw radians — REP-103 XYZ (matches imu pkg)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PixhawkImuBridge(Node):
    def __init__(self):
        super().__init__('pixhawk_imu_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('imu_topic', '/pixhawk/imu/data')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('rate_hz', 50)

        self._port = self.get_parameter('port').value
        self._baud = int(self.get_parameter('baud').value)
        self._topic = self.get_parameter('imu_topic').value
        self._frame = self.get_parameter('imu_frame').value
        self._rate = int(self.get_parameter('rate_hz').value)

        self._pub = self.create_publisher(
            Imu, self._topic, qos_profile_sensor_data)

        # latest RAW_IMU accel (m/s^2), filled by the reader thread.
        self._accel = (0.0, 0.0, 0.0)

        self.get_logger().info(
            f'connecting {self._port} @ {self._baud} …')
        self._master = mavutil.mavlink_connection(self._port, baud=self._baud)
        self._master.wait_heartbeat(timeout=10)
        self.get_logger().info(
            f'heartbeat OK (sysid={self._master.target_system} '
            f'compid={self._master.target_component})')

        self._request_stream(ATTITUDE_MSG_ID, self._rate)
        self._request_stream(RAW_IMU_MSG_ID, self._rate)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'pixhawk_imu_bridge up — publishing {self._topic} '
            f'from ATTITUDE+RAW_IMU')

    def _request_stream(self, msg_id, hz):
        interval_us = int(1e6 / hz)
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0)

    def _reader(self):
        """Single serial reader: ATTITUDE publishes, RAW_IMU caches accel."""
        while not self._stop.is_set():
            try:
                msg = self._master.recv_match(
                    type=['ATTITUDE', 'RAW_IMU'], blocking=True, timeout=1.0)
            except Exception as exc:                 # serial hiccup — retry
                self.get_logger().warn(f'recv error: {exc}')
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'RAW_IMU':
                # accel reported in mg -> m/s^2
                self._accel = (
                    msg.xacc / 1000.0 * G,
                    msg.yacc / 1000.0 * G,
                    msg.zacc / 1000.0 * G)
            elif t == 'ATTITUDE':
                self._publish_attitude(msg)

    def _publish_attitude(self, att):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self._frame

        qx, qy, qz, qw = quat_from_euler(att.roll, att.pitch, att.yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        # ATTITUDE body rates (rad/s), FRD.
        imu.angular_velocity.x = att.rollspeed
        imu.angular_velocity.y = att.pitchspeed
        imu.angular_velocity.z = att.yawspeed

        ax, ay, az = self._accel
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        self._pub.publish(imu)

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
