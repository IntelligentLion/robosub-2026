#!/usr/bin/env python3
"""diagnostics_node — prints IMU orientation/gyro/accel at 10 Hz.

Caches the latest ZED IMU message and renders the fixed text block on a
timer (decoupled from the IMU rate so output stays a steady 10 Hz).
Roll/pitch/yaw come from the absolute reported orientation via the shared
euler helper. Angles printed in degrees for human reading.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from imu.imu_math import euler_from_quat


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')
        self.declare_parameter('imu_topic', '/zed2i/zed_node/imu/data')
        self._imu_topic = self.get_parameter('imu_topic').value
        self._last = None
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_timer(0.1, self._print)  # 10 Hz
        self.get_logger().info(
            f'diagnostics_node up — reading {self._imu_topic}')

    def _imu_cb(self, msg: Imu):
        self._last = msg

    def _print(self):
        if self._last is None:
            self.get_logger().warn('waiting for IMU data...', throttle_duration_sec=5.0)
            return
        m = self._last
        q = (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
        roll, pitch, yaw = (math.degrees(a) for a in euler_from_quat(q))
        g = m.angular_velocity
        a = m.linear_acceleration
        block = (
            "\nOrientation\n"
            f"\nRoll:  {roll:8.2f} deg"
            f"\nPitch: {pitch:8.2f} deg"
            f"\nYaw:   {yaw:8.2f} deg"
            "\n\nQuaternion\n"
            f"\nx {q[0]:+.4f}"
            f"\ny {q[1]:+.4f}"
            f"\nz {q[2]:+.4f}"
            f"\nw {q[3]:+.4f}"
            "\n\nGyroscope (rad/s)\n"
            f"\nX {g.x:+.4f}"
            f"\nY {g.y:+.4f}"
            f"\nZ {g.z:+.4f}"
            "\n\nAccelerometer (m/s^2)\n"
            f"\nX {a.x:+.4f}"
            f"\nY {a.y:+.4f}"
            f"\nZ {a.z:+.4f}\n"
        )
        # print() keeps the block clean; logger would prefix every line.
        print(block, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
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
