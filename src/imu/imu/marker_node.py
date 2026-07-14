#!/usr/bin/env python3
"""marker_node — RViz arrows for body axes and IMU motion vectors.

All arrows are drawn in `base_link` (the frame orientation_node rotates), so
they turn with the sub in RViz. Body axes are fixed unit arrows; gravity,
angular-velocity and linear-acceleration arrows are built from the live IMU
sample. Each arrow has a stable (ns,id) so RViz updates in place.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray


def _arrow(frame, stamp, ns, mid, vec, rgb, scale=1.0):
    """One ARROW marker from origin to `vec` (scaled), colored rgb."""
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.scale.x = 0.02  # shaft diameter
    m.scale.y = 0.04  # head diameter
    m.scale.z = 0.06  # head length
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    m.points = [Point(x=0.0, y=0.0, z=0.0),
                Point(x=vec[0] * scale, y=vec[1] * scale, z=vec[2] * scale)]
    return m


class MarkerNode(Node):
    def __init__(self):
        super().__init__('marker_node')
        self.declare_parameter('imu_topic', '/zed2i/zed_node/imu/data')
        self.declare_parameter('marker_frame', 'base_link')
        self._imu_topic = self.get_parameter('imu_topic').value
        self._frame = self.get_parameter('marker_frame').value
        self._last = None
        self._pub = self.create_publisher(MarkerArray, 'imu/markers', 10)
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self._publish)  # 20 Hz
        self.get_logger().info(
            f'marker_node up — reading {self._imu_topic}, frame {self._frame}')

    def _imu_cb(self, msg: Imu):
        self._last = msg

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # fixed body axes (unit length)
        arr.markers.append(_arrow(self._frame, stamp, 'axis', 0, (1, 0, 0), (1.0, 0.0, 0.0)))  # forward +X red
        arr.markers.append(_arrow(self._frame, stamp, 'axis', 1, (0, 1, 0), (0.0, 1.0, 0.0)))  # right   +Y green
        arr.markers.append(_arrow(self._frame, stamp, 'axis', 2, (0, 0, 1), (0.0, 0.0, 1.0)))  # up      +Z blue
        if self._last is not None:
            a = self._last.linear_acceleration
            g = self._last.angular_velocity
            # gravity = measured accel direction (yellow), scaled down from ~9.8
            arr.markers.append(_arrow(
                self._frame, stamp, 'gravity', 3, (a.x, a.y, a.z), (1.0, 1.0, 0.0), scale=0.1))
            # linear acceleration (magenta)
            arr.markers.append(_arrow(
                self._frame, stamp, 'accel', 4, (a.x, a.y, a.z), (1.0, 0.0, 1.0), scale=0.1))
            # angular velocity (cyan)
            arr.markers.append(_arrow(
                self._frame, stamp, 'gyro', 5, (g.x, g.y, g.z), (0.0, 1.0, 1.0), scale=1.0))
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
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
