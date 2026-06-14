#!/usr/bin/env python3
"""Localization node — fuses VIO odometry, drift corrections, and depth.

Subscribes:
  - odom/bottom              (nav_msgs/Odometry)          – ZED VIO from bottom camera
  - localization/correction  (geometry_msgs/PoseStamped)  – drift corrections from markers
  - depth/info               (auv_msgs/DepthInfo)         – depth sensor data

Publishes:
  - localization/pose        (geometry_msgs/PoseStamped)   – fused sub pose at 10 Hz
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from auv_msgs.msg import DepthInfo


def _is_finite(v):
    return math.isfinite(v)


MAX_POSITION_M = 50.0
MAX_OFFSET_M = 10.0


def _quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _yaw_to_quat(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('correction_alpha', 0.3)

        rate = self.get_parameter('publish_rate').value
        self._alpha = self.get_parameter('correction_alpha').value

        # Fused state
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0
        self._quat = _yaw_to_quat(0.0)

        # Correction offset applied to raw VIO
        self._offset_x = 0.0
        self._offset_y = 0.0
        self._offset_z = 0.0
        self._offset_yaw = 0.0

        # Latest raw VIO values
        self._vio_x = 0.0
        self._vio_y = 0.0
        self._vio_z = 0.0
        self._vio_yaw = 0.0
        self._vio_quat = _yaw_to_quat(0.0)
        self._vio_received = False

        # Depth from depth sensor
        self._depth_m = -1.0

        self.create_subscription(
            Odometry, 'odom/bottom', self._odom_cb, 10)
        # Accept VSLAM odometry as an alternative VIO source
        self.create_subscription(
            Odometry, 'vslam/odometry', self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, 'localization/correction', self._correction_cb, 10)
        self.create_subscription(
            DepthInfo, 'depth/info', self._depth_cb, 10)

        self._pose_pub = self.create_publisher(
            PoseStamped, 'localization/pose', 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Localization node started — rate={rate} Hz, '
            f'correction_alpha={self._alpha}')

    def _odom_cb(self, msg: Odometry):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            if not all(_is_finite(v) for v in (p.x, p.y, p.z, q.w, q.x, q.y, q.z)):
                self.get_logger().warn('Non-finite VIO data — ignoring frame')
                return

            self._vio_x = max(-MAX_POSITION_M, min(MAX_POSITION_M, p.x))
            self._vio_y = max(-MAX_POSITION_M, min(MAX_POSITION_M, p.y))
            self._vio_z = max(-MAX_POSITION_M, min(MAX_POSITION_M, p.z))
            self._vio_yaw = _quat_to_yaw(q)
            self._vio_quat = q
            self._vio_received = True

            self._x = self._vio_x + self._offset_x
            self._y = self._vio_y + self._offset_y
            self._z = self._vio_z + self._offset_z
            self._yaw = self._vio_yaw + self._offset_yaw
        except Exception as e:
            self.get_logger().error(f'Odom callback error: {e}')

    def _correction_cb(self, msg: PoseStamped):
        try:
            corrected_x = msg.pose.position.x
            corrected_y = msg.pose.position.y
            corrected_z = msg.pose.position.z

            if not all(_is_finite(v) for v in (corrected_x, corrected_y, corrected_z)):
                self.get_logger().warn('Non-finite correction — ignoring')
                return

            corrected_yaw = _quat_to_yaw(msg.pose.orientation)

            target_offset_x = corrected_x - self._vio_x
            target_offset_y = corrected_y - self._vio_y
            target_offset_z = corrected_z - self._vio_z
            target_offset_yaw = corrected_yaw - self._vio_yaw

            if any(abs(v) > MAX_OFFSET_M for v in (target_offset_x, target_offset_y, target_offset_z)):
                self.get_logger().warn(
                    f'Correction offset too large — rejecting '
                    f'(dx={target_offset_x:.2f} dy={target_offset_y:.2f} dz={target_offset_z:.2f})')
                return

            a = self._alpha
            self._offset_x += a * (target_offset_x - self._offset_x)
            self._offset_y += a * (target_offset_y - self._offset_y)
            self._offset_z += a * (target_offset_z - self._offset_z)
            self._offset_yaw += a * (target_offset_yaw - self._offset_yaw)

            self.get_logger().info(
                f'Correction applied — offset: '
                f'dx={self._offset_x:.3f} dy={self._offset_y:.3f} '
                f'dz={self._offset_z:.3f} dyaw={math.degrees(self._offset_yaw):.1f}°')
        except Exception as e:
            self.get_logger().error(f'Correction callback error: {e}')

    def _depth_cb(self, msg: DepthInfo):
        try:
            if _is_finite(msg.sub_depth_m):
                self._depth_m = msg.sub_depth_m
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def _publish(self):
        if not self._vio_received:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.position.x = self._x
        msg.pose.position.y = self._y

        # Prefer depth sensor over VIO y-axis when available
        if self._depth_m > 0:
            msg.pose.position.z = -self._depth_m
        else:
            msg.pose.position.z = self._z

        msg.pose.orientation = _yaw_to_quat(self._yaw)

        self._pose_pub.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = LocalizationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f'Unhandled exception: {e}')
        else:
            print(f'[localization_node] Fatal startup error: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
