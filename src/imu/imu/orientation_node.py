#!/usr/bin/env python3
"""orientation_node — owns the TF tree and zeroes IMU orientation.

Subscribes the ZED 2i IMU, captures a startup reference quaternion (first
`calib_samples` averaged), and broadcasts odom->base_link as the current
orientation RELATIVE to that reference. So the sub reads level/identity at
launch and there is no cross-run accumulation. map->odom and
base_link->imu_link are static. Call /imu/reset_orientation to re-zero live.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from imu.imu_math import (
    normalize, quat_relative, quat_average, euler_from_quat,
)

# math is stdlib; used for the mount-offset rpy->quat conversion.
import math


def quat_from_euler(roll, pitch, yaw):
    """(x,y,z,w) from roll/pitch/yaw radians — REP-103 XYZ."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OrientationNode(Node):
    def __init__(self):
        super().__init__('orientation_node')
        self.declare_parameter('imu_topic', '/zed2i/zed_node/imu/data')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('calib_samples', 100)
        self.declare_parameter('mount_rpy', [0.0, 0.0, 0.0])
        # Set false when another node owns the vehicle pose (e.g.
        # control/rviz_visualizer, which publishes odom→base_link WITH a
        # translation). Two broadcasters on one TF edge fight, and RViz shows
        # whichever arrived last — so exactly one node may own that edge.
        # Default true keeps imu_viz.launch.py / pix_imu_viz.launch.py working
        # as they always have, where this node IS the only pose source.
        self.declare_parameter('publish_tf', True)

        self._imu_topic = self.get_parameter('imu_topic').value
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._map = self.get_parameter('map_frame').value
        self._calib_n = int(self.get_parameter('calib_samples').value)
        self._mount_rpy = list(self.get_parameter('mount_rpy').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        self._q_ref = None            # reference quaternion (None until calibrated)
        self._calib_buf = []          # samples collected during calibration
        self._recalibrate = False     # set by the reset service
        self._last_warn = self.get_clock().now()

        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._rpy_pub = self.create_publisher(Vector3Stamped, 'imu/rpy', 10)
        self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_service(
            Trigger, 'imu/reset_orientation', self._reset_cb)

        self._publish_static_tf()
        self.get_logger().info(
            f'orientation_node up — IMU topic {self._imu_topic}, '
            f'calibrating over {self._calib_n} samples')

    # ---- static transforms (map->odom identity, base_link->imu_link mount) ----
    def _publish_static_tf(self):
        now = self.get_clock().now().to_msg()

        # base_link->imu_link is the sensor MOUNT: it is ours regardless of who
        # owns the vehicle pose, so it is published either way.
        b2i = TransformStamped()
        b2i.header.stamp = now
        b2i.header.frame_id = self._child
        b2i.child_frame_id = self._imu_frame
        qx, qy, qz, qw = quat_from_euler(*self._mount_rpy)
        b2i.transform.rotation.x = qx
        b2i.transform.rotation.y = qy
        b2i.transform.rotation.z = qz
        b2i.transform.rotation.w = qw

        transforms = [b2i]
        if self._publish_tf:
            # map->odom identity. Skipped when another node owns the pose — it
            # publishes this edge itself, and two broadcasters would fight.
            m2o = TransformStamped()
            m2o.header.stamp = now
            m2o.header.frame_id = self._map
            m2o.child_frame_id = self._parent
            m2o.transform.rotation.w = 1.0
            transforms.append(m2o)

        self._static_tf.sendTransform(transforms)

    def _reset_cb(self, request, response):
        self._q_ref = None
        self._calib_buf = []
        self._recalibrate = True
        response.success = True
        response.message = 'orientation reference cleared; recalibrating'
        self.get_logger().info('reset_orientation: recalibrating reference')
        return response

    def _imu_cb(self, msg: Imu):
        q_cur = normalize((
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w))

        # --- calibration phase: gather reference, publish identity meanwhile ---
        if self._q_ref is None:
            self._calib_buf.append(q_cur)
            if len(self._calib_buf) >= self._calib_n:
                self._q_ref = quat_average(self._calib_buf)
                self._recalibrate = False
                self.get_logger().info('reference captured — orientation zeroed')
            self._broadcast((0.0, 0.0, 0.0, 1.0), msg.header.stamp)
            return

        # --- steady state: current relative to reference (the zeroing) ---
        q_zeroed = quat_relative(self._q_ref, q_cur)
        self._broadcast(q_zeroed, msg.header.stamp)

        r, p, y = euler_from_quat(q_zeroed)
        rpy = Vector3Stamped()
        rpy.header.stamp = msg.header.stamp
        rpy.header.frame_id = self._child
        rpy.vector.x, rpy.vector.y, rpy.vector.z = r, p, y
        self._rpy_pub.publish(rpy)

    def _broadcast(self, q, stamp):
        if not self._publish_tf:
            return          # another node owns odom→base_link; imu/rpy still flows
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._parent
        t.child_frame_id = self._child
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
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
