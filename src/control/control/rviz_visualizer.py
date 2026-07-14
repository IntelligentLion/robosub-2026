#!/usr/bin/env python3
"""rviz_visualizer — subscribe-only view of what motion_node is doing.

Decoupled by construction: it reads the debug topics motion_node already
publishes and never publishes movement_command. Leave it out of a competition
launch and nothing about the vehicle's behaviour changes.

Subscribes (all published by motion_node / thruster_node)
  heading/{current,target,error,yaw_correction}, depth/{current,target},
  motion/{forward_cmd,vertical_cmd}   (Float32)
  submerge/state, pixhawk/mode        (String)
  vslam/odometry                      (Odometry) — only when pose_source=zed
  viz/target_waypoint                 (PointStamped) — optional, see below

Publishes
  viz/markers   (MarkerArray)  heading arrows, error arc, text, depth plane
  viz/path      (Path)         where we have been
  TF            map → odom → base_link

Position honesty: with only a Bar02 and a Pixhawk IMU there is NO XY position
sensor. The default pose_source dead-reckons from commanded velocity, and both
the path colour and the on-screen text say so. See pose_source.py.

The target-waypoint marker subscribes to viz/target_waypoint but nothing in this
stack publishes it today — it is a hook for a future waypoint follower, and
renders nothing until something does.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from control.pose_source import DeadReckonPose, ZedOdomPose

FLOAT_TOPICS = ('heading/current', 'heading/target', 'heading/error',
                'heading/yaw_correction', 'depth/current', 'depth/target',
                'motion/forward_cmd', 'motion/vertical_cmd')

ARROW_LEN = 1.0            # m — heading arrows
ESTIMATE_RGBA = (1.0, 0.55, 0.0, 1.0)     # orange: dead-reckoned, drifting
MEASURED_RGBA = (0.1, 0.9, 0.3, 1.0)      # green: a real position sensor
CURRENT_RGBA = (0.2, 0.6, 1.0, 1.0)       # blue: where we point
DESIRED_RGBA = (1.0, 0.9, 0.1, 1.0)       # yellow: where we want to point
ERROR_RGBA = (1.0, 0.2, 0.2, 0.9)         # red: the gap between them


def _quat_z(yaw):
    """Quaternion for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RvizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')
        self.declare_parameter('pose_source', 'pixhawk_imu')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('path_length', 500)
        self.declare_parameter('surge_scale', 0.5)
        self.declare_parameter('strafe_scale', 0.4)

        self._map = str(self.get_parameter('map_frame').value)
        self._odom = str(self.get_parameter('odom_frame').value)
        self._base = str(self.get_parameter('base_frame').value)
        self._path_len = int(self.get_parameter('path_length').value)

        source = str(self.get_parameter('pose_source').value).lower()
        if source == 'zed':
            self._pose = ZedOdomPose()
            self.create_subscription(
                Odometry, 'vslam/odometry', self._on_odom, 10)
            self.get_logger().info('pose_source=zed — relaying vslam/odometry')
        else:
            self._pose = DeadReckonPose(
                surge_scale=float(self.get_parameter('surge_scale').value),
                strafe_scale=float(self.get_parameter('strafe_scale').value))
            self.get_logger().warn(
                'pose_source=pixhawk_imu — XY is DEAD-RECKONED from commanded '
                'velocity and WILL drift. There is no XY position sensor on '
                'this vehicle. Depth is real; the track is only indicative.')

        self._vals = {t: float('nan') for t in FLOAT_TOPICS}
        for topic in FLOAT_TOPICS:
            self.create_subscription(
                Float32, topic,
                lambda msg, t=topic: self._vals.__setitem__(t, msg.data), 10)

        self._sub_state = 'idle'
        self._mode = 'UNKNOWN'
        self.create_subscription(
            String, 'submerge/state',
            lambda m: setattr(self, '_sub_state', m.data), 10)
        self.create_subscription(
            String, 'pixhawk/mode', lambda m: setattr(self, '_mode', m.data), 10)

        self._waypoint = None
        self.create_subscription(
            PointStamped, 'viz/target_waypoint', self._on_waypoint, 10)

        # Full attitude for the TF. The pose source only carries yaw, but this
        # node owns odom→base_link, and dropping roll/pitch would hide exactly
        # the thing ALT_HOLD's self-levelling is supposed to be doing.
        self._quat = None
        self.create_subscription(
            Imu, 'pixhawk/imu/data', self._on_imu, qos_profile_sensor_data)

        self._marker_pub = self.create_publisher(MarkerArray, 'viz/markers', 10)
        self._path_pub = self.create_publisher(Path, 'viz/path', 10)
        self._tf = TransformBroadcaster(self)

        self._path = Path()
        self._path.header.frame_id = self._odom
        self._last_tick = self.get_clock().now()

        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'rviz_visualizer up — {rate:.0f} Hz')

    # ─── inputs ─────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose.set_pose(p.x, p.y, p.z, yaw)

    def _on_waypoint(self, msg: PointStamped):
        self._waypoint = (msg.point.x, msg.point.y, msg.point.z)

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._quat = (q.x, q.y, q.z, q.w)

    def _v(self, key):
        return self._vals.get(key, float('nan'))

    # ─── tick ───────────────────────────────────────────────────────

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now

        if isinstance(self._pose, DeadReckonPose):
            yaw = self._v('heading/current')
            depth = self._v('depth/current')
            self._pose.update(
                surge_cmd=_or0(self._v('motion/forward_cmd')),
                strafe_cmd=0.0,
                yaw_rad=None if math.isnan(yaw) else yaw,
                depth_m=None if math.isnan(depth) else depth,
                dt=dt)

        pose = self._pose.pose()
        if pose is None:
            return
        x, y, z, yaw = pose

        self._publish_tf(now, x, y, z, yaw)
        self._publish_path(now, x, y, z)
        self._publish_markers(now, x, y, z, yaw)

    # ─── outputs ────────────────────────────────────────────────────

    def _publish_tf(self, now, x, y, z, yaw):
        # map → odom is identity: with no global fix there is nothing to
        # correct odom against. It exists so RViz has the standard frame tree
        # and a future localization node can start publishing it for real.
        m2o = TransformStamped()
        m2o.header.stamp = now.to_msg()
        m2o.header.frame_id = self._map
        m2o.child_frame_id = self._odom
        m2o.transform.rotation.w = 1.0
        self._tf.sendTransform(m2o)

        o2b = TransformStamped()
        o2b.header.stamp = now.to_msg()
        o2b.header.frame_id = self._odom
        o2b.child_frame_id = self._base
        o2b.transform.translation.x = float(x)
        o2b.transform.translation.y = float(y)
        o2b.transform.translation.z = float(z)
        # Full attitude when the IMU is up (so roll/pitch — ALT_HOLD's
        # self-levelling — are visible); yaw-only as a fallback.
        qx, qy, qz, qw = self._quat if self._quat is not None else _quat_z(yaw)
        o2b.transform.rotation.x = qx
        o2b.transform.rotation.y = qy
        o2b.transform.rotation.z = qz
        o2b.transform.rotation.w = qw
        self._tf.sendTransform(o2b)

    def _publish_path(self, now, x, y, z):
        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = self._odom
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self._path.poses.append(ps)
        if len(self._path.poses) > self._path_len:
            self._path.poses = self._path.poses[-self._path_len:]
        self._path.header.stamp = now.to_msg()
        self._path_pub.publish(self._path)

    def _publish_markers(self, now, x, y, z, yaw):
        arr = MarkerArray()
        stamp = now.to_msg()
        estimated = self._pose.is_estimate()

        def base(mid, mtype, rgba, scale):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self._odom
            m.ns = 'motion'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.scale.x, m.scale.y, m.scale.z = scale
            m.pose.orientation.w = 1.0
            return m

        origin = Point(x=float(x), y=float(y), z=float(z))

        # Current heading (blue) and desired heading (yellow), as arrows from
        # the vehicle. The gap between them IS the heading error.
        cur = self._v('heading/current')
        tgt = self._v('heading/target')
        if not math.isnan(cur):
            a = base(1, Marker.ARROW, CURRENT_RGBA, (0.06, 0.12, 0.0))
            a.points = [origin, _ahead(origin, cur, ARROW_LEN)]
            arr.markers.append(a)
        if not math.isnan(tgt):
            a = base(2, Marker.ARROW, DESIRED_RGBA, (0.06, 0.12, 0.0))
            a.points = [origin, _ahead(origin, tgt, ARROW_LEN)]
            arr.markers.append(a)

        # The arc between them, drawn thick when the error is large.
        if not math.isnan(cur) and not math.isnan(tgt):
            arc = base(3, Marker.LINE_STRIP, ERROR_RGBA, (0.03, 0.0, 0.0))
            err = _wrap(cur - tgt)
            steps = max(2, int(abs(err) / 0.05) + 2)
            arc.points = [
                _ahead(origin, tgt + err * i / (steps - 1), ARROW_LEN * 0.85)
                for i in range(steps)]
            arr.markers.append(arc)

        # Target-depth plane: makes depth error visible in space, not just text.
        tgt_depth = self._v('depth/target')
        if not math.isnan(tgt_depth) and tgt_depth > 0.0:
            plane = base(4, Marker.CUBE, (0.3, 0.6, 1.0, 0.15), (4.0, 4.0, 0.02))
            plane.pose.position.x = float(x)
            plane.pose.position.y = float(y)
            plane.pose.position.z = float(tgt_depth)
            arr.markers.append(plane)

        if self._waypoint is not None:
            wp = base(5, Marker.SPHERE, (1.0, 0.3, 1.0, 0.9), (0.3, 0.3, 0.3))
            wp.pose.position.x, wp.pose.position.y, wp.pose.position.z = (
                float(v) for v in self._waypoint)
            arr.markers.append(wp)

        text = base(6, Marker.TEXT_VIEW_FACING,
                    ESTIMATE_RGBA if estimated else MEASURED_RGBA,
                    (0.0, 0.0, 0.18))
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = float(z) - 1.0        # float it above the sub
        text.text = self._status_text(estimated)
        arr.markers.append(text)

        self._marker_pub.publish(arr)

    def _status_text(self, estimated):
        def deg(key):
            v = self._v(key)
            return 'n/a' if math.isnan(v) else f'{math.degrees(v):+.1f}°'

        def num(key, unit='', fmt='{:+.2f}'):
            v = self._v(key)
            return 'n/a' if math.isnan(v) else fmt.format(v) + unit

        lines = [
            f'mode      {self._mode}',
            f'submerge  {self._sub_state}',
            f'depth     {num("depth/current", " m")}'
            f'  →  {num("depth/target", " m")}',
            f'heading   {deg("heading/current")}  →  {deg("heading/target")}',
            f'error     {deg("heading/error")}',
            f'yaw corr  {num("heading/yaw_correction")}',
            f'forward   {num("motion/forward_cmd")}',
            f'vertical  {num("motion/vertical_cmd")}',
        ]
        if estimated:
            lines.append('')
            lines.append('POSITION ESTIMATED (dead-reckoned)')
            lines.append('no XY sensor — track drifts')
        return '\n'.join(lines)


def _or0(v):
    return 0.0 if math.isnan(v) else v


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _ahead(origin, yaw, dist):
    return Point(x=origin.x + dist * math.cos(yaw),
                 y=origin.y + dist * math.sin(yaw),
                 z=origin.z)


def main(args=None):
    rclpy.init(args=args)
    node = RvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
