#!/usr/bin/env python3
"""
ZED2i VSLAM ROS2 Node
=====================
Publishes:
  /zed2i/pose          — geometry_msgs/PoseStamped     (camera pose in world frame)
  /zed2i/odom          — nav_msgs/Odometry             (visual odometry)
  /zed2i/imu/data      — sensor_msgs/Imu               (accelerometer + gyroscope)
  /zed2i/imu/mag       — sensor_msgs/MagneticField     (magnetometer)
  /zed2i/path          — nav_msgs/Path                 (accumulated trajectory)
  /zed2i/point_cloud   — sensor_msgs/PointCloud2       (depth-fused point cloud / pseudo-LiDAR)
  /zed2i/image/left    — sensor_msgs/Image             (left RGB frame)
  /zed2i/image/depth   — sensor_msgs/Image             (32-bit depth map, metres)
  /zed2i/camera_info   — sensor_msgs/CameraInfo        (left camera intrinsics)

TF frames published:  map → odom → base_link → camera_link
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import numpy as np
import math
import struct
import time

# ROS2 message types
from geometry_msgs.msg import (
    PoseStamped, TransformStamped, Quaternion, Vector3, Point
)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import (
    Imu, MagneticField, PointCloud2, PointField, Image, CameraInfo
)
from std_msgs.msg import Header, ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from builtin_interfaces.msg import Time as RosTime

# ZED SDK — imported inside try/except so the node can still load in CI
try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except ImportError:
    ZED_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Helper utilities
# ─────────────────────────────────────────────────────────────────────────────

def sl_translation_to_ros(t) -> Point:
    """Convert ZED SDK Translation to ROS Point (ZED: X-right, Y-up, Z-back → ROS ENU)."""
    return Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))


def sl_orientation_to_ros(o) -> Quaternion:
    """Convert ZED SDK Orientation (x,y,z,w) to ROS Quaternion."""
    return Quaternion(x=float(o[0]), y=float(o[1]), z=float(o[2]), w=float(o[3]))


def make_header(frame_id: str, node: Node) -> Header:
    h = Header()
    h.stamp = node.get_clock().now().to_msg()
    h.frame_id = frame_id
    return h


def rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """Convert 3×3 rotation matrix to quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return Quaternion(x=x, y=y, z=z, w=w)


def numpy_to_pointcloud2(points: np.ndarray, header: Header) -> PointCloud2:
    """
    Convert Nx4 float32 array (X,Y,Z,intensity) to sensor_msgs/PointCloud2.
    Compatible with RViz2 and most LiDAR consumers.
    """
    fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 16
    data = points.astype(np.float32).tobytes()
    msg = PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=point_step * len(points),
        data=data,
    )
    return msg


# ─────────────────────────────────────────────────────────────────────────────
# Main Node
# ─────────────────────────────────────────────────────────────────────────────

class ZED2iVSLAMNode(Node):

    def __init__(self):
        super().__init__('zed2i_vslam')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('resolution',          'HD720')   # HD2K | HD1080 | HD720 | VGA
        self.declare_parameter('fps',                 30)
        self.declare_parameter('depth_mode',          'ULTRA')   # ULTRA | QUALITY | PERFORMANCE | NEURAL
        self.declare_parameter('coordinate_system',   'RIGHT_HANDED_Z_UP_X_FWD')
        self.declare_parameter('enable_spatial_map',  True)
        self.declare_parameter('imu_rate_hz',         200)
        self.declare_parameter('publish_point_cloud', True)
        self.declare_parameter('point_cloud_downsample', 4)      # keep 1/N rows
        self.declare_parameter('image_every_n_frames',  3)       # publish image/depth every N grabs
        self.declare_parameter('pc_every_n_frames',     3)       # publish point cloud every N grabs
        self.declare_parameter('base_frame',          'base_link')
        self.declare_parameter('camera_frame',        'camera_link')
        self.declare_parameter('odom_frame',          'odom')
        self.declare_parameter('map_frame',           'map')

        res_str    = self.get_parameter('resolution').value
        fps        = self.get_parameter('fps').value
        depth_str  = self.get_parameter('depth_mode').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.map_frame    = self.get_parameter('map_frame').value
        self._do_publish_pc    = self.get_parameter('publish_point_cloud').value
        self.pc_ds             = self.get_parameter('point_cloud_downsample').value
        self._img_every_n      = self.get_parameter('image_every_n_frames').value
        self._pc_every_n       = self.get_parameter('pc_every_n_frames').value
        self._img_frame_ctr    = 0
        self._pc_frame_ctr     = 0

        # ── QoS ─────────────────────────────────────────────────────────────
        # Use RELIABLE for everything so RViz2 (which defaults to RELIABLE)
        # can subscribe without QoS-incompatibility warnings.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_pose       = self.create_publisher(PoseStamped,    '/zed2i/pose',          reliable_qos)
        self.pub_odom       = self.create_publisher(Odometry,       '/zed2i/odom',          sensor_qos)
        self.pub_imu        = self.create_publisher(Imu,            '/zed2i/imu/data',      sensor_qos)
        self.pub_mag        = self.create_publisher(MagneticField,  '/zed2i/imu/mag',       sensor_qos)
        self.pub_path       = self.create_publisher(Path,           '/zed2i/path',          reliable_qos)
        self.pub_pc         = self.create_publisher(PointCloud2,    '/zed2i/point_cloud',   sensor_qos)
        self.pub_img_left   = self.create_publisher(Image,          '/zed2i/image/left',    sensor_qos)
        self.pub_img_depth  = self.create_publisher(Image,          '/zed2i/image/depth',   sensor_qos)
        self.pub_cam_info   = self.create_publisher(CameraInfo,     '/zed2i/camera_info',   reliable_qos)

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_broadcaster        = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        # ── Path history ─────────────────────────────────────────────────────
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        # ── Previous pose for odometry delta ─────────────────────────────────
        self._prev_pose_matrix: np.ndarray | None = None
        self._prev_imu_ts: float = 0.0

        # ── ZED SDK init ─────────────────────────────────────────────────────
        if ZED_AVAILABLE:
            self._init_zed(res_str, fps, depth_str)
            period = 1.0 / fps
            self.timer = self.create_timer(period, self._grab_callback)
        else:
            self.get_logger().warn(
                'pyzed not found — running in SIMULATION mode (synthetic data). '
                'Install the ZED SDK and pyzed to use real hardware.'
            )
            self._sim_t = 0.0
            self.timer = self.create_timer(1.0 / 15.0, self._sim_callback)

        self.get_logger().info('ZED2i VSLAM node started.')

    # ─────────────────────────────────────────────────────────────────────────
    # ZED SDK initialisation
    # ─────────────────────────────────────────────────────────────────────────

    def _init_zed(self, res_str: str, fps: int, depth_str: str):
        res_map = {
            'HD2K':  sl.RESOLUTION.HD2K,
            'HD1080':sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA':   sl.RESOLUTION.VGA,
        }
        depth_map = {
            'ULTRA':       sl.DEPTH_MODE.ULTRA,
            'QUALITY':     sl.DEPTH_MODE.QUALITY,
            'PERFORMANCE': sl.DEPTH_MODE.PERFORMANCE,
            'NEURAL':      sl.DEPTH_MODE.NEURAL,
            'NEURAL_PLUS': sl.DEPTH_MODE.NEURAL_PLUS if hasattr(sl.DEPTH_MODE, 'NEURAL_PLUS') else sl.DEPTH_MODE.NEURAL,
        }

        init_params = sl.InitParameters()
        init_params.camera_resolution          = res_map.get(res_str, sl.RESOLUTION.VGA)
        init_params.camera_fps                 = fps
        # ULTRA is deprecated in SDK 4.x — fall back to NEURAL gracefully
        requested = depth_map.get(depth_str, sl.DEPTH_MODE.NEURAL)
        if depth_str == 'ULTRA':
            self.get_logger().warn('ULTRA depth mode is deprecated in SDK 4.x; using NEURAL instead.')
            requested = sl.DEPTH_MODE.NEURAL
        init_params.depth_mode                 = requested
        init_params.coordinate_units           = sl.UNIT.METER
        init_params.coordinate_system          = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_minimum_distance     = 0.3
        init_params.depth_maximum_distance     = 15.0
        # Cap GPU memory to leave headroom for RViz2 / other processes
        try:
            init_params.sdk_gpu_id = 0
        except AttributeError:
            pass

        self.zed = sl.Camera()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(f'Failed to open ZED2i: {status}')
            raise RuntimeError(f'ZED open failed: {status}')

        # Positional tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_area_memory    = True
        tracking_params.enable_imu_fusion     = True
        tracking_params.set_gravity_as_origin = True
        self.zed.enable_positional_tracking(tracking_params)

        # Spatial mapping — disabled by default on Jetson (GPU-memory intensive).
        # Enable with enable_spatial_map:=true only if you have headroom.
        if self.get_parameter('enable_spatial_map').value:
            map_params = sl.SpatialMappingParameters()
            try:
                map_params.resolution_meter = 0.08   # coarser = less VRAM
                map_params.range_meter      = 8.0
                map_params.save_texture     = False
            except AttributeError:
                map_params = sl.SpatialMappingParameters(
                    resolution=sl.SpatialMappingParameters.MAPPING_RESOLUTION.LOW,
                    range=sl.SpatialMappingParameters.MAPPING_RANGE.MEDIUM,
                    save_texture=False,
                )
            err = self.zed.enable_spatial_mapping(map_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().warn(f'Spatial mapping not enabled: {err}')

        # Pre-allocate SDK containers — force CPU memory so GPU heap is not touched
        # for image/depth retrieval. This is the key fix for Jetson CUDA OOM errors.
        self._zed_image   = sl.Mat()
        self._zed_depth   = sl.Mat()
        self._zed_pc      = sl.Mat()
        self._zed_pose    = sl.Pose()
        self._zed_imu     = sl.SensorsData()
        self._zed_runtime = sl.RuntimeParameters(enable_fill_mode=False)
        # Disable fill mode — less GPU work, avoids CUDA alloc for hole-filling

        # Camera intrinsics for CameraInfo
        calib = self.zed.get_camera_information().camera_configuration.calibration_parameters
        left  = calib.left_cam
        self._cam_info = self._build_camera_info(left)
        self.get_logger().info('ZED2i opened successfully.')

    def _build_camera_info(self, left_cam) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = self.camera_frame
        msg.width  = self.zed.get_camera_information().camera_configuration.resolution.width
        msg.height = self.zed.get_camera_information().camera_configuration.resolution.height
        fx, fy = left_cam.fx, left_cam.fy
        cx, cy = left_cam.cx, left_cam.cy
        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        msg.d = list(left_cam.disto)
        msg.distortion_model = 'plumb_bob'
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    # ─────────────────────────────────────────────────────────────────────────
    # Main grab callback (real hardware)
    # ─────────────────────────────────────────────────────────────────────────

    def _grab_callback(self):
        grab_err = self.zed.grab(self._zed_runtime)
        if grab_err != sl.ERROR_CODE.SUCCESS:
            # CUDA errors (700 = illegal address) mean GPU OOM — log and keep retrying
            # rather than crashing the node.
            if grab_err in (sl.ERROR_CODE.CAMERA_NOT_DETECTED,
                            sl.ERROR_CODE.SENSORS_NOT_AVAILABLE):
                self.get_logger().error(f'Grab fatal: {grab_err}', throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(f'Grab skipped: {grab_err}', throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()

        # ── Pose / Odometry — pure CPU, no CUDA alloc ────────────────────────
        state = self.zed.get_position(self._zed_pose, sl.REFERENCE_FRAME.WORLD)
        if state == sl.POSITIONAL_TRACKING_STATE.OK:
            np_pose = self._pose_to_matrix(self._zed_pose)
            self._publish_pose(np_pose, stamp)
            self._publish_path(np_pose, stamp)
            self._publish_tf(np_pose, stamp)
            self._publish_odometry(np_pose, stamp)

        # ── IMU — pure CPU ───────────────────────────────────────────────────
        self.zed.get_sensors_data(self._zed_imu, sl.TIME_REFERENCE.IMAGE)
        self._publish_imu(self._zed_imu, stamp)

        # ── Images / Depth — throttled + subscriber-gated ────────────────────
        self._img_frame_ctr += 1
        if self._img_frame_ctr >= self._img_every_n:
            self._img_frame_ctr = 0
            want_img   = self.pub_img_left.get_subscription_count() > 0
            want_depth = self.pub_img_depth.get_subscription_count() > 0
            if want_img:
                try:
                    err = self.zed.retrieve_image(self._zed_image, sl.VIEW.LEFT,
                                                  sl.MEM.CPU)
                    if err == sl.ERROR_CODE.SUCCESS:
                        self._publish_image_left(self._zed_image, stamp)
                except Exception as e:
                    self.get_logger().warn(f'Image retrieve failed: {e}', throttle_duration_sec=5.0)
            if want_depth:
                try:
                    err = self.zed.retrieve_measure(self._zed_depth, sl.MEASURE.DEPTH,
                                                    sl.MEM.CPU)
                    if err == sl.ERROR_CODE.SUCCESS:
                        self._publish_depth(self._zed_depth, stamp)
                except Exception as e:
                    self.get_logger().warn(f'Depth retrieve failed: {e}', throttle_duration_sec=5.0)

        # ── Point cloud — throttled + subscriber-gated ───────────────────────
        if self._do_publish_pc and self.pub_pc.get_subscription_count() > 0:
            self._pc_frame_ctr += 1
            if self._pc_frame_ctr >= self._pc_every_n:
                self._pc_frame_ctr = 0
                try:
                    err = self.zed.retrieve_measure(self._zed_pc, sl.MEASURE.XYZRGBA,
                                                    sl.MEM.CPU)
                    if err == sl.ERROR_CODE.SUCCESS:
                        self._publish_point_cloud(self._zed_pc, stamp)
                except Exception as e:
                    self.get_logger().warn(f'PC retrieve failed: {e}', throttle_duration_sec=5.0)

        # ── Camera info ──────────────────────────────────────────────────────
        self._cam_info.header.stamp = stamp
        self.pub_cam_info.publish(self._cam_info)

    # ─────────────────────────────────────────────────────────────────────────
    # Publish helpers (real hardware)
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _pose_to_matrix(zed_pose) -> np.ndarray:
        """
        Convert sl.Pose to a 4x4 numpy homogeneous matrix.

        Strategy:
          1. Try pose_data(MEM.CPU).m — available in all SDK versions, returns a
             flat 16-element list that is the most reliable extraction method.
          2. Fall back to get_translation() + get_orientation() using _vec3/_quat.
        """
        # Primary: pose_data with optional Transform argument (SDK 3.x–5.x)
        try:
            tf = sl.Transform()
            zed_pose.pose_data(tf)
            arr = np.array(tf.m, dtype=np.float64).reshape(4, 4)
            # Sanity-check: bottom row must be [0,0,0,1]
            if abs(arr[3, 3] - 1.0) < 0.1:
                return arr
        except Exception:
            pass

        # Fallback: reconstruct from translation + orientation
        t = zed_pose.get_translation()
        o = zed_pose.get_orientation()

        def _to_arr(v, n):
            try:
                a = np.asarray(v.get(), dtype=np.float64).ravel()
                if a.size >= n:
                    return a[:n]
            except (TypeError, ValueError, AttributeError):
                pass
            try:
                a = np.asarray(v, dtype=np.float64).ravel()
                if a.size >= n:
                    return a[:n]
            except (TypeError, ValueError):
                pass
            for attrs in [('x','y','z','w')[:n], ('ox','oy','oz','ow')[:n]]:
                try:
                    return np.array([float(getattr(v, a)) for a in attrs])
                except AttributeError:
                    pass
            raise TypeError(f"Cannot extract {n}-vector from {type(v)}")

        tx, ty, tz     = _to_arr(t, 3)
        ox, oy, oz, ow = _to_arr(o, 4)

        x2, y2, z2 = ox*2, oy*2, oz*2
        xx, yy, zz = ox*x2, oy*y2, oz*z2
        xy, xz, yz = ox*y2, ox*z2, oy*z2
        wx, wy, wz = ow*x2, ow*y2, ow*z2
        R = np.array([
            [1-(yy+zz), xy-wz,     xz+wy    ],
            [xy+wz,     1-(xx+zz), yz-wx    ],
            [xz-wy,     yz+wx,     1-(xx+yy)],
        ])
        m = np.eye(4)
        m[:3, :3] = R
        m[:3,  3] = [tx, ty, tz]
        return m

    def _publish_pose(self, pose_matrix: np.ndarray, stamp):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.map_frame
        t = pose_matrix[:3, 3]
        R = pose_matrix[:3, :3]
        msg.pose.position    = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
        msg.pose.orientation = rotation_matrix_to_quaternion(R)
        self.pub_pose.publish(msg)

    def _publish_path(self, pose_matrix: np.ndarray, stamp):
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self.map_frame
        t = pose_matrix[:3, 3]
        R = pose_matrix[:3, :3]
        ps.pose.position    = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
        ps.pose.orientation = rotation_matrix_to_quaternion(R)
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(ps)
        self.pub_path.publish(self.path_msg)

    def _publish_tf(self, pose_matrix: np.ndarray, stamp):
        ts = TransformStamped()
        ts.header.stamp    = stamp
        ts.header.frame_id = self.map_frame
        ts.child_frame_id  = self.base_frame
        t = pose_matrix[:3, 3]
        R = pose_matrix[:3, :3]
        q = rotation_matrix_to_quaternion(R)
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        ts.transform.rotation      = q
        self.tf_broadcaster.sendTransform(ts)

    def _publish_odometry(self, pose_matrix: np.ndarray, stamp):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id  = self.base_frame
        t = pose_matrix[:3, 3]
        R = pose_matrix[:3, :3]
        msg.pose.pose.position    = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
        msg.pose.pose.orientation = rotation_matrix_to_quaternion(R)
        if self._prev_pose_matrix is not None:
            dt = 1.0 / self.get_parameter('fps').value
            delta = np.linalg.inv(self._prev_pose_matrix) @ pose_matrix
            msg.twist.twist.linear.x  = float(delta[0, 3]) / dt
            msg.twist.twist.linear.y  = float(delta[1, 3]) / dt
            msg.twist.twist.linear.z  = float(delta[2, 3]) / dt
        self._prev_pose_matrix = pose_matrix.copy()
        self.pub_odom.publish(msg)

    @staticmethod
    @staticmethod
    def _vec3(v) -> tuple:
        """
        Extract (x, y, z) from any pyzed vector type.

        pyzed bindings vary by SDK version and platform — the only universally
        reliable conversion is np.array(v), which the C++ binding always supports.
        Attribute / subscript access is tried as a fast-path but must never be
        the sole fallback since both can be absent simultaneously.
        """
        try:
            a = np.asarray(v.get(), dtype=np.float64).ravel()
            if a.size >= 3:
                return float(a[0]), float(a[1]), float(a[2])
        except (TypeError, ValueError, AttributeError):
            pass
        try:
            a = np.asarray(v, dtype=np.float64).ravel()
            if a.size >= 3:
                return float(a[0]), float(a[1]), float(a[2])
        except (TypeError, ValueError):
            pass
        # Named attribute fast-paths (some SDK versions only)
        for attrs in [('x','y','z'), ('ox','oy','oz')]:
            try:
                return tuple(float(getattr(v, a)) for a in attrs)
            except AttributeError:
                pass
        raise TypeError(f"Cannot extract vec3 from {type(v)}: {v}")

    @staticmethod
    def _quat(o) -> tuple:
        """
        Extract (x, y, z, w) from any pyzed orientation type.

        Same strategy as _vec3: np.array() first, attribute access as fallback.
        Note the w component is index 3 in pyzed's layout.
        """
        try:
            a = np.asarray(o.get(), dtype=np.float64).ravel()
            if a.size >= 4:
                return float(a[0]), float(a[1]), float(a[2]), float(a[3])
        except (TypeError, ValueError, AttributeError):
            pass
        try:
            a = np.asarray(o, dtype=np.float64).ravel()
            if a.size >= 4:
                return float(a[0]), float(a[1]), float(a[2]), float(a[3])
        except (TypeError, ValueError):
            pass
        for attrs in [('x','y','z','w'), ('ox','oy','oz','ow')]:
            try:
                return tuple(float(getattr(o, a)) for a in attrs)
            except AttributeError:
                pass
        raise TypeError(f"Cannot extract quat from {type(o)}: {o}")

    def _publish_imu(self, sensors_data, stamp):
        imu_data = sensors_data.get_imu_data()

        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = self.camera_frame

        ax, ay, az = self._vec3(imu_data.get_linear_acceleration())
        gx, gy, gz = self._vec3(imu_data.get_angular_velocity())
        ox, oy, oz, ow = self._quat(imu_data.get_pose().get_orientation())

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity.x    = gx
        imu_msg.angular_velocity.y    = gy
        imu_msg.angular_velocity.z    = gz
        imu_msg.orientation.x         = ox
        imu_msg.orientation.y         = oy
        imu_msg.orientation.z         = oz
        imu_msg.orientation.w         = ow

        # Pull SDK covariance matrices when available, else use fixed diagonals
        try:
            la_cov = imu_data.linear_acceleration_covariance   # 3x3 as list[9]
            av_cov = imu_data.angular_velocity_covariance
            for i in range(9):
                imu_msg.linear_acceleration_covariance[i] = float(la_cov[i])
                imu_msg.angular_velocity_covariance[i]    = float(av_cov[i])
        except AttributeError:
            imu_msg.linear_acceleration_covariance[0] = 1e-4
            imu_msg.linear_acceleration_covariance[4] = 1e-4
            imu_msg.linear_acceleration_covariance[8] = 1e-4
            imu_msg.angular_velocity_covariance[0]    = 1e-5
            imu_msg.angular_velocity_covariance[4]    = 1e-5
            imu_msg.angular_velocity_covariance[8]    = 1e-5

        imu_msg.orientation_covariance[0] = 1e-4
        imu_msg.orientation_covariance[4] = 1e-4
        imu_msg.orientation_covariance[8] = 1e-4

        self.pub_imu.publish(imu_msg)

        # Magnetometer
        mag_data = sensors_data.get_magnetometer_data()
        mag_msg  = MagneticField()
        mag_msg.header = imu_msg.header
        mx, my, mz = self._vec3(mag_data.get_magnetic_field_calibrated())
        mag_msg.magnetic_field.x = mx * 1e-6  # uT -> T
        mag_msg.magnetic_field.y = my * 1e-6
        mag_msg.magnetic_field.z = mz * 1e-6
        self.pub_mag.publish(mag_msg)

    def _publish_image_left(self, zed_mat, stamp):
        np_img = zed_mat.get_data()   # BGRA uint8
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.camera_frame
        msg.height          = np_img.shape[0]
        msg.width           = np_img.shape[1]
        msg.encoding        = 'bgra8'
        msg.step            = msg.width * 4
        msg.data            = np_img.tobytes()
        self.pub_img_left.publish(msg)

    def _publish_depth(self, zed_mat, stamp):
        np_depth = zed_mat.get_data().astype(np.float32)
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.camera_frame
        msg.height          = np_depth.shape[0]
        msg.width           = np_depth.shape[1]
        msg.encoding        = '32FC1'
        msg.step            = msg.width * 4
        msg.data            = np_depth.flatten().tobytes()
        self.pub_img_depth.publish(msg)

    def _publish_point_cloud(self, zed_mat, stamp):
        np_pc = zed_mat.get_data()  # H×W×4 (X,Y,Z,RGBA packed)
        # Downsample rows for performance
        np_pc = np_pc[::self.pc_ds, ::self.pc_ds]
        H, W, _ = np_pc.shape
        xyz    = np_pc[:, :, :3].reshape(-1, 3)
        rgba   = np_pc[:, :, 3].reshape(-1)         # float view of packed RGBA
        # Filter invalid (NaN/Inf)
        valid = np.isfinite(xyz).all(axis=1)
        xyz   = xyz[valid]
        rgba  = rgba[valid]
        # Intensity from luminance (dot-product, float32 throughout)
        rgba_int  = rgba.view(np.uint8).reshape(-1, 4)
        luma_w    = np.array([0.299, 0.587, 0.114], dtype=np.float32) / 255.0
        intensity = rgba_int[:, :3].astype(np.float32) @ luma_w
        points = np.column_stack([xyz, intensity])
        header = Header()
        header.stamp    = stamp
        header.frame_id = self.camera_frame
        self.pub_pc.publish(numpy_to_pointcloud2(points, header))

    # ─────────────────────────────────────────────────────────────────────────
    # Static TF: base_link → camera_link  (ZED2i mount offset)
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_static_transforms(self):
        """Publish static base_link → camera_link transform.
        Adjust the translation to match your physical mount."""
        ts = TransformStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.header.frame_id = self.base_frame
        ts.child_frame_id  = self.camera_frame
        # ZED2i typically mounted ~15 cm forward, ~10 cm up from base
        ts.transform.translation.x = 0.15
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.10
        ts.transform.rotation.w    = 1.0   # no rotation (camera faces forward)
        self.static_tf_broadcaster.sendTransform(ts)

    # ─────────────────────────────────────────────────────────────────────────
    # Simulation mode (no ZED SDK)
    # ─────────────────────────────────────────────────────────────────────────

    def _sim_callback(self):
        """Generate synthetic motion data — lemniscate (figure-8) trajectory."""
        t = self._sim_t
        self._sim_t += 1.0 / 15.0

        # Lemniscate parametric path
        a    = 3.0
        denom = 1 + math.sin(t) ** 2
        x    = a * math.cos(t) / denom
        y    = a * math.sin(t) * math.cos(t) / denom
        z    = 0.5 * math.sin(2 * t)
        yaw  = math.atan2(
            a * (math.cos(t)**2 - math.sin(t)**2) / denom - 2*a*math.cos(t)*math.sin(t)**2*math.cos(t)/denom**2,
            -a * math.sin(t) / denom - a * math.cos(t) * 2 * math.sin(t) * math.cos(t) / denom**2,
        )

        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        pose_mat = np.eye(4)
        pose_mat[:3, :3] = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                                     [math.sin(yaw),  math.cos(yaw), 0],
                                     [0,              0,             1]])
        pose_mat[:3, 3] = [x, y, z]

        stamp = self.get_clock().now().to_msg()

        self._publish_pose(pose_mat, stamp)
        self._publish_path(pose_mat, stamp)
        self._publish_tf(pose_mat, stamp)
        self._publish_odometry(pose_mat, stamp)

        # Synthetic IMU
        imu = Imu()
        imu.header.stamp    = stamp
        imu.header.frame_id = self.camera_frame
        imu.linear_acceleration.z  = 9.81
        imu.angular_velocity.z     = 0.05 * math.cos(t)
        imu.orientation.x          = 0.0
        imu.orientation.y          = 0.0
        imu.orientation.z          = sy
        imu.orientation.w          = cy
        imu.orientation_covariance[0]         = 1e-4
        imu.orientation_covariance[4]         = 1e-4
        imu.orientation_covariance[8]         = 1e-4
        imu.angular_velocity_covariance[0]    = 1e-5
        imu.angular_velocity_covariance[4]    = 1e-5
        imu.angular_velocity_covariance[8]    = 1e-5
        imu.linear_acceleration_covariance[0] = 1e-4
        imu.linear_acceleration_covariance[4] = 1e-4
        imu.linear_acceleration_covariance[8] = 1e-4
        self.pub_imu.publish(imu)

        # Synthetic magnetometer
        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x = 2.0e-5
        mag.magnetic_field.y = 0.5e-5
        mag.magnetic_field.z = -4.3e-5
        self.pub_mag.publish(mag)

        # Synthetic point cloud (flat disc of points with noise)
        N = 500
        angles = np.linspace(0, 2*np.pi, N)
        radii  = np.random.uniform(0.5, 5.0, N)
        px     = radii * np.cos(angles) + x
        py     = radii * np.sin(angles) + y
        pz     = np.random.normal(0, 0.05, N) + z
        pi_    = np.ones(N) * 0.5
        pts    = np.column_stack([px, py, pz, pi_]).astype(np.float32)
        header = Header()
        header.stamp    = stamp
        header.frame_id = self.camera_frame
        self.pub_pc.publish(numpy_to_pointcloud2(pts, header))

        self._prev_pose_matrix = pose_mat.copy()

    # ─────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if ZED_AVAILABLE and hasattr(self, 'zed'):
            self.get_logger().info('Disabling ZED tracking and closing camera...')
            self.zed.disable_spatial_mapping()
            self.zed.disable_positional_tracking()
            self.zed.close()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ZED2iVSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
