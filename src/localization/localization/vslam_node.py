#!/usr/bin/env python3
"""ZED-based VSLAM node using the ZED SDK positional tracking.

This node opens the ZED2i camera, enables positional tracking, and publishes
the camera pose as `vslam/odometry` (nav_msgs/Odometry) and a `vslam/path`.

Parameters:
  - zed_fps (int): desired camera FPS
  - enable_area_memory (bool): enable area memory (mapping) when supported
  - svo (str): optional SVO input file
  - frame_id (str): child frame id for published odom (default: zed_left_camera_frame)
  - odom_frame (str): odom frame id (default: odom)
"""

import math
from time import sleep

import rclpy
from rclpy.node import Node

import pyzed.sl as sl
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from std_srvs.srv import Trigger as TriggerSrv
from rclpy.callback_groups import ReentrantCallbackGroup

# Bounds for the area-map export busy-wait (NASA rule 2: every loop has a
# strict, compile-time upper bound so a stuck ZED export can never hang init).
_AREA_EXPORT_MAX_POLLS = 1000
_AREA_EXPORT_POLL_S = 0.01


def _quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _wait_area_export(zed, get_state, exporting, success):
    """Bounded poll of the ZED area-export state. Returns the terminal state
    (or the last polled state if the bound is hit)."""
    state = get_state()
    for _ in range(_AREA_EXPORT_MAX_POLLS):
        if state != exporting:
            break
        sleep(_AREA_EXPORT_POLL_S)
        state = get_state()
    return state


class VSLAMZedNode(Node):
    def __init__(self):
        super().__init__('vslam_zed_node')

        self.declare_parameter('zed_fps', 30)
        self.declare_parameter('enable_area_memory', False)
        self.declare_parameter('area_map_path', '')
        self.declare_parameter('save_area_on_exit', False)
        # TF tuning offsets (degrees and meters)
        self.declare_parameter('frame_rot_z_deg', 0.0)
        self.declare_parameter('frame_trans_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('svo', '')
        self.declare_parameter('frame_id', 'zed_left_camera_frame')
        self.declare_parameter('odom_frame', 'odom')

        self.zed_fps = self.get_parameter('zed_fps').value
        self.enable_area_memory = self.get_parameter('enable_area_memory').value
        self.area_map_path = self.get_parameter('area_map_path').value
        self.save_area_on_exit = self.get_parameter('save_area_on_exit').value
        self.frame_rot_z_deg = float(self.get_parameter('frame_rot_z_deg').value)
        self.frame_trans_offset = list(self.get_parameter('frame_trans_offset').value)
        self.svo = self.get_parameter('svo').value
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame = self.get_parameter('odom_frame').value

        self.odom_pub = self.create_publisher(Odometry, 'vslam/odometry', 10)
        self.path_pub = self.create_publisher(Path, 'vslam/path', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._cb_group = ReentrantCallbackGroup()
        # Services to save/load area maps at runtime
        self.create_service(TriggerSrv, 'vslam/save_area', self._handle_save_area, callback_group=self._cb_group)
        self.create_service(TriggerSrv, 'vslam/load_area', self._handle_load_area, callback_group=self._cb_group)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame

        # ZED objects
        self.zed = None
        self.get_logger().info('vslam_zed_node starting')

        # Start ZED camera in a background thread to avoid blocking init
        from threading import Thread
        Thread(target=self._run_zed_loop, daemon=True).start()

    def _run_zed_loop(self):
        try:
            input_type = sl.InputType()
            if self.svo:
                input_type.set_from_svo_file(self.svo)

            init_params = sl.InitParameters()
            init_params.coordinate_units = sl.UNIT.METER
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            init_params.camera_fps = self.zed_fps

            self.zed = sl.Camera()
            status = self.zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().fatal(f'Failed to open ZED camera: {status}')
                return

            pt_params = sl.PositionalTrackingParameters()
            # If user provided an area file path, instruct ZED to use it on enable.
            if self.area_map_path:
                pt_params.area_file_path = self.area_map_path
            pt_params.enable_area_memory = self.enable_area_memory
            pt_status = self.zed.enable_positional_tracking(pt_params)
            if pt_status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().warn(f'Positional tracking unavailable: {pt_status}')
            else:
                self.get_logger().info('ZED positional tracking enabled')

            zed_pose = sl.Pose()

            while rclpy.ok():
                if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
                    continue

                # Get pose in WORLD reference frame
                self.zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
                trans = zed_pose.get_translation(sl.Translation()).get()
                orient = zed_pose.get_orientation(sl.Orientation()).get()

                # orient is [qw, qx, qy, qz] in ZED Python API; map to geometry_msgs
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.frame_id

                odom.pose.pose.position.x = float(trans[0])
                odom.pose.pose.position.y = float(trans[1])
                odom.pose.pose.position.z = float(trans[2])

                # ZED orientation ordering may differ; assume (x,y,z,w) from API
                try:
                    # Try common ordering: qx,qy,qz,qw
                    qx = float(orient[0]); qy = float(orient[1]); qz = float(orient[2]); qw = float(orient[3])
                except Exception:
                    # fallback: qw,qx,qy,qz
                    qw = float(orient[0]); qx = float(orient[1]); qy = float(orient[2]); qz = float(orient[3])
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw

                # Apply configured frame translation offset
                try:
                    tx, ty, tz = self.frame_trans_offset
                    odom.pose.pose.position.x += float(tx)
                    odom.pose.pose.position.y += float(ty)
                    odom.pose.pose.position.z += float(tz)
                except Exception:
                    pass

                # Apply Z-rotation offset (degrees) to orientation
                try:
                    # Convert to (w,x,y,z) for math
                    q_rec = (odom.pose.pose.orientation.w,
                             odom.pose.pose.orientation.x,
                             odom.pose.pose.orientation.y,
                             odom.pose.pose.orientation.z)
                    theta = math.radians(float(self.frame_rot_z_deg))
                    qz = (math.cos(theta/2.0), 0.0, 0.0, math.sin(theta/2.0))

                    # Apply rotation offset before recorded orientation: q_final = qz * q_rec
                    qf = _quat_mul(qz, q_rec)
                    odom.pose.pose.orientation.w = qf[0]
                    odom.pose.pose.orientation.x = qf[1]
                    odom.pose.pose.orientation.y = qf[2]
                    odom.pose.pose.orientation.z = qf[3]
                except Exception:
                    pass

                # Publish odometry
                self.odom_pub.publish(odom)

                # Publish TF
                t = TransformStamped()
                t.header = odom.header
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.frame_id
                t.transform.translation.x = odom.pose.pose.position.x
                t.transform.translation.y = odom.pose.pose.position.y
                t.transform.translation.z = odom.pose.pose.position.z
                t.transform.rotation = odom.pose.pose.orientation
                try:
                    self.tf_broadcaster.sendTransform(t)
                except Exception:
                    pass

                # Append to path and publish
                ps = PoseStamped()
                ps.header = odom.header
                ps.pose = odom.pose.pose
                self.path.header.stamp = odom.header.stamp
                self.path.poses.append(ps)
                if len(self.path.poses) > 500:
                    self.path.poses.pop(0)
                self.path_pub.publish(self.path)

        finally:
            try:
                if self.zed is not None:
                    # Save area map on exit if requested
                    try:
                        if self.save_area_on_exit and self.area_map_path:
                            self.get_logger().info(f'Saving area map to {self.area_map_path} on shutdown')
                            status = self.zed.save_area_map(self.area_map_path)
                            if status <= sl.ERROR_CODE.SUCCESS:
                                export_state = _wait_area_export(
                                    self.zed,
                                    self.zed.get_area_export_state,
                                    sl.AREA_EXPORTING_STATE.RUNNING,
                                    sl.AREA_EXPORTING_STATE.SUCCESS)
                                if export_state == sl.AREA_EXPORTING_STATE.SUCCESS:
                                    self.get_logger().info(f'Area map saved: {self.area_map_path}')
                                else:
                                    self.get_logger().warn(f'Failed to save area map: {export_state}')
                            else:
                                self.get_logger().warn(f'Failed to save area map, status: {status}')
                    except Exception as e:
                        self.get_logger().warn(f'Exception during area save on exit: {e}')
                    try:
                        self.zed.disable_positional_tracking()
                    except Exception:
                        pass
                    self.zed.close()
            except Exception:
                pass

    def _handle_save_area(self, request, response):
        resp = TriggerSrv.Response()
        if not self.zed:
            resp.success = False
            resp.message = 'ZED not initialized'
            return resp
        if not self.area_map_path:
            resp.success = False
            resp.message = 'Parameter area_map_path not set'
            return resp
        try:
            self.get_logger().info(f'Saving area map to {self.area_map_path} (service)')
            status = self.zed.save_area_map(self.area_map_path)
            if status <= sl.ERROR_CODE.SUCCESS:
                export_state = _wait_area_export(
                    self.zed,
                    self.zed.get_area_export_state,
                    sl.AREA_EXPORTING_STATE.RUNNING,
                    sl.AREA_EXPORTING_STATE.SUCCESS)
                if export_state == sl.AREA_EXPORTING_STATE.SUCCESS:
                    resp.success = True
                    resp.message = f'Saved area map: {self.area_map_path}'
                else:
                    resp.success = False
                    resp.message = f'Failed to export area map: {export_state}'
            else:
                resp.success = False
                resp.message = f'save_area_map returned status {status}'
        except Exception as e:
            resp.success = False
            resp.message = f'Exception while saving area map: {e}'
        return resp

    def _handle_load_area(self, request, response):
        resp = TriggerSrv.Response()
        if not self.zed:
            resp.success = False
            resp.message = 'ZED not initialized'
            return resp
        if not self.area_map_path:
            resp.success = False
            resp.message = 'Parameter area_map_path not set'
            return resp
        try:
            # Re-enable positional tracking with area file path to relocalize
            try:
                self.zed.disable_positional_tracking()
            except Exception:
                pass
            pt_params = sl.PositionalTrackingParameters()
            pt_params.enable_area_memory = True
            pt_params.area_file_path = self.area_map_path
            status = self.zed.enable_positional_tracking(pt_params)
            if status == sl.ERROR_CODE.SUCCESS:
                resp.success = True
                resp.message = f'Loaded area map: {self.area_map_path}'
            else:
                resp.success = False
                resp.message = f'Failed to enable positional tracking with area map: {status}'
        except Exception as e:
            resp.success = False
            resp.message = f'Exception while loading area map: {e}'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = VSLAMZedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
