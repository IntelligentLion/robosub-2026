#!/usr/bin/env python3
"""Bottom-facing ZED 2i camera node.

Runs ZED positional tracking (VIO) for odometry and detects path markers
on the pool floor. Publishes:
  - odom/bottom        (nav_msgs/Odometry)       – 6-DOF pose from ZED VIO
  - vision/path_markers (auv_msgs/ObjectDetectionArray) – detected floor markers
  - depth/sub_depth    (std_msgs/Float32)         – sub depth from VIO translation
"""

import os
import gc
import argparse
import sys

import numpy as np
import cv2
import pyzed.sl as sl

from threading import Event, Lock, Thread
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, Twist
from std_msgs.msg import Float32, Header
from nav_msgs.msg import Odometry
from auv_msgs.msg import ObjectDetection, ObjectDetectionArray

from vision.detector import OnnxDetector, TensorRTDetector
from vision.detector import _parse_yolo, _nms, _OnnxResult, _OnnxBox, _EmptyResult
from vision.detector import xywh2abcd, detections_to_custom_box


_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_ONNX = os.path.join(_PKG_DIR, 'zed_right_24_06_15.onnx')


lock = Lock()
frame_ready = Event()
inference_done = Event()
exit_signal = False
detections = []
detection_infos = []
model_names = {}


class BottomCameraNode(Node):
    def __init__(self):
        super().__init__('bottom_camera_node')

        self.odom_pub = self.create_publisher(Odometry, 'odom/bottom', 10)
        self.marker_pub = self.create_publisher(
            ObjectDetectionArray, 'vision/path_markers', 10)
        self.depth_pub = self.create_publisher(Float32, 'depth/sub_depth', 10)

    def publish_odometry(self, translation, orientation, velocity):
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(translation[0])
        msg.pose.pose.position.y = float(translation[1])
        msg.pose.pose.position.z = float(translation[2])
        msg.pose.pose.orientation.x = float(orientation[0])
        msg.pose.pose.orientation.y = float(orientation[1])
        msg.pose.pose.orientation.z = float(orientation[2])
        msg.pose.pose.orientation.w = float(orientation[3])

        msg.twist.twist.linear.x = float(velocity[0])
        msg.twist.twist.linear.y = float(velocity[1])
        msg.twist.twist.linear.z = float(velocity[2])

        self.odom_pub.publish(msg)

    def publish_markers(self, infos):
        msg = ObjectDetectionArray()
        for info in infos:
            det = ObjectDetection()
            det.label = str(info['label'])
            det.confidence = float(info['confidence'])
            det.position = Point(
                x=float(info['center_x']),
                y=float(info['center_y']),
                z=float(info.get('depth_m', -1.0)),
            )
            det.bbox_width = float(info['bbox_width'])
            det.bbox_height = float(info['bbox_height'])
            msg.detections.append(det)
        self.marker_pub.publish(msg)
        if msg.detections:
            labels = [d.label for d in msg.detections]
            self.get_logger().info(
                f'Path markers: {len(msg.detections)} — {", ".join(labels)}')

    def publish_depth(self, depth_m: float):
        msg = Float32()
        msg.data = depth_m
        self.depth_pub.publish(msg)


def _inference_thread(weights, img_size, conf_thres, iou_thres, device, onnx_path):
    global exit_signal, detections, detection_infos, model_names

    print('[BottomCam] Initializing inference...')

    model = None
    if onnx_path:
        try:
            import tensorrt, torch
            model = TensorRTDetector(onnx_path)
            print('[BottomCam] Backend: TensorRT GPU (FP16)')
        except Exception as e:
            print(f'[BottomCam] TensorRT unavailable ({e}), using ONNX/CPU')
        if model is None:
            model = OnnxDetector(onnx_path)
            print('[BottomCam] Backend: ONNX (CPU)')
        model_names = model.names
        use_onnx = True
    else:
        from ultralytics import YOLO
        model = YOLO(weights)
        model_names = model.names
        use_onnx = False

    print(f'[BottomCam] Classes: {model_names}')

    while not exit_signal:
        if not frame_ready.wait(timeout=0.1):
            continue
        frame_ready.clear()
        if exit_signal:
            break

        try:
            with lock:
                img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)

            if use_onnx:
                det = model.predict(
                    img, imgsz=img_size, conf=conf_thres, iou=iou_thres)[0].boxes
            else:
                det = model.predict(
                    img, save=False, imgsz=img_size, conf=conf_thres,
                    iou=iou_thres, device=device)[0].cpu().numpy().boxes

            zed_boxes, infos = detections_to_custom_box(det, img, model_names)
            with lock:
                detections = zed_boxes
                detection_infos = infos
        except Exception as e:
            print(f'[BottomCam] Inference error: {e}')
        finally:
            inference_done.set()


def run_bottom_camera(node: BottomCameraNode):
    global image_net, exit_signal, detections, detection_infos

    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default=_DEFAULT_ONNX)
    parser.add_argument('--onnx', type=str, default=_DEFAULT_ONNX)
    parser.add_argument('--serial', type=int, default=0,
                        help='ZED serial number (0 = auto-detect second camera)')
    parser.add_argument('--img_size', type=int, default=416)
    parser.add_argument('--conf_thres', type=float, default=0.4)
    parser.add_argument('--device', type=str, default='cuda')
    parser.add_argument('--zed_fps', type=int, default=30)
    opt, _ = parser.parse_known_args(sys.argv[1:])

    inf_thread = Thread(
        target=_inference_thread,
        kwargs={
            'weights': opt.weights,
            'img_size': opt.img_size,
            'conf_thres': opt.conf_thres,
            'iou_thres': 0.45,
            'device': opt.device,
            'onnx_path': opt.onnx,
        },
        daemon=True,
    )
    inf_thread.start()

    print('[BottomCam] Initializing ZED camera...')
    zed = sl.Camera()
    image_tmp = sl.Mat()
    positional_tracking_enabled = False
    object_detection_enabled = False

    init_candidates = [
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.PERFORMANCE, opt.zed_fps),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.PERFORMANCE, opt.zed_fps),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.PERFORMANCE, 15),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.NONE,        opt.zed_fps),
    ]

    runtime_params = sl.RuntimeParameters()
    runtime_params.enable_fill_mode = False
    status = sl.ERROR_CODE.FAILURE
    init_params = None

    for resolution, depth_mode, fps in init_candidates:
        candidate = sl.InitParameters(svo_real_time_mode=True)
        candidate.coordinate_units = sl.UNIT.METER
        candidate.depth_mode = depth_mode
        candidate.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        candidate.depth_minimum_distance = 0.15
        candidate.depth_maximum_distance = 5.0
        candidate.camera_resolution = resolution
        candidate.camera_fps = fps
        candidate.sdk_verbose = 0
        if opt.serial > 0:
            candidate.set_from_serial_number(opt.serial)

        status = zed.open(candidate)
        if status == sl.ERROR_CODE.SUCCESS:
            init_params = candidate
            print(f'[BottomCam] ZED opened: {resolution}, {depth_mode}, {fps} fps')
            break
        print(f'[BottomCam] ZED open failed ({resolution}, {depth_mode}, {fps}): {status}')
        zed.close()
        gc.collect()

    if status != sl.ERROR_CODE.SUCCESS:
        print(f'[BottomCam] All camera configs failed: {status}')
        exit_signal = True
        inf_thread.join(timeout=2.0)
        return

    has_depth = init_params.depth_mode != sl.DEPTH_MODE.NONE

    zed_pose = sl.Pose()
    zed_sensors = sl.SensorsData()

    try:
        # Positional tracking — the core of VIO odometry
        if has_depth:
            pt_params = sl.PositionalTrackingParameters()
            pt_params.enable_area_memory = True
            pt_params.enable_imu_fusion = True
            pt_status = zed.enable_positional_tracking(pt_params)
            if pt_status == sl.ERROR_CODE.SUCCESS:
                positional_tracking_enabled = True
                print('[BottomCam] Positional tracking (VIO) enabled')
            else:
                print(f'[BottomCam] Positional tracking failed: {pt_status}')
        else:
            print('[BottomCam] No depth — positional tracking unavailable')

        # Object detection for path marker ingestion
        if has_depth:
            obj_param = sl.ObjectDetectionParameters()
            obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
            obj_param.enable_tracking = True
            obj_param.enable_segmentation = False
            od_status = zed.enable_object_detection(obj_param)
            if od_status == sl.ERROR_CODE.SUCCESS:
                object_detection_enabled = True
            else:
                print(f'[BottomCam] Object detection unavailable: {od_status}')

        objects = sl.Objects()
        obj_runtime = sl.CustomObjectDetectionRuntimeParameters()

        print('[BottomCam] Starting main loop...')

        while not exit_signal:
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                sleep(0.005)
                continue

            # Grab frame for inference
            with lock:
                zed.retrieve_image(image_tmp, sl.VIEW.LEFT)
                image_net = image_tmp.get_data()

            inference_done.clear()
            frame_ready.set()
            inference_done.wait(timeout=1.0)

            # Ingest detections into ZED tracker for 3D positions
            if object_detection_enabled:
                with lock:
                    zed.ingest_custom_box_objects(detections)
                zed.retrieve_custom_objects(objects, obj_runtime)

                # Enrich with depth
                with lock:
                    for i, obj in enumerate(objects.object_list):
                        if i >= len(detection_infos):
                            break
                        pos = obj.position
                        dist = float(np.sqrt(
                            float(pos[0])**2 + float(pos[1])**2 + float(pos[2])**2))
                        detection_infos[i]['depth_m'] = dist if dist > 0.01 else -1.0
                    local_infos = list(detection_infos)
            else:
                with lock:
                    local_infos = list(detection_infos)

            # Publish path marker detections
            node.publish_markers(local_infos)

            # Publish VIO odometry
            if positional_tracking_enabled:
                tracking_state = zed.get_position(
                    zed_pose, sl.REFERENCE_FRAME.WORLD)

                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    t = zed_pose.get_translation(sl.Translation()).get()
                    o = zed_pose.get_orientation(sl.Orientation()).get()

                    # Velocity from IMU
                    zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.CURRENT)
                    imu = zed_sensors.get_imu_data()
                    ang_vel = imu.get_angular_velocity()

                    # ZED RIGHT_HANDED_Y_UP: x=right, y=up, z=backward
                    # Depth below surface = -y
                    sub_depth_m = -float(t[1])
                    node.publish_depth(sub_depth_m)

                    velocity = [0.0, 0.0, 0.0]
                    try:
                        twist = zed_pose.get_twist()
                        velocity = [float(twist[0]), float(twist[1]), float(twist[2])]
                    except Exception:
                        pass

                    node.publish_odometry(
                        translation=[float(t[0]), float(t[1]), float(t[2])],
                        orientation=[float(o[0]), float(o[1]), float(o[2]), float(o[3])],
                        velocity=velocity,
                    )

    finally:
        exit_signal = True
        frame_ready.set()
        inf_thread.join(timeout=2.0)
        if object_detection_enabled:
            try:
                zed.disable_object_detection()
            except Exception:
                pass
        if positional_tracking_enabled:
            try:
                zed.disable_positional_tracking()
            except Exception:
                pass
        try:
            zed.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = BottomCameraNode()
    try:
        run_bottom_camera(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
