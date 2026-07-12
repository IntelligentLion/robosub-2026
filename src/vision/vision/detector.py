#!/usr/bin/env python3

import os
import sys
import gc
import numpy as np
import argparse
import cv2
import pyzed.sl as sl

from threading import Event, Lock, Thread
from time import sleep, monotonic
import threading

from vision.cv_viewer import tracking_viewer as cv_viewer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from auv_msgs.msg import ObjectDetection, ObjectDetectionArray


# ─────────────────────────────────────────────────────────────────────────────
#  ONNX detector (CPU fallback)
# ─────────────────────────────────────────────────────────────────────────────

class _OnnxBox:
    __slots__ = ('xywh', 'cls', 'conf')
    def __init__(self, xywh: np.ndarray, cls: np.ndarray, conf: np.ndarray):
        self.xywh = xywh
        self.cls  = cls
        self.conf = conf


class OnnxDetector:
    def __init__(self, onnx_path: str, class_names=None):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.log_severity_level = 3
        so.intra_op_num_threads = 2
        so.inter_op_num_threads = 1
        providers = []
        if 'CUDAExecutionProvider' in ort.get_available_providers():
            providers.append(('CUDAExecutionProvider', {
                'device_id': 0,
                'arena_extend_strategy': 'kSameAsRequested',
                'gpu_mem_limit': 512 * 1024 * 1024,
            }))
            print('[OnnxDetector] CUDA EP available — using GPU acceleration')
        providers.append('CPUExecutionProvider')
        self.session = ort.InferenceSession(
            onnx_path, sess_options=so,
            providers=providers,
        )

        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        shape = inp.shape
        self.input_h = int(shape[2]) if len(shape) >= 4 else 640
        self.input_w = int(shape[3]) if len(shape) >= 4 else 640

        out_shape = self.session.get_outputs()[0].shape
        nc = max(int(out_shape[1]) - 4, 1) if len(out_shape) == 3 else 1

        if class_names is not None:
            self.names = ({i: n for i, n in enumerate(class_names)}
                          if isinstance(class_names, list) else class_names)
        else:
            self.names = self._read_onnx_names(onnx_path, nc)

        self._blob = np.empty((1, 3, self.input_h, self.input_w), dtype=np.float32)
        print(f'[OnnxDetector] Loaded {onnx_path}')
        print(f'[OnnxDetector] Input: {self.input_h}x{self.input_w}  Classes: {list(self.names.values())}')

    @staticmethod
    def _read_onnx_names(onnx_path: str, nc: int) -> dict:
        try:
            import onnx, ast
            model_proto = onnx.load(onnx_path)
            for prop in model_proto.metadata_props:
                if prop.key == 'names':
                    names = ast.literal_eval(prop.value)
                    if isinstance(names, dict):
                        return {int(k): v for k, v in names.items()}
                    if isinstance(names, list):
                        return {i: n for i, n in enumerate(names)}
        except Exception:
            pass
        return {i: f'class_{i}' for i in range(nc)}

    def predict(self, img, imgsz=640, conf=0.4, iou=0.45, device='cuda'):
        orig_h, orig_w = img.shape[:2]
        self._preprocess(img)
        raw = self.session.run(None, {self.input_name: self._blob})[0]
        boxes, cls_ids, confs = _parse_yolo(raw, conf)
        if len(boxes) == 0:
            return [_EmptyResult()]

        inv_gain = 1.0 / self._gain
        boxes[:, 0] = (boxes[:, 0] - self._pad_x) * inv_gain
        boxes[:, 1] = (boxes[:, 1] - self._pad_y) * inv_gain
        boxes[:, 2] *= inv_gain
        boxes[:, 3] *= inv_gain

        keep = _nms(boxes, confs, iou)
        return [_OnnxResult([
            _OnnxBox(boxes[i:i+1], cls_ids[i:i+1].astype(np.float32), confs[i:i+1])
            for i in keep
        ])]

    def _preprocess(self, img: np.ndarray):
        canvas, self._gain, self._pad_x, self._pad_y = _letterbox(
            img, self.input_w, self.input_h)
        blob = canvas.astype(np.float32, copy=False)
        blob *= (1.0 / 255.0)
        np.copyto(self._blob, blob.transpose(2, 0, 1)[np.newaxis])


class _OnnxResult:
    __slots__ = ('_boxes',)
    def __init__(self, boxes):
        self._boxes = boxes
    @property
    def boxes(self):
        return self._boxes
    def cpu(self):
        return self
    def numpy(self):
        return self


class _EmptyResult:
    @property
    def boxes(self):
        return []
    def cpu(self):
        return self
    def numpy(self):
        return self


# ─────────────────────────────────────────────────────────────────────────────
#  TensorRT detector (GPU, FP16)
# ─────────────────────────────────────────────────────────────────────────────

def _cuda_check(ret):
    """Unwrap a cuda-python runtime return tuple, raising on error.

    cudart calls return ``(err,)`` or ``(err, value, ...)``. Returns the single
    value when present so callers can write ``ptr = _cuda_check(cudaMalloc(n))``.
    """
    if isinstance(ret, tuple):
        err, rest = ret[0], ret[1:]
    else:
        err, rest = ret, ()
    if int(err) != 0:
        raise RuntimeError(f'CUDA runtime error: {err}')
    if len(rest) == 1:
        return rest[0]
    if len(rest) > 1:
        return rest
    return None


class TensorRTDetector:
    def __init__(self, onnx_path: str, class_names=None):
        import tensorrt as trt
        from cuda.bindings import runtime as cudart

        engine_path = onnx_path.replace('.onnx', '.engine')
        logger = trt.Logger(trt.Logger.WARNING)

        if not os.path.exists(engine_path):
            print(f'[TensorRTDetector] Building FP16 engine from {os.path.basename(onnx_path)} '
                  f'(one-time, ~2-5 min)...')
            builder = trt.Builder(logger)
            network = builder.create_network(
                1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
            )
            parser = trt.OnnxParser(network, logger)
            with open(onnx_path, 'rb') as f:
                if not parser.parse(f.read()):
                    for i in range(parser.num_errors):
                        print(parser.get_error(i))
                    raise RuntimeError('TRT: failed to parse ONNX model')
            config = builder.create_builder_config()
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 26)
            if builder.platform_has_fast_fp16:
                config.set_flag(trt.BuilderFlag.FP16)
                print('[TensorRTDetector] FP16 enabled')
            serialized = builder.build_serialized_network(network, config)
            if serialized is None:
                raise RuntimeError('TRT: engine build failed (OOM?)')
            with open(engine_path, 'wb') as f:
                f.write(serialized)
            del builder, network, parser, config, serialized
            gc.collect()
            print(f'[TensorRTDetector] Engine saved: {engine_path}')

        runtime = trt.Runtime(logger)
        with open(engine_path, 'rb') as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.input_name  = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)
        in_shape  = self.engine.get_tensor_shape(self.input_name)
        out_shape = self.engine.get_tensor_shape(self.output_name)
        self.input_h = int(in_shape[2])
        self.input_w = int(in_shape[3])

        # Seg model (ffc_rs_26) has a 3rd tensor: output1, the mask-proto
        # (1,32,104,104). Unused by _parse_yolo, but TRT enqueueV3 refuses to
        # run unless every output tensor has an address set — allocate a
        # throwaway device buffer for it too.
        self._extra_out_names = [
            self.engine.get_tensor_name(i)
            for i in range(self.engine.num_io_tensors)
            if i not in (0, 1)
        ]
        self._extra_out_bufs = []

        # Pre-allocate persistent CUDA buffers via cuda-python (no torch — the
        # Jetson's torch wheel is CUDA-13 and can't init the 12.6 driver, so we
        # manage device memory natively against the system CUDA 12.6 / TRT 10.3).
        self._cudart = cudart
        in_count = int(in_shape[0]) * int(in_shape[1]) * self.input_h * self.input_w
        out_count = 1
        for d in out_shape:
            out_count *= int(d)
        self._inp_nbytes = in_count * 4          # float32
        self._out_nbytes = out_count * 4

        self._d_in = _cuda_check(cudart.cudaMalloc(self._inp_nbytes))
        self._d_out = _cuda_check(cudart.cudaMalloc(self._out_nbytes))
        self._stream = _cuda_check(cudart.cudaStreamCreate())
        self._stream_handle = int(self._stream)

        self.context.set_tensor_address(self.input_name,  int(self._d_in))
        self.context.set_tensor_address(self.output_name, int(self._d_out))

        for name in self._extra_out_names:
            shape = self.engine.get_tensor_shape(name)
            nbytes = 4
            for d in shape:
                nbytes *= int(d)
            buf = _cuda_check(cudart.cudaMalloc(nbytes))
            self.context.set_tensor_address(name, int(buf))
            self._extra_out_bufs.append(buf)

        # CPU-side pinned-friendly contiguous buffers for host↔device copies.
        self._blob = np.ascontiguousarray(
            np.empty((1, 3, self.input_h, self.input_w), dtype=np.float32))
        self._out_host = np.ascontiguousarray(
            np.empty(tuple(int(d) for d in out_shape), dtype=np.float32))

        nc = max(int(out_shape[1]) - 4, 1)
        if class_names is not None:
            self.names = ({i: n for i, n in enumerate(class_names)}
                          if isinstance(class_names, list) else class_names)
        else:
            self.names = OnnxDetector._read_onnx_names(onnx_path, nc)

        print(f'[TensorRTDetector] Ready  input:{self.input_h}x{self.input_w}'
              f'  classes:{list(self.names.values())}')

    def predict(self, img, imgsz=640, conf=0.4, iou=0.45, device='cuda'):
        cudart = self._cudart
        orig_h, orig_w = img.shape[:2]
        self._preprocess(img)

        # Host→device, run, device→host on a single stream, then sync.
        _cuda_check(cudart.cudaMemcpyAsync(
            int(self._d_in), self._blob.ctypes.data, self._inp_nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self._stream))
        self.context.execute_async_v3(self._stream_handle)
        _cuda_check(cudart.cudaMemcpyAsync(
            self._out_host.ctypes.data, int(self._d_out), self._out_nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self._stream))
        _cuda_check(cudart.cudaStreamSynchronize(self._stream))

        raw = self._out_host
        boxes, cls_ids, confs = _parse_yolo(raw, conf)
        if len(boxes) == 0:
            return [_EmptyResult()]

        inv_gain = 1.0 / self._gain
        boxes[:, 0] = (boxes[:, 0] - self._pad_x) * inv_gain
        boxes[:, 1] = (boxes[:, 1] - self._pad_y) * inv_gain
        boxes[:, 2] *= inv_gain
        boxes[:, 3] *= inv_gain

        keep = _nms(boxes, confs, iou)
        return [_OnnxResult([
            _OnnxBox(boxes[i:i+1], cls_ids[i:i+1].astype(np.float32), confs[i:i+1])
            for i in keep
        ])]

    def _preprocess(self, img: np.ndarray):
        canvas, self._gain, self._pad_x, self._pad_y = _letterbox(
            img, self.input_w, self.input_h)
        blob = canvas.astype(np.float32, copy=False)
        blob *= (1.0 / 255.0)
        np.copyto(self._blob, blob.transpose(2, 0, 1)[np.newaxis])

    def __del__(self):
        # Free device memory / stream best-effort on teardown.
        cudart = getattr(self, '_cudart', None)
        if cudart is None:
            return
        try:
            if getattr(self, '_d_in', None) is not None:
                cudart.cudaFree(self._d_in)
            if getattr(self, '_d_out', None) is not None:
                cudart.cudaFree(self._d_out)
            for buf in getattr(self, '_extra_out_bufs', []):
                cudart.cudaFree(buf)
            if getattr(self, '_stream', None) is not None:
                cudart.cudaStreamDestroy(self._stream)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
#  Shared parsing / NMS
# ─────────────────────────────────────────────────────────────────────────────

def _parse_yolo(raw: np.ndarray, conf_thres: float):
    if raw.ndim == 3 and raw.shape[2] <= raw.shape[1]:
        # Box-major end2end-NMS export (e.g. ffc_rs_26): (1, num_det, 6+mask) =
        # [x1, y1, x2, y2, conf, cls_id, mask_coeffs...] already NMS'd on-engine.
        data = raw[0]
        boxes_xyxy = data[:, :4]
        confidences = data[:, 4]
        class_ids = data[:, 5].astype(np.intp)
        mask = confidences >= conf_thres
        boxes_xyxy, class_ids, confidences = (
            boxes_xyxy[mask], class_ids[mask], confidences[mask])
        x1, y1, x2, y2 = (boxes_xyxy[:, 0], boxes_xyxy[:, 1],
                           boxes_xyxy[:, 2], boxes_xyxy[:, 3])
        boxes_xywh = np.stack(
            [(x1 + x2) * 0.5, (y1 + y2) * 0.5, x2 - x1, y2 - y1], axis=1)
        return boxes_xywh, class_ids, confidences

    if raw.ndim == 3:
        data = raw[0].T                      # channel-major: (4+nc[+32], anchors)
    elif raw.ndim == 2:
        data = raw
    else:
        return np.empty((0, 4)), np.empty(0, dtype=np.intp), np.empty(0)

    boxes_xywh = data[:, :4]
    scores = data[:, 4:]
    if scores.shape[1] == 1:
        confidences = scores[:, 0]
        class_ids = np.zeros(len(data), dtype=np.intp)
    else:
        class_ids = np.argmax(scores, axis=1)
        confidences = scores[np.arange(len(class_ids)), class_ids]

    mask = confidences >= conf_thres
    return boxes_xywh[mask], class_ids[mask], confidences[mask]


def _letterbox(img: np.ndarray, dst_w: int, dst_h: int):
    """Aspect-preserving resize + gray pad (ultralytics-style). A plain
    resize squashes 1280x720 → 416x416 (~1.8x horizontal distortion), which
    the model never saw in training. Returns (canvas, gain, pad_x, pad_y)
    so predictions can be mapped back to the original frame."""
    h, w = img.shape[:2]
    gain = min(dst_w / w, dst_h / h)
    sw, sh = int(round(w * gain)), int(round(h * gain))
    resized = cv2.resize(img, (sw, sh))
    canvas = np.full((dst_h, dst_w, img.shape[2]), 114, dtype=img.dtype)
    pad_x, pad_y = (dst_w - sw) // 2, (dst_h - sh) // 2
    canvas[pad_y:pad_y + sh, pad_x:pad_x + sw] = resized
    return canvas, gain, pad_x, pad_y


_NMS_MAX_DETECTIONS = 500


def _nms(boxes_xywh: np.ndarray, scores: np.ndarray, iou_thres: float):
    if len(boxes_xywh) == 0:
        return []
    cx, cy, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
    x1 = cx - w * 0.5; x2 = cx + w * 0.5
    y1 = cy - h * 0.5; y2 = cy + h * 0.5
    areas = np.maximum(w * h, 0)
    order = scores.argsort()[::-1]
    if len(order) > _NMS_MAX_DETECTIONS:
        order = order[:_NMS_MAX_DETECTIONS]
    keep = []
    for _ in range(len(order)):
        if len(order) == 0:
            break
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        rest = order[1:]
        ix1 = np.maximum(x1[i], x1[rest])
        iy1 = np.maximum(y1[i], y1[rest])
        ix2 = np.minimum(x2[i], x2[rest])
        iy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0, ix2 - ix1) * np.maximum(0, iy2 - iy1)
        iou = inter / (areas[i] + areas[rest] - inter + 1e-6)
        order = rest[iou <= iou_thres]
    return keep


# ─────────────────────────────────────────────────────────────────────────────
#  Shared model cache (avoids loading the model twice when both cameras run)
# ─────────────────────────────────────────────────────────────────────────────

class _ThreadSafeModel:
    """Wraps a detector so concurrent predict() calls are serialized."""
    def __init__(self, model):
        self._model = model
        self._lock = threading.Lock()

    @property
    def names(self):
        return self._model.names

    def predict(self, *args, **kwargs):
        with self._lock:
            return self._model.predict(*args, **kwargs)


_shared_model = None
_shared_model_lock = threading.Lock()


def _available_memory_mb():
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) // 1024
    except Exception:
        pass
    return 0


_MIN_MB_FOR_TRT_BUILD = 2048


def get_shared_model(onnx_path: str, class_names=None):
    """Return a process-wide singleton detector, creating it on first call."""
    global _shared_model
    with _shared_model_lock:
        if _shared_model is not None:
            print('[get_shared_model] Reusing existing model instance')
            return _shared_model

        model = None
        engine_path = onnx_path.replace('.onnx', '.engine')
        has_engine = os.path.exists(engine_path)
        avail_mb = _available_memory_mb()

        try:
            import tensorrt  # noqa: F401 — native TRT 10.3, no torch needed
            if has_engine:
                model = TensorRTDetector(onnx_path, class_names=class_names)
                print('[get_shared_model] Backend: TensorRT GPU (loaded pre-built engine)')
            elif avail_mb >= _MIN_MB_FOR_TRT_BUILD:
                print(f'[get_shared_model] {avail_mb} MB available — building TensorRT engine')
                model = TensorRTDetector(onnx_path, class_names=class_names)
                print('[get_shared_model] Backend: TensorRT GPU (FP16)')
            else:
                print(f'[get_shared_model] Only {avail_mb} MB available (need {_MIN_MB_FOR_TRT_BUILD} MB) '
                      f'— skipping TensorRT build, use build_engine.py offline')
        except Exception as trt_err:
            print(f'[get_shared_model] TensorRT failed ({trt_err}), trying ONNX')

        if model is None:
            model = OnnxDetector(onnx_path, class_names=class_names)
            active = model.session.get_providers()[0]
            print(f'[get_shared_model] Backend: ONNX ({active})')

        _shared_model = _ThreadSafeModel(model)
        return _shared_model


# ─────────────────────────────────────────────────────────────────────────────
#  ROS node
# ─────────────────────────────────────────────────────────────────────────────

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(ObjectDetectionArray, 'vision/detections', 10)
        self.sub_depth_pub_ = self.create_publisher(Float32, 'depth/sub_depth', 10)
        # Front-cam VIO pose → localization_node (which fuses to
        # localization/pose). Same topic/frame vslam_node uses, so the front
        # camera alone can feed the controller without a second ZED session.
        self.odom_pub_ = self.create_publisher(Odometry, 'vslam/odometry', 10)
        self._odom_frame = 'odom'
        self._child_frame = 'zed_left_camera_frame'

    def publish_detections(self, infos):
        msg = ObjectDetectionArray()
        for info in infos:
            detection = ObjectDetection()
            detection.label = str(info['label'])
            detection.confidence = float(info['confidence'])
            detection.position = Point(
                x=float(info['center_x']),
                y=float(info['center_y']),
                z=float(info.get('depth_m', -1.0)),
            )
            detection.bbox_width = float(info['bbox_width'])
            detection.bbox_height = float(info['bbox_height'])
            msg.detections.append(detection)
        self.publisher_.publish(msg)
        if msg.detections:
            labels = [d.label for d in msg.detections]
            self.get_logger().info(
                f'Published {len(msg.detections)} detections: {", ".join(labels)}'
            )

    def publish_sub_depth(self, depth_m: float):
        msg = Float32()
        msg.data = depth_m
        self.sub_depth_pub_.publish(msg)

    def publish_odometry(self, translation, orientation):
        """Publish the ZED WORLD pose as nav_msgs/Odometry on vslam/odometry.

        Mirrors vslam_node exactly: RIGHT_HANDED_Y_UP metres, ZED Python
        orientation ordering (qx, qy, qz, qw). localization_node subscribes
        this topic and fuses it to localization/pose.
        """
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = float(translation[0])
        odom.pose.pose.position.y = float(translation[1])
        odom.pose.pose.position.z = float(translation[2])
        odom.pose.pose.orientation.x = float(orientation[0])
        odom.pose.pose.orientation.y = float(orientation[1])
        odom.pose.pose.orientation.z = float(orientation[2])
        odom.pose.pose.orientation.w = float(orientation[3])
        self.odom_pub_.publish(odom)


# ─────────────────────────────────────────────────────────────────────────────
#  Globals shared between threads
# ─────────────────────────────────────────────────────────────────────────────

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
# Forward-facing camera model (RoboSub 2026). Deploy the matching .onnx
# (and optional .engine) via deploy_model.sh / convert_to_onnx.py.
_DEFAULT_ONNX = os.path.join(_PKG_DIR, 'ffc_rs_26.onnx')

lock = Lock()
frame_ready = Event()
inference_done = Event()
exit_signal = False
detections = []
detection_infos = []
model_names = {}
inference_device = 'gpu'


# ─────────────────────────────────────────────────────────────────────────────
#  Helper functions
# ─────────────────────────────────────────────────────────────────────────────

def xywh2abcd(xywh, im_shape):
    output = np.zeros((4, 2))
    img_h, img_w = im_shape[:2]
    x_min = max(0.0, xywh[0] - 0.5 * xywh[2])
    x_max = min(float(img_w - 1), xywh[0] + 0.5 * xywh[2])
    y_min = max(0.0, xywh[1] - 0.5 * xywh[3])
    y_max = min(float(img_h - 1), xywh[1] + 0.5 * xywh[3])
    output[0] = [x_min, y_min]
    output[1] = [x_max, y_min]
    output[2] = [x_max, y_max]
    output[3] = [x_min, y_max]
    return output


def detections_to_custom_box(det_list, im0, names):
    zed_boxes = []
    info_dicts = []
    img_h, img_w = im0.shape[:2]
    inv_w = 1.0 / img_w
    inv_h = 1.0 / img_h
    for det in det_list:
        xywh = det.xywh[0]
        cls_id = int(np.asarray(det.cls).flat[0])
        conf = float(np.asarray(det.conf).flat[0])

        obj = sl.CustomBoxObjectData()
        obj.bounding_box_2d = xywh2abcd(xywh, im0.shape)
        obj.label = cls_id
        obj.probability = conf
        obj.is_grounded = False
        zed_boxes.append(obj)

        info_dicts.append({
            'label':       names.get(cls_id, str(cls_id)),
            'confidence':  conf,
            'center_x':    float(xywh[0]) * inv_w,
            'center_y':    float(xywh[1]) * inv_h,
            'bbox_width':  float(xywh[2]) * inv_w,
            'bbox_height': float(xywh[3]) * inv_h,
            'depth_m':     -1.0,
        })
    return zed_boxes, info_dicts


def enrich_depths(info_dicts, zed_objects):
    for i, obj in enumerate(zed_objects.object_list):
        if i >= len(info_dicts):
            break
        pos = obj.position
        dist = float(np.sqrt(float(pos[0])**2 + float(pos[1])**2 + float(pos[2])**2))
        info_dicts[i]['depth_m'] = dist if dist > 0.01 else -1.0


# ─────────────────────────────────────────────────────────────────────────────
#  Inference thread
# ─────────────────────────────────────────────────────────────────────────────

_MAX_INFERENCE_ERRORS = 50


def inference_thread(weights, img_size, conf_thres=0.2, iou_thres=0.45, device='cuda', onnx_path=None, preloaded_model=None):
    global image_net, exit_signal, detections, detection_infos
    global model_names, inference_device

    consecutive_errors = 0

    try:
        if preloaded_model is not None:
            model = preloaded_model
            model_names = model.names
            use_onnx = True
        elif onnx_path:
            model = get_shared_model(onnx_path)
            model_names = model.names
            use_onnx = True
        else:
            import torch  # noqa: F401 — load torch first for a clean error if absent
            from ultralytics import YOLO
            model = YOLO(weights)
            model_names = model.names
            use_onnx = False
    except Exception as e:
        print(f'[inference_thread] FATAL: model load failed: {e}')
        exit_signal = True
        inference_done.set()
        return

    print(f'Model classes: {model_names}')
    inference_device = device

    while not exit_signal:
        if not frame_ready.wait(timeout=0.5):
            continue
        frame_ready.clear()
        if exit_signal:
            break

        try:
            with lock:
                # ZED get_data() is BGRA — model was trained on RGB, so the
                # red/blue swap here matters (RGBA2RGB kept the wrong order
                # and fed the model channel-swapped images).
                img = cv2.cvtColor(image_net, cv2.COLOR_BGRA2RGB)

            try:
                if use_onnx:
                    det = model.predict(img, imgsz=img_size, conf=conf_thres, iou=iou_thres)[0].boxes
                else:
                    # ultralytics assumes BGR for raw ndarrays and swaps
                    # internally — hand it BGR so its swap yields RGB.
                    det = model.predict(
                        img[..., ::-1], save=False, imgsz=img_size,
                        conf=conf_thres, iou=iou_thres,
                        device=inference_device,
                    )[0].cpu().numpy().boxes
            except RuntimeError as err:
                print(f'Inference error: {err}')
                if not use_onnx and inference_device != 'cpu':
                    print('Falling back to CPU inference...')
                    inference_device = 'cpu'
                    det = model.predict(
                        img, save=False, imgsz=img_size, conf=conf_thres,
                        iou=iou_thres, device='cpu',
                    )[0].cpu().numpy().boxes
                else:
                    det = []

            zed_boxes, infos = detections_to_custom_box(det, img, model_names)
            with lock:
                detections = zed_boxes
                detection_infos = infos
            consecutive_errors = 0
        except Exception as e:
            consecutive_errors += 1
            print(f'[inference_thread] Error ({consecutive_errors}/{_MAX_INFERENCE_ERRORS}): {e}')
            if consecutive_errors >= _MAX_INFERENCE_ERRORS:
                print('[inference_thread] Too many errors — stopping inference')
                exit_signal = True
        finally:
            inference_done.set()


# ─────────────────────────────────────────────────────────────────────────────
#  Main camera + ZED loop
# ─────────────────────────────────────────────────────────────────────────────

def _interruptible_sleep(seconds):
    """Sleep in 0.1 s slices, bailing early if exit_signal is set."""
    for _ in range(max(1, int(seconds / 0.1))):
        if exit_signal:
            return
        sleep(0.1)


def _open_zed(zed, input_type, init_candidates):
    """One pass over the resolution/depth/fps candidates.

    Returns the InitParameters that opened, or None if none did. Always frees a
    failed open (close + gc) so a retry starts clean.
    """
    for resolution, depth_mode, fps in init_candidates:
        candidate = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        candidate.coordinate_units = sl.UNIT.METER
        candidate.depth_mode = depth_mode
        candidate.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        candidate.depth_minimum_distance = 0.3
        candidate.depth_maximum_distance = 20
        candidate.camera_resolution = resolution
        candidate.camera_fps = fps
        candidate.sdk_verbose = 0
        if zed.open(candidate) == sl.ERROR_CODE.SUCCESS:
            print(f'Initialized Camera: resolution={resolution}, '
                  f'depth={depth_mode}, fps={fps}')
            return candidate
        print(f'ZED open failed ({resolution}, {depth_mode}, {fps})')
        # Explicit close to free leaked CUDA memory from a failed open.
        zed.close()
        gc.collect()
    return None


def _open_zed_with_retry(zed, input_type, init_candidates, *, backoff_s=3.0):
    """Retry the whole candidate list with backoff until it opens or exit.

    A camera that is briefly busy (a previous run still releasing it after an
    SSH drop) or momentarily unplugged recovers on its own — just rerun, no
    power-cycle. Returns InitParameters, or None if exit_signal was set first.
    """
    attempt = 0
    while not exit_signal:
        attempt += 1
        init_params = _open_zed(zed, input_type, init_candidates)
        if init_params is not None:
            return init_params
        print(f'\n[Vision] camera open failed (attempt {attempt}) — '
              f'retrying in {backoff_s:.0f}s.')
        print('  If a previous run still holds it: fuser -k /dev/video0 /dev/video1')
        print('  Otherwise replug the ZED into a USB 3.0 port. (Ctrl+C to abort.)')
        _interruptible_sleep(backoff_s)
    return None


def _enable_zed_features(zed, init_params):
    """Enable positional tracking + custom-box object detection.

    Returns (has_depth, positional_tracking_enabled, object_detection_enabled,
    obj_param). Safe to call again after a reopen.
    """
    has_depth = init_params.depth_mode != sl.DEPTH_MODE.NONE
    positional_tracking_enabled = False
    object_detection_enabled = False

    if has_depth:
        pt_status = zed.enable_positional_tracking(
            sl.PositionalTrackingParameters())
        if pt_status == sl.ERROR_CODE.SUCCESS:
            positional_tracking_enabled = True
            print('ZED positional tracking enabled')
        else:
            print(f'Positional tracking not available: {pt_status}')
    else:
        print('Skipping positional tracking (no depth mode)')

    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
    obj_param.enable_tracking = True
    obj_param.enable_segmentation = False
    if has_depth:
        od_status = zed.enable_object_detection(obj_param)
        if od_status == sl.ERROR_CODE.SUCCESS:
            object_detection_enabled = True
        else:
            print(f'ZED object detection unavailable: {od_status} — 2D-only')
    else:
        print('Running in 2D-only mode (no depth — detections lack distance)')

    return (has_depth, positional_tracking_enabled,
            object_detection_enabled, obj_param)


def run_detector(node):
    global image_net, exit_signal, detections, detection_infos

    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default=_DEFAULT_ONNX)
    parser.add_argument('--onnx', type=str, default=_DEFAULT_ONNX)
    parser.add_argument('--classes', type=str, default=None)
    parser.add_argument('--svo', type=str, default=None)
    parser.add_argument('--img_size', type=int, default=416)
    parser.add_argument('--conf_thres', type=float, default=0.4)
    parser.add_argument('--device', type=str, default='cuda')
    parser.add_argument('--zed_fps', type=int, default=60)
    parser.add_argument('--view', action='store_true',
                        help='Show the annotated live OpenCV window. OFF by default — '
                             'on the headless sub the per-frame full-res retrieve + render '
                             '+ GUI pump is wasted work on the Jetson.')
    parser.add_argument('--save_frames', type=str, default=None,
                        help='Directory to dump one camera frame per second '
                             '(JPEG, with detection boxes drawn). Headless '
                             'alternative to --view for diagnosing what the '
                             'model actually sees.')
    # Strip ROS args (e.g. `--ros-args -r __node:=vision_node` injected by
    # launch) so argparse only sees the detector's own flags.
    from rclpy.utilities import remove_ros_args
    opt = parser.parse_args(args=remove_ros_args(sys.argv)[1:])

    if opt.save_frames:
        os.makedirs(opt.save_frames, exist_ok=True)
        print(f'[Vision] saving 1 annotated frame/s to {opt.save_frames}')
    last_frame_save = 0.0

    onnx_path = opt.onnx
    class_names = None
    if onnx_path and opt.classes:
        class_names = [c.strip() for c in opt.classes.split(',') if c.strip()]

    model_label = (onnx_path or opt.weights).split('/')[-1]

    # Load model BEFORE opening ZED camera — TensorRT engine build needs
    # the full GPU memory pool, which ZED would otherwise consume.
    print('Initializing Model (before camera to avoid GPU OOM)...')
    if onnx_path:
        preloaded = get_shared_model(onnx_path, class_names=class_names)
    else:
        preloaded = None
    gc.collect()

    capture_thread = Thread(
        target=inference_thread,
        kwargs={
            'weights':    opt.weights,
            'img_size':   opt.img_size,
            'conf_thres': opt.conf_thres,
            'iou_thres':  0.45,
            'device':     opt.device,
            'onnx_path':  onnx_path,
            'preloaded_model': preloaded,
        },
        daemon=True,
    )
    capture_thread.start()

    print('Initializing Camera...')
    zed = sl.Camera()
    image_left_tmp = sl.Mat()
    image_left = sl.Mat()
    object_detection_enabled = False
    positional_tracking_enabled = False

    input_type = sl.InputType()
    if opt.svo:
        input_type.set_from_svo_file(opt.svo)

    # Prioritise PERFORMANCE — NEURAL/ULTRA consistently OOM on Jetson Orin 8GB.
    # Keep them as lower-priority options for machines with more VRAM.
    init_candidates = [
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.PERFORMANCE,  opt.zed_fps),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.PERFORMANCE,  opt.zed_fps),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.PERFORMANCE,  30),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.PERFORMANCE,  30),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.NEURAL,       opt.zed_fps),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.ULTRA,        opt.zed_fps),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.NONE,         opt.zed_fps),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.NONE,         opt.zed_fps),
    ]

    runtime_params = sl.RuntimeParameters()
    runtime_params.enable_fill_mode = False

    # Robust open: keep retrying instead of giving up after one pass, so a
    # camera that is briefly busy or unplugged recovers without a power-cycle.
    init_params = _open_zed_with_retry(zed, input_type, init_candidates)
    if init_params is None:                      # exit requested while waiting
        capture_thread.join(timeout=2.0)
        return

    try:
        (has_depth, positional_tracking_enabled,
         object_detection_enabled, obj_param) = _enable_zed_features(
            zed, init_params)

        objects = sl.Objects()
        obj_runtime_param = sl.CustomObjectDetectionRuntimeParameters()
        zed_pose = sl.Pose()

        camera_infos = zed.get_camera_information()
        camera_res = camera_infos.camera_configuration.resolution
        display_resolution = sl.Resolution(
            min(camera_res.width, 960), min(camera_res.height, 540)
        )
        image_scale = [
            display_resolution.width  / camera_res.width,
            display_resolution.height / camera_res.height,
        ]

        mode_label = '3D' if object_detection_enabled else '2D-only'
        hud_line1 = f'MODEL: {model_label} conf>={opt.conf_thres:.2f}'
        hud_line3 = f'ZED: {camera_res.width}x{camera_res.height}@{camera_infos.camera_configuration.fps}'

        consecutive_grab_failures = 0
        _GRAB_FAILS_BEFORE_REOPEN = 200

        while not exit_signal:
            grab_status = zed.grab(runtime_params)
            if grab_status != sl.ERROR_CODE.SUCCESS:
                consecutive_grab_failures += 1
                if consecutive_grab_failures >= _GRAB_FAILS_BEFORE_REOPEN:
                    # Camera likely disconnected mid-run. Tear down and reopen
                    # (with retry) instead of exiting, so the stream recovers on
                    # reconnect — no need to restart the script or the AUV.
                    print(f'[Vision] {consecutive_grab_failures} grab failures '
                          f'({grab_status}) — camera lost. Recovering (reopen)…')
                    try:
                        if object_detection_enabled:
                            zed.disable_object_detection()
                        if positional_tracking_enabled:
                            zed.disable_positional_tracking()
                    except Exception:
                        pass
                    try:
                        zed.close()
                    except Exception:
                        pass
                    gc.collect()
                    init_params = _open_zed_with_retry(
                        zed, input_type, init_candidates)
                    if init_params is None:      # exit requested during recovery
                        break
                    (has_depth, positional_tracking_enabled,
                     object_detection_enabled, obj_param) = _enable_zed_features(
                        zed, init_params)
                    print('[Vision] camera recovered — resuming stream.')
                    consecutive_grab_failures = 0
                    continue
                sleep(0.005)
                continue
            consecutive_grab_failures = 0

            # Grab frame for inference
            with lock:
                zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
                image_net = image_left_tmp.get_data()

            inference_done.clear()
            frame_ready.set()

            # Wait for inference thread
            inference_done.wait(timeout=1.0)

            if object_detection_enabled:
                with lock:
                    zed.ingest_custom_box_objects(detections)
                zed.retrieve_custom_objects(objects, obj_runtime_param)
                with lock:
                    enrich_depths(detection_infos, objects)
                    local_infos = list(detection_infos)
            else:
                with lock:
                    local_infos = list(detection_infos)

            try:
                if positional_tracking_enabled:
                    zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
                    translation = zed_pose.get_translation(sl.Translation()).get()
                    sub_depth_m = -float(translation[1])
                    node.publish_sub_depth(sub_depth_m)
                    # Same pose, published as full 6-DOF odometry so the front
                    # camera feeds localization/pose (no separate vslam_node /
                    # 2nd ZED session needed).
                    orientation = zed_pose.get_orientation(sl.Orientation()).get()
                    node.publish_odometry(translation, orientation)

                node.publish_detections(local_infos)
            except Exception:
                # Publisher handles get destroyed while this thread is still
                # mid-loop during shutdown (InvalidHandle) — exit quietly
                # instead of spamming a traceback.
                if exit_signal:
                    break
                raise

            if opt.save_frames:
                now = monotonic()
                if now - last_frame_save >= 1.0:
                    last_frame_save = now
                    with lock:
                        frame = cv2.cvtColor(image_net, cv2.COLOR_BGRA2BGR)
                    fh, fw = frame.shape[:2]
                    for info in local_infos:
                        w = info['bbox_width'] * fw
                        h = info['bbox_height'] * fh
                        x0 = int(info['center_x'] * fw - w / 2)
                        y0 = int(info['center_y'] * fh - h / 2)
                        cv2.rectangle(frame, (x0, y0),
                                      (int(x0 + w), int(y0 + h)),
                                      (0, 255, 0), 2)
                        cv2.putText(frame,
                                    f"{info['label']} {info['confidence']:.2f}",
                                    (x0, max(y0 - 6, 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 255, 0), 2)
                    cv2.imwrite(f'{opt.save_frames}/frame_{now:.0f}.jpg',
                                frame)

            # Optional live view — OFF by default. On the headless sub this
            # second full-resolution retrieve_image + render_2D + GUI pump runs
            # every frame for nothing; skipping it frees CPU/USB bandwidth on
            # the Orin Nano. Enable with --view when debugging on a desk.
            if opt.view:
                zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
                image_left_ocv = image_left.get_data()
                if object_detection_enabled:
                    cv_viewer.render_2D(image_left_ocv, image_scale, objects, obj_param.enable_tracking)

                cv2.putText(image_left_ocv, hud_line1,
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0, 255), 2)
                cv2.putText(image_left_ocv, f'DEVICE: {inference_device} img:{opt.img_size} [{mode_label}]',
                            (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255, 255), 2)
                cv2.putText(image_left_ocv, hud_line3,
                            (10, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0, 255), 2)
                cv2.imshow('ZED | Live View', image_left_ocv)

                key = cv2.waitKey(1)
                if key in [27, ord('q'), ord('Q')]:
                    exit_signal = True
    finally:
        exit_signal = True
        frame_ready.set()
        capture_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
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
    global exit_signal
    rclpy.init(args=None)
    vision_node = VisionNode()

    # Release the camera cleanly on terminate / hangup / interrupt. SIGHUP is
    # the key one: when an SSH session drops, the foreground process gets
    # SIGHUP — without this handler it dies WITHOUT closing the ZED, leaving the
    # camera locked so the next run fails ("camera stream failed") until a power
    # cycle. Setting exit_signal lets run_detector's finally close the camera.
    import signal as _signal

    def _request_shutdown(signum, _frame):
        global exit_signal
        print(f'\n[Vision] signal {signum} — releasing camera and exiting.')
        exit_signal = True

    for _sig in (_signal.SIGINT, _signal.SIGTERM, _signal.SIGHUP):
        try:
            _signal.signal(_sig, _request_shutdown)
        except (ValueError, OSError, AttributeError):
            pass        # not in main thread / platform lacks the signal

    try:
        run_detector(vision_node)
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
