#!/usr/bin/env python3

import os
import sys
import numpy as np
import argparse
import cv2
import pyzed.sl as sl

from threading import Lock, Thread
from time import sleep

from vision.ogl_viewer import viewer as gl
from vision.cv_viewer import tracking_viewer as cv_viewer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from auv_msgs.msg import ObjectDetection, ObjectDetectionArray


# ─────────────────────────────────────────────────────────────────────────────
#  ONNX placeholder detector
#  Accepts any YOLOv8-style ONNX model (output: (1, 4+nc, na) or (1, na, 5+nc))
#  and exposes the same interface used by the Ultralytics YOLO path.
# ─────────────────────────────────────────────────────────────────────────────

class _OnnxBox:
    """Mimics one Ultralytics Boxes entry."""
    def __init__(self, xywh: np.ndarray, cls: np.ndarray, conf: np.ndarray):
        self.xywh = xywh   # shape (1, 4)
        self.cls  = cls    # shape (1,)
        self.conf = conf   # shape (1,)


class OnnxDetector:
    """Placeholder ONNX multi-class detector with Ultralytics-compatible interface."""

    def __init__(self, onnx_path: str, class_names=None):
        import onnxruntime as ort
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        
        so = ort.SessionOptions()
        so.log_severity_level = 3  # ORT logging: 0=Verbose, 1=Info, 2=Warning, 3=Error, 4=Fatal
        self.session = ort.InferenceSession(onnx_path, sess_options=so, providers=providers)
        
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        shape = inp.shape
        self.input_h = int(shape[2]) if len(shape) >= 4 else 640
        self.input_w = int(shape[3]) if len(shape) >= 4 else 640

        # Derive class count from the output shape
        out_shape = self.session.get_outputs()[0].shape
        if len(out_shape) == 3:
            # (1, 4+nc, na) – YOLOv8 style
            nc = int(out_shape[1]) - 4
        else:
            nc = 1
        nc = max(nc, 1)

        if class_names is not None:
            # Caller-supplied names take priority
            if isinstance(class_names, list):
                self.names = {i: n for i, n in enumerate(class_names)}
            else:
                self.names = class_names
        else:
            # Try to read names from Ultralytics-embedded ONNX metadata
            self.names = self._read_onnx_names(onnx_path, nc)

        print(f'[OnnxDetector] Loaded {onnx_path}')
        print(f'[OnnxDetector] Input: {self.input_h}x{self.input_w}  Classes: {list(self.names.values())}')

    @staticmethod
    def _read_onnx_names(onnx_path: str, nc: int) -> dict:
        """Read class names from Ultralytics ONNX metadata_props, fall back to class_N."""
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

    # ------------------------------------------------------------------
    def predict(self, img, imgsz=640, conf=0.4, iou=0.45, device='cuda'):
        """Return [[_OnnxBox, ...]] – same nesting as Ultralytics."""
        orig_h, orig_w = img.shape[:2]
        blob = self._preprocess(img)
        raw = self.session.run(None, {self.input_name: blob})[0]
        boxes, cls_ids, confs = self._parse(raw, conf)
        if len(boxes) == 0:
            return [_EmptyResult()]

        # Scale back to original image pixels
        sx = orig_w / self.input_w
        sy = orig_h / self.input_h
        boxes[:, 0] *= sx
        boxes[:, 1] *= sy
        boxes[:, 2] *= sx
        boxes[:, 3] *= sy

        keep = _nms(boxes, confs, iou)
        result = [
            _OnnxBox(
                xywh=boxes[i:i+1],
                cls=cls_ids[i:i+1].astype(np.float32),
                conf=confs[i:i+1],
            )
            for i in keep
        ]
        return [_OnnxResult(result)]

    # ------------------------------------------------------------------
    def _preprocess(self, img: np.ndarray) -> np.ndarray:
        resized = cv2.resize(img, (self.input_w, self.input_h))
        blob = resized.astype(np.float32) / 255.0
        return blob.transpose(2, 0, 1)[np.newaxis]  # BCHW

    # ------------------------------------------------------------------
    @staticmethod
    def _parse(raw: np.ndarray, conf_thres: float):
        """Parse raw ONNX output into (boxes_xywh, class_ids, confidences)."""
        if raw.ndim == 3:
            # YOLOv8: (1, 4+nc, na) → (na, 4+nc)
            data = raw[0].T
        elif raw.ndim == 2:
            data = raw
        else:
            return np.empty((0, 4)), np.empty(0, int), np.empty(0)

        boxes_xywh = data[:, :4]
        scores = data[:, 4:]
        if scores.shape[1] == 1:
            # Binary / objectness-only output
            confidences = scores[:, 0]
            class_ids = np.zeros(len(data), dtype=int)
        else:
            class_ids = np.argmax(scores, axis=1)
            confidences = scores[np.arange(len(class_ids)), class_ids]

        mask = confidences >= conf_thres
        return boxes_xywh[mask], class_ids[mask], confidences[mask]


class _OnnxResult:
    @property
    def boxes(self):
        return self._boxes

    def __init__(self, boxes):
        self._boxes = boxes

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
#  TensorRT detector  (JetPack 6 — cuDNN 9 / TRT 10 native, no onnxruntime CUDA)
# ─────────────────────────────────────────────────────────────────────────────

class TensorRTDetector:
    """GPU inference via TensorRT 10 — bypasses onnxruntime's cuDNN 8 dependency.

    Builds a .engine file from the ONNX model on first run (~2-5 min) and
    caches it alongside the .onnx file for fast subsequent startups.
    """

    def __init__(self, onnx_path: str, class_names=None):
        import tensorrt as trt
        import torch

        engine_path = onnx_path.replace('.onnx', '.engine')
        logger = trt.Logger(trt.Logger.WARNING)

        if not os.path.exists(engine_path):
            print(f'[TensorRTDetector] Building engine from {os.path.basename(onnx_path)} '
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
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)
            serialized = builder.build_serialized_network(network, config)
            with open(engine_path, 'wb') as f:
                f.write(serialized)
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

        nc = max(int(out_shape[1]) - 4, 1)
        if class_names is not None:
            self.names = ({i: n for i, n in enumerate(class_names)}
                          if isinstance(class_names, list) else class_names)
        else:
            self.names = OnnxDetector._read_onnx_names(onnx_path, nc)

        print(f'[TensorRTDetector] Ready  input:{self.input_h}x{self.input_w}'
              f'  classes:{list(self.names.values())}')

    def predict(self, img, imgsz=640, conf=0.4, iou=0.45, device='cuda'):
        import torch

        orig_h, orig_w = img.shape[:2]
        blob = self._preprocess(img)

        inp = torch.from_numpy(blob).cuda()
        out_shape = self.engine.get_tensor_shape(self.output_name)
        out = torch.zeros((int(out_shape[0]), int(out_shape[1]), int(out_shape[2])),
                          dtype=torch.float32, device='cuda')

        self.context.set_tensor_address(self.input_name,  inp.data_ptr())
        self.context.set_tensor_address(self.output_name, out.data_ptr())
        self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        torch.cuda.current_stream().synchronize()

        raw = out.cpu().numpy()
        boxes, cls_ids, confs = OnnxDetector._parse(raw, conf)
        if len(boxes) == 0:
            return [_EmptyResult()]

        sx = orig_w / self.input_w
        sy = orig_h / self.input_h
        boxes[:, 0] *= sx;  boxes[:, 1] *= sy
        boxes[:, 2] *= sx;  boxes[:, 3] *= sy

        keep = _nms(boxes, confs, iou)
        return [_OnnxResult([
            _OnnxBox(xywh=boxes[i:i+1],
                     cls=cls_ids[i:i+1].astype(np.float32),
                     conf=confs[i:i+1])
            for i in keep
        ])]

    def _preprocess(self, img: np.ndarray) -> np.ndarray:
        resized = cv2.resize(img, (self.input_w, self.input_h))
        blob = resized.astype(np.float32) / 255.0
        return blob.transpose(2, 0, 1)[np.newaxis]


def _nms(boxes_xywh: np.ndarray, scores: np.ndarray, iou_thres: float):
    if len(boxes_xywh) == 0:
        return []
    cx, cy, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
    x1 = cx - w / 2; x2 = cx + w / 2
    y1 = cy - h / 2; y2 = cy + h / 2
    areas = (w * h).clip(min=0)
    order = scores.argsort()[::-1]
    keep = []
    while len(order):
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        ix1 = np.maximum(x1[i], x1[order[1:]])
        iy1 = np.maximum(y1[i], y1[order[1:]])
        ix2 = np.minimum(x2[i], x2[order[1:]])
        iy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, ix2 - ix1) * np.maximum(0, iy2 - iy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[1:][iou <= iou_thres]
    return keep


# ─────────────────────────────────────────────────────────────────────────────
#  ROS node
# ─────────────────────────────────────────────────────────────────────────────

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(ObjectDetectionArray, 'vision/detections', 10)
        self.sub_depth_pub_ = self.create_publisher(Float32, 'depth/sub_depth', 10)

    def publish_detections(self, infos):
        """Publish enriched detection data with positions and bbox dimensions.

        Each entry in *infos* is a dict with keys:
            label, confidence, center_x, center_y, bbox_width, bbox_height, depth_m
        center_x/y are normalised to [0, 1] (0.5 = frame centre).
        bbox_width/height are normalised to [0, 1].
        depth_m is Euclidean distance to object in metres (−1 = unknown).
        """
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


# ─────────────────────────────────────────────────────────────────────────────
#  Globals shared between threads
# ─────────────────────────────────────────────────────────────────────────────

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_ONNX = os.path.join(_PKG_DIR, 'zed_right_24_06_15.onnx')

lock = Lock()
run_signal = False
exit_signal = False
detections = []          # sl.CustomBoxObjectData list (for ZED ingestion)
detection_infos = []     # enriched dicts for ROS publishing
model_names = {}         # class-id → name mapping
inference_device = 'gpu'


# ─────────────────────────────────────────────────────────────────────────────
#  Helper functions
# ─────────────────────────────────────────────────────────────────────────────

def xywh2abcd(xywh, im_shape):
    output = np.zeros((4, 2))
    img_h, img_w = im_shape[:2]
    # Clamp to [0, image_size) so ZED's unsigned-int bounding box never gets negative values
    x_min = max(0.0, xywh[0] - 0.5 * xywh[2])
    x_max = min(float(img_w - 1), xywh[0] + 0.5 * xywh[2])
    y_min = max(0.0, xywh[1] - 0.5 * xywh[3])
    y_max = min(float(img_h - 1), xywh[1] + 0.5 * xywh[3])
    output[0] = [x_min, y_min]
    output[1] = [x_max, y_min]
    output[2] = [x_max, y_max]
    output[3] = [x_min, y_max]
    return output


def detections_to_custom_box(detections, im0, names):
    """Convert model boxes to ZED CustomBoxObjectData + enriched info dicts.

    Returns (zed_boxes, info_dicts).
    """
    zed_boxes = []
    info_dicts = []
    img_h, img_w = im0.shape[:2]
    for det in detections:
        xywh = det.xywh[0]  # [cx, cy, w, h] in pixels

        obj = sl.CustomBoxObjectData()
        obj.bounding_box_2d = xywh2abcd(xywh, im0.shape)
        obj.label = int(np.asarray(det.cls).flat[0])
        obj.probability = float(np.asarray(det.conf).flat[0])
        obj.is_grounded = False
        zed_boxes.append(obj)

        cls_id = int(np.asarray(det.cls).flat[0])
        conf = float(np.asarray(det.conf).flat[0])
        cls_name = names.get(cls_id, str(cls_id))
        info_dicts.append({
            'label':       cls_name,
            'confidence':  conf,
            'center_x':    float(xywh[0]) / img_w,
            'center_y':    float(xywh[1]) / img_h,
            'bbox_width':  float(xywh[2]) / img_w,
            'bbox_height': float(xywh[3]) / img_h,
            'depth_m':     -1.0,  # populated later from ZED objects
        })
    return zed_boxes, info_dicts


def enrich_depths(info_dicts, zed_objects):
    """Fill depth_m in info_dicts from ZED's retrieved 3D object positions."""
    for i, obj in enumerate(zed_objects.object_list):
        if i >= len(info_dicts):
            break
        pos = obj.position  # [x, y, z] in metres
        dist = float(np.sqrt(float(pos[0])**2 + float(pos[1])**2 + float(pos[2])**2))
        info_dicts[i]['depth_m'] = dist if dist > 0.01 else -1.0


# ─────────────────────────────────────────────────────────────────────────────
#  Inference thread  (ONNX or YOLO)
# ─────────────────────────────────────────────────────────────────────────────

def inference_thread(weights, img_size, conf_thres=0.2, iou_thres=0.45, device='cuda', onnx_path=None):
    global image_net, exit_signal, run_signal, detections, detection_infos
    global model_names, inference_device

    print('Initializing Network...')

    if onnx_path:
        # Prefer TensorRT (native JetPack 6 GPU) over onnxruntime (cuDNN 8 mismatch)
        model = None
        try:
            import tensorrt, torch
            model = TensorRTDetector(onnx_path)
            print('[inference] Backend: TensorRT GPU')
        except Exception as trt_err:
            print(f'[inference] TensorRT unavailable ({trt_err}), using ONNX/CPU')
        if model is None:
            model = OnnxDetector(onnx_path)
            print('[inference] Backend: ONNX (CPU fallback)')
        model_names = model.names
        use_onnx = True
    else:
        import torch
        from ultralytics import YOLO
        model = YOLO(weights)
        model_names = model.names
        use_onnx = False

    print(f'Model classes: {model_names}')
    inference_device = device

    while not exit_signal:
        if run_signal:
            try:
                lock.acquire()
                img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
                lock.release()

                try:
                    if use_onnx:
                        results = model.predict(img, imgsz=img_size, conf=conf_thres, iou=iou_thres)
                        det = results[0].boxes
                    else:
                        import torch
                        det = model.predict(
                            img, save=False, imgsz=img_size, conf=conf_thres,
                            iou=iou_thres, device=inference_device,
                        )[0].cpu().numpy().boxes
                except RuntimeError as err:
                    err_msg = str(err)
                    print(f'Inference error on device={inference_device}: {err_msg}')
                    if not use_onnx and inference_device != 'cpu':
                        print('Falling back to CPU inference...')
                        inference_device = 'cpu'
                        import torch
                        det = model.predict(
                            img, save=False, imgsz=img_size, conf=conf_thres,
                            iou=iou_thres, device='cpu',
                        )[0].cpu().numpy().boxes
                    else:
                        det = []

                zed_boxes, infos = detections_to_custom_box(det, img, model_names)
                detections = zed_boxes
                detection_infos = infos
            finally:
                if lock.locked():
                    lock.release()
                run_signal = False
        sleep(0.01)


# ─────────────────────────────────────────────────────────────────────────────
#  Main camera + ZED loop
# ─────────────────────────────────────────────────────────────────────────────

def run_detector(node):
    global image_net, exit_signal, run_signal, detections, detection_infos

    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default=_DEFAULT_ONNX,
                        help='PyTorch model path (ignored when --onnx is set)')
    parser.add_argument('--onnx', type=str, default=_DEFAULT_ONNX,
                        help='Path to ONNX model file; if set, uses ONNX inference instead of YOLO')
    parser.add_argument('--classes', type=str, default=None,
                        help='Comma-separated class names for ONNX model, e.g. gate,buoy,torpedo')
    parser.add_argument('--svo', type=str, default=None,
                        help='Optional SVO file; if not passed, uses the plugged camera')
    parser.add_argument('--img_size', type=int, default=416, help='Inference size (pixels)')
    parser.add_argument('--conf_thres', type=float, default=0.4, help='Object confidence threshold')
    parser.add_argument('--device', type=str, default='cuda', help='Inference device (cpu, cuda:0, …)')
    parser.add_argument('--zed_fps', type=int, default=60, help='ZED camera FPS')
    opt = parser.parse_args()

    onnx_path = opt.onnx
    class_names = None
    if onnx_path and opt.classes:
        class_names = [c.strip() for c in opt.classes.split(',') if c.strip()]

    model_label = (onnx_path or opt.weights).split('/')[-1]

    capture_thread = Thread(
        target=inference_thread,
        kwargs={
            'weights':    opt.weights,
            'img_size':   opt.img_size,
            'conf_thres': opt.conf_thres,
            'iou_thres':  0.45,
            'device':     opt.device,
            'onnx_path':  onnx_path,
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

    init_candidates = [
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.NEURAL, opt.zed_fps),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.NONE,   opt.zed_fps),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.NEURAL, opt.zed_fps),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.NONE,   opt.zed_fps),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.NEURAL, 30),
        (sl.RESOLUTION.HD720, sl.DEPTH_MODE.NONE,   30),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.NEURAL, 30),
        (sl.RESOLUTION.VGA,   sl.DEPTH_MODE.NONE,   30),
    ]

    runtime_params = sl.RuntimeParameters()
    status = sl.ERROR_CODE.FAILURE
    init_params = None

    for resolution, depth_mode, fps in init_candidates:
        candidate = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        candidate.coordinate_units = sl.UNIT.METER
        candidate.depth_mode = depth_mode
        candidate.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        candidate.depth_maximum_distance = 30
        candidate.camera_resolution = resolution
        candidate.camera_fps = fps
        status = zed.open(candidate)
        if status == sl.ERROR_CODE.SUCCESS:
            init_params = candidate
            print(f'Initialized Camera: resolution={resolution}, depth={depth_mode}, fps={fps}')
            break
        print(f'ZED open failed ({resolution}, {depth_mode}, {fps}): {status}')

    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        print('\nAll camera configurations failed. Troubleshooting steps:')
        print('  1. Run /usr/local/zed/tools/ZED_Diagnostic for a hardware report.')
        print('  2. Replug the ZED camera into a USB 3.0 port.')
        print('  3. Check that no other process holds the camera: fuser /dev/video0 /dev/video1')
        exit_signal = True
        capture_thread.join(timeout=2.0)
        return

    try:
        # Enable ZED positional tracking so we can report sub depth
        pt_params = sl.PositionalTrackingParameters()
        pt_status = zed.enable_positional_tracking(pt_params)
        if pt_status == sl.ERROR_CODE.SUCCESS:
            positional_tracking_enabled = True
            print('ZED positional tracking enabled')
        else:
            print(f'Positional tracking not available: {pt_status}')

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        obj_param.enable_segmentation = False
        od_status = zed.enable_object_detection(obj_param)
        if od_status != sl.ERROR_CODE.SUCCESS:
            print(f'Failed to enable object detection: {od_status}')
            exit_signal = True
            return
        object_detection_enabled = True

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

        while not exit_signal:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                # Grab frame for inference
                lock.acquire()
                zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
                image_net = image_left_tmp.get_data()
                lock.release()
                run_signal = True

                # Wait for inference thread to finish
                while run_signal and not exit_signal:
                    sleep(0.001)

                # Feed results back into ZED and retrieve 3D-enriched objects
                lock.acquire()
                zed.ingest_custom_box_objects(detections)
                lock.release()
                zed.retrieve_custom_objects(objects, obj_runtime_param)

                # Populate depth_m from ZED's 3D object positions
                lock.acquire()
                enrich_depths(detection_infos, objects)
                local_infos = list(detection_infos)
                lock.release()

                # Publish sub depth from ZED positional tracking
                if positional_tracking_enabled:
                    zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
                    translation = zed_pose.get_translation(sl.Translation()).get()
                    # Y is up in RIGHT_HANDED_Y_UP; negate so positive = depth below surface
                    sub_depth_m = -float(translation[1])
                    node.publish_sub_depth(sub_depth_m)

                # Render and publish
                zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
                image_left_ocv = image_left.get_data().copy()
                cv_viewer.render_2D(image_left_ocv, image_scale, objects, obj_param.enable_tracking)

                cv2.putText(image_left_ocv, f'MODEL: {model_label} conf>={opt.conf_thres:.2f}',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0, 255), 2)
                cv2.putText(image_left_ocv, f'DEVICE: {inference_device} img:{opt.img_size}',
                            (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255, 255), 2)
                cv2.putText(image_left_ocv,
                            f'ZED: {camera_res.width}x{camera_res.height}@{camera_infos.camera_configuration.fps}',
                            (10, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0, 255), 2)
                cv2.imshow('ZED | Live View', image_left_ocv)

                node.publish_detections(local_infos)

                key = cv2.waitKey(1)
                if key in [27, ord('q'), ord('Q')]:
                    exit_signal = True
            else:
                sleep(0.01)
    finally:
        exit_signal = True
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
    rclpy.init(args=None)
    vision_node = VisionNode()
    try:
        run_detector(vision_node)
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
