#!/usr/bin/env python3
"""Vision-guided gate runner — submerge, see the gate, square up, pass through.

Mission sequence:
    1. submerge to depth (open-loop, held by constant z — see DEPTH note)
    2. GATE 1 : detect 'gate' (YOLO), SQUARE UP to it + center, creep forward,
                then commit a straight push to pass through
    3. MARKER : HARDCODED open-loop path (NO detection, by design) —
                forward → strafe right 5 ft → forward → strafe left 7 ft →
                turn 180° to face start → forward
    4. GATE 2 : detect, square up + center + approach, pass through, and keep
                driving to clear at least CLEAR_FT past it at the end

Anti-angle design (so the sub enters the gate square, not skewed):
    • CENTERING uses STRAFE only — the sub slides sideways to line up while
      holding heading. (Yawing to chase an off-center gate is what made the old
      version arc in at an angle; that behaviour is removed.)
    • YAW is used ONLY to SQUARE UP to the gate plane. With the ZED depth map we
      sample the gate's left vs right post depth; if one side is nearer, the gate
      is rotated relative to us, so we rotate to equalise those depths. Heading is
      held on the resulting (squared) bearing through the whole pass-through.
    • Forward thrust is throttled down whenever the sub is off-center OR not yet
      squared, so it straightens BEFORE closing the distance.
    • Graceful fallback: no depth → hold the locked heading + strafe-center; no
      heading source either → strafe-center on a fixed yaw. Each step still helps.

manual_control axes:  x surge(+fwd)  y strafe(+right)  z depth(500 neutral,
<500 deeper)  r yaw(+CW).

⚠ THRUSTERS WILL SPIN. Stop thruster_node first (single serial owner). Clear the
props / run on a tether, keep the kill switch reachable. Ctrl+C → stop+disarm.

  python3 gate_runner.py
"""
from pymavlink import mavutil
import time
import math
import threading
import numpy as np

# ============================ CONFIG ========================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200
ARM_SETTLE_TIME = 0
CONFIRM = True               # ask "go" before arming; False = run immediately

# ── DEPTH ───────────────────────────────────────────────────────────────────
# Sealed-in-hull baro can't feed ALT_HOLD, so default is the open-loop fallback:
# STABILIZE keeps the sub level (IMU) and we hold a constant below-neutral z on
# every leg. With a real water pressure sensor, set FLIGHT_MODE='ALT_HOLD' and
# DEPTH_Z=500.
FLIGHT_MODE = 'STABILIZE'
DEPTH_Z     = 430            # <500 = net down-thrust; tune to ~neutral buoyancy
NEUTRAL_Z   = 500            # used only if you switch to ALT_HOLD
SUBMERGE_Z      = 200
SUBMERGE_TIME   = 5.0

# ── Control loop / command stream ───────────────────────────────────────────
RATE_HZ = 15
PERIOD  = 1.0 / RATE_HZ

# ── Vision / detection ──────────────────────────────────────────────────────
MODEL_PATH   = 'best.pt'
IMG_SIZE     = 640
CONF_THRESH  = 0.45
GATE_CLASS   = 0             # 'gate'
DET_STALE_S  = 0.4

FRAME_SOURCE      = 'ros2'   # 'ros2' | 'opencv'
ZED_IMAGE_TOPIC   = '/zed/zed_node/left/image_rect_color'
CAMERA_INDEX      = 0

# ── Horizontal centering (STRAFE only — this is the anti-arc fix) ───────────
# ex = (gate_center_x - image_center_x)/(image_width/2)  ∈ [-1,+1]; +ex = right.
KP_STRAFE   = 600
STRAFE_MAX  = 350
EX_SMOOTH   = 0.5            # EMA on ex/frac (0..1, higher = snappier)
EX_PASS     = 0.12          # |ex| this small counts as "centered" for commit

# ── Heading lock (hold a straight bearing; needs ZED odometry) ──────────────
USE_HEADING_LOCK = True
ODOM_TOPIC    = 'vslam/odometry'
VERTICAL_AXIS = 'y'          # world-up axis of the ZED frame (Y_UP → 'y')
HEAD_STALE_S  = 0.5
YAW_SIGN      = 1            # flip to -1 if heading-hold corrects the wrong way
KP_YAW_HOLD   = 0.8          # yaw effort (0..1) per radian of heading error
YAW_MAX       = 200          # clamp on all yaw output

# ── Depth-based gate-plane squaring (needs ZED depth map) ───────────────────
USE_DEPTH_SQUARING = True
DEPTH_TOPIC      = '/zed/zed_node/depth/depth_registered'
EDGE_FRAC        = 0.12      # post sampling strip = this fraction of bbox width
VBAND_TRIM       = 0.20      # ignore top/bottom 20% of bbox when sampling depth
DEPTH_MIN_M      = 0.3       # valid stereo range
DEPTH_MAX_M      = 30.0
DEPTH_SQUARE_SIGN = 1        # flip to -1 if squaring rotates the wrong way
KP_SQUARE        = 0.15      # rad/s of heading change per metre of L/R depth err
KP_SQUARE_DIRECT = 250       # raw yaw per metre (used only if NO heading source)
MAX_SQUARE_OFFSET = 0.5      # rad: cap how far squaring may rotate off the lock
DEPTH_ERR_FULL   = 0.6       # |L-R| depth (m) at/above which forward is throttled
DEPTH_ERR_OK     = 0.12      # |L-R| depth (m) below which we count as "squared"

# ── Forward approach behaviour ──────────────────────────────────────────────
FWD_APPROACH = 350
FWD_MIN      = 120
EX_SLOW      = 0.6
GATE_CLOSE_FRAC = 0.55       # bbox_width/img_width meaning "close" → pass-through
CLOSE_ALIGN_TIMEOUT = 4.0    # if close but never aligns/squares, commit anyway

# ── Pass-through (open-loop, timed; holds the squared heading) ──────────────
FWD_PASS        = 450
GATE1_PASS_TIME = 4.0
CLEAR_FT        = 3.0
GATE2_PASS_TIME = 6.0        # straight push for gate 2 incl. the CLEAR_FT margin

# ── Search when no gate is visible ──────────────────────────────────────────
SEARCH_YAW       = 160
SEARCH_SWEEP_S   = 2.5
SEARCH_TIMEOUT   = 25.0
LOST_CLOSE_FRAMES = 6

# ── Marker sequence (HARDCODED, open-loop, NO detection by design) ──────────
# Between the two gates the sub runs this fixed, timed path. Distances are hit
# by TIME (calibrate each *_TIME against your real thrust), and every leg holds
# depth + the latched heading so the forwards/strafes stay straight (no curl).
#   forward → strafe right 5 ft → forward → strafe left 7 ft → turn 180 → forward
M_FWD1_THRUST     = 350      # 1) forward toward the marker
M_FWD1_TIME       = 3.0
M_STRAFE_R_THRUST = 400      # 2) strafe RIGHT 5 ft   (+y)
M_STRAFE_R_TIME   = 5.5
M_FWD2_THRUST     = 350      # 3) forward past the marker
M_FWD2_TIME       = 3.0
M_STRAFE_L_THRUST = 400      # 4) strafe LEFT 7 ft    (-y)
M_STRAFE_L_TIME   = 7.5
M_FWD3_THRUST     = 350      # 6) forward past the marker (after the turn)
M_FWD3_TIME       = 3.0
# 5) turn 180° to face the starting point.
#    Closed-loop on ZED heading when available (accumulates measured rotation,
#    so it's an actual 180° regardless of thrust), else a timed fallback.
TURN_DEG     = 180          # +CW
TURN_YAW     = 300          # yaw effort during a turn
TURN_TOL_DEG = 5            # stop within this many degrees of target
TURN_TIMEOUT = 15.0         # safety cap on the closed-loop turn
TURN_TIME    = 6.0          # TIMED fallback duration if no heading source
# ============================================================================


# ── Shared detection state (written by the detector thread) ─────────────────
_det_lock = threading.Lock()
_det = {'t': None, 'ex': 0.0, 'frac': 0.0, 'conf': 0.0, 'seen': False,
        'bbox': None, 'fw': 0, 'fh': 0}     # bbox = (x1,y1,x2,y2) in det pixels
_frame_source = None
_depth_source = None
_heading_source = None
_detector_run = threading.Event()


# ── Frame sources ───────────────────────────────────────────────────────────
class OpenCVFrameSource:
    def __init__(self, index):
        import cv2
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"OpenCV camera {index} did not open")

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


class Ros2ImageSource:
    """Subscribe to a ROS2 sensor_msgs/Image topic; keep the latest BGR frame."""
    def __init__(self, topic, encoding='bgr8'):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        self._bridge = CvBridge()
        self._enc = encoding
        self._latest = None
        self._lock = threading.Lock()
        try:
            rclpy.init()
        except Exception:
            pass

        class _N(Node):
            def __init__(self, outer):
                super().__init__('gate_runner_' + topic.strip('/').replace('/', '_'))
                self._outer = outer
                self.create_subscription(Image, topic, self._cb, 10)

            def _cb(self, msg):
                frame = self._outer._bridge.imgmsg_to_cv2(msg, self._outer._enc)
                with self._outer._lock:
                    self._outer._latest = frame

        from rclpy.executors import SingleThreadedExecutor
        self._node = _N(self)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        threading.Thread(target=self._exec.spin, daemon=True).start()

    def read(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def close(self):
        try:
            self._exec.shutdown()
            self._node.destroy_node()
        except Exception:
            pass


def make_frame_source():
    if FRAME_SOURCE == 'opencv':
        return OpenCVFrameSource(CAMERA_INDEX)
    return Ros2ImageSource(ZED_IMAGE_TOPIC, encoding='bgr8')


def make_depth_source():
    """Depth map for squaring; only meaningful with the ROS2 ZED stack."""
    if not USE_DEPTH_SQUARING or FRAME_SOURCE != 'ros2':
        return None
    try:
        return Ros2ImageSource(DEPTH_TOPIC, encoding='passthrough')
    except Exception as exc:
        print(f"[square] depth source unavailable ({exc}) — "
              f"squaring falls back to heading-hold only.")
        return None


# ── Heading source (ZED odometry quaternion → yaw) ──────────────────────────
class HeadingSource:
    def __init__(self, topic):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from nav_msgs.msg import Odometry
        self._h = None
        self._t = None
        self._lock = threading.Lock()
        try:
            rclpy.init()
        except Exception:
            pass

        class _N(Node):
            def __init__(self, outer):
                super().__init__('gate_runner_heading')
                self._outer = outer
                self.create_subscription(Odometry, topic, self._cb, 10)

            def _cb(self, msg):
                q = msg.pose.pose.orientation
                self._outer._update(q.x, q.y, q.z, q.w)

        self._node = _N(self)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        threading.Thread(target=self._exec.spin, daemon=True).start()

    def _update(self, x, y, z, w):
        h = heading_about_axis(x, y, z, w, VERTICAL_AXIS)
        if h is None or not math.isfinite(h):
            return
        with self._lock:
            self._h, self._t = h, time.monotonic()

    def get(self):
        with self._lock:
            if self._t is None or time.monotonic() - self._t > HEAD_STALE_S:
                return None
            return self._h

    def close(self):
        try:
            self._exec.shutdown()
            self._node.destroy_node()
        except Exception:
            pass


def heading_about_axis(x, y, z, w, axis):
    if axis == 'y':
        return math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))
    if axis == 'z':
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def get_heading():
    return _heading_source.get() if (_heading_source is not None) else None


# ── Detector thread: publish best gate's (ex, frac, conf, bbox) ─────────────
def detector_loop(model):
    ex_s, frac_s, have = 0.0, 0.0, False
    while _detector_run.is_set():
        frame = _frame_source.read()
        if frame is None:
            time.sleep(0.01)
            continue
        h, w = frame.shape[:2]
        res = model.predict(frame, imgsz=IMG_SIZE, conf=CONF_THRESH,
                            classes=[GATE_CLASS], verbose=False)[0]
        boxes = res.boxes
        best = None
        if boxes is not None and len(boxes) > 0:
            xywh = boxes.xywh.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            areas = xywh[:, 2] * xywh[:, 3]
            i = int(np.argmax(areas))
            cx, _, bw, _ = xywh[i]
            best = (float((cx - w / 2.0) / (w / 2.0)), float(bw / w),
                    float(confs[i]), tuple(float(v) for v in xyxy[i]))
        if best is not None:
            ex, frac, conf, bbox = best
            if have:
                ex_s = EX_SMOOTH * ex + (1 - EX_SMOOTH) * ex_s
                frac_s = EX_SMOOTH * frac + (1 - EX_SMOOTH) * frac_s
            else:
                ex_s, frac_s, have = ex, frac, True
            with _det_lock:
                _det.update(t=time.monotonic(), ex=ex_s, frac=frac_s, conf=conf,
                            seen=True, bbox=bbox, fw=w, fh=h)
        else:
            have = False
            with _det_lock:
                _det['seen'] = False
        time.sleep(0.005)


def get_detection():
    with _det_lock:
        t = _det['t']
        fresh = (t is not None and _det['seen']
                 and time.monotonic() - t <= DET_STALE_S)
        return (fresh, _det['ex'], _det['frac'], _det['conf'],
                _det['bbox'], _det['fw'], _det['fh'])


def sample_edge_depths(bbox, fw, fh):
    """Median depth (m) in the left-post and right-post strips of the gate bbox.
    Returns (left_d, right_d), either may be None if too few valid pixels.
    """
    if _depth_source is None or bbox is None:
        return None, None
    depth = _depth_source.read()
    if depth is None:
        return None, None
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    dh, dw = depth.shape[:2]
    sx = dw / float(fw) if fw else 1.0
    sy = dh / float(fh) if fh else 1.0
    x1, y1, x2, y2 = bbox
    x1, x2 = sorted((x1 * sx, x2 * sx))
    y1, y2 = sorted((y1 * sy, y2 * sy))
    bw = max(2.0, x2 - x1)
    strip = max(2, int(EDGE_FRAC * bw))
    yt = int(y1 + VBAND_TRIM * (y2 - y1))
    yb = int(y2 - VBAND_TRIM * (y2 - y1))
    yt, yb = max(0, yt), min(dh, max(yt + 1, yb))

    def med(xa, xb):
        xa, xb = max(0, int(xa)), min(dw, int(xb))
        if xb - xa < 1 or yb - yt < 1:
            return None
        patch = depth[yt:yb, xa:xb].ravel()
        valid = patch[np.isfinite(patch) & (patch > DEPTH_MIN_M)
                      & (patch < DEPTH_MAX_M)]
        return float(np.median(valid)) if valid.size >= 10 else None

    return med(x1, x1 + strip), med(x2 - strip, x2)


# ── MAVLink helpers ─────────────────────────────────────────────────────────
def connect():
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system}, "
          f"component {master.target_component}")
    return master


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Mode {mode_name} not in available modes: "
                         f"{list(mode_map.keys())}")
    master.set_mode(mode_map[mode_name])
    print(f">>> Mode set to {mode_name}")


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    print("Vehicle armed")
    time.sleep(ARM_SETTLE_TIME)


def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    print("Vehicle disarmed")
    time.sleep(2)


def cruise_z():
    return NEUTRAL_Z if FLIGHT_MODE == 'ALT_HOLD' else DEPTH_Z


def send_manual_control(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        max(-1000, min(1000, int(x))),
        max(-1000, min(1000, int(y))),
        max(0,     min(1000, int(z))),
        max(-1000, min(1000, int(r))),
        buttons=0)
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def heading_hold_yaw(cur, target):
    """Yaw command to hold `target` heading, or 0 if no heading available."""
    if cur is None or target is None:
        return 0
    return clamp(YAW_SIGN * KP_YAW_HOLD * wrap(target - cur) * 1000,
                 -YAW_MAX, YAW_MAX)


# ── Mission legs ────────────────────────────────────────────────────────────
def submerge(master):
    print(f"\n>>> SUBMERGE  ({SUBMERGE_TIME}s @ z={SUBMERGE_Z})")
    start = time.time()
    while time.time() - start < SUBMERGE_TIME:
        send_manual_control(master, 0, 0, SUBMERGE_Z, 0)
        time.sleep(PERIOD)


def drive_leg(master, label, x, y, seconds, hold_heading):
    """HARDCODED open-loop leg: hold (x, y) for `seconds`, keeping depth and the
    latched heading (so forwards/strafes track straight instead of curling)."""
    print(f"\n  · {label}  ({seconds}s  x={x} y={y}, depth+heading held)")
    z = cruise_z()
    start = time.time()
    while time.time() - start < seconds:
        cur = get_heading() if USE_HEADING_LOCK else None
        send_manual_control(master, x, y, z, heading_hold_yaw(cur, hold_heading))
        time.sleep(PERIOD)


def turn(master, degrees):
    """Rotate `degrees` (CW positive) and return the new heading to hold.
    Closed-loop on the ZED heading when available — it accumulates the MEASURED
    rotation (robust to the ±180° wrap ambiguity), so it's a true 180° rather
    than a guess at thrust×time. Falls back to a timed turn with no heading."""
    z = cruise_z()
    cur = get_heading() if USE_HEADING_LOCK else None
    d = 1 if degrees >= 0 else -1
    if cur is None:
        print(f"\n  · turn {degrees}° (TIMED fallback {TURN_TIME}s — no heading)")
        start = time.time()
        while time.time() - start < TURN_TIME:
            send_manual_control(master, 0, 0, z, d * TURN_YAW)
            time.sleep(PERIOD)
        send_manual_control(master, 0, 0, z, 0)
        return None

    print(f"\n  · turn {degrees}° (closed-loop on heading)")
    goal = math.radians(abs(degrees))
    tol = math.radians(TURN_TOL_DEG)
    turned, prev = 0.0, cur
    start = time.time()
    while turned < goal - tol:
        if time.time() - start > TURN_TIMEOUT:
            print("    turn timeout — stopping.")
            break
        h = get_heading()
        if h is not None and prev is not None:
            turned += abs(wrap(h - prev))     # unwrapped accumulation
            prev = h
        send_manual_control(master, 0, 0, z, d * TURN_YAW)
        time.sleep(PERIOD)
    send_manual_control(master, 0, 0, z, 0)
    print(f"    turned ~{math.degrees(turned):.0f}°")
    new_h = get_heading()
    return new_h if new_h is not None else cur


def run_marker_sequence(master):
    """The fixed, hardcoded path between the two gates (no detection)."""
    print("\n>>> MARKER SEQUENCE (hardcoded, no detection)")
    hd = get_heading() if USE_HEADING_LOCK else None   # latch bearing to hold
    drive_leg(master, "forward toward marker", M_FWD1_THRUST, 0, M_FWD1_TIME, hd)
    drive_leg(master, "strafe right 5 ft",     0,  M_STRAFE_R_THRUST, M_STRAFE_R_TIME, hd)
    drive_leg(master, "forward past marker",   M_FWD2_THRUST, 0, M_FWD2_TIME, hd)
    drive_leg(master, "strafe left 7 ft",      0, -M_STRAFE_L_THRUST, M_STRAFE_L_TIME, hd)
    hd = turn(master, TURN_DEG)                          # re-latch after the 180
    drive_leg(master, "forward past marker (after turn)", M_FWD3_THRUST, 0, M_FWD3_TIME, hd)


def approach_gate(master, label, pass_time):
    """Square up to the gate plane + center horizontally while creeping forward,
    then commit a straight, heading-held pass-through. Returns True if passed.
    """
    print(f"\n>>> {label}  — searching for gate...")
    z = cruise_z()
    init_heading = get_heading() if USE_HEADING_LOCK else None
    target_heading = init_heading
    acquired = False
    lost = 0
    sweep_dir = 1
    sweep_t0 = time.time()
    search_t0 = time.time()
    close_unaligned_t0 = None

    while True:
        fresh, ex, frac, conf, bbox, fw, fh = get_detection()
        cur = get_heading() if USE_HEADING_LOCK else None

        if fresh:
            acquired = True
            lost = 0
            search_t0 = time.time()

            # --- squaring: measure L/R post depth, drive yaw to equalise ---
            depth_err = None
            if USE_DEPTH_SQUARING:
                ld, rd = sample_edge_depths(bbox, fw, fh)
                if ld is not None and rd is not None:
                    depth_err = rd - ld          # >0: right post farther

            if depth_err is not None:
                if cur is not None and target_heading is not None:
                    target_heading += (DEPTH_SQUARE_SIGN * KP_SQUARE
                                       * depth_err * PERIOD)
                    off = clamp(wrap(target_heading - init_heading),
                                -MAX_SQUARE_OFFSET, MAX_SQUARE_OFFSET)
                    target_heading = init_heading + off
                    yaw = heading_hold_yaw(cur, target_heading)
                else:                            # no heading source: direct yaw
                    yaw = clamp(DEPTH_SQUARE_SIGN * KP_SQUARE_DIRECT * depth_err,
                                -YAW_MAX, YAW_MAX)
            else:
                yaw = heading_hold_yaw(cur, target_heading)

            # --- centering: STRAFE only (never yaw toward the gate) ---
            strafe = clamp(KP_STRAFE * ex, -STRAFE_MAX, STRAFE_MAX)

            # --- forward: throttle by both off-center AND not-squared ---
            ex_term = abs(ex) / EX_SLOW
            sq_term = (abs(depth_err) / DEPTH_ERR_FULL) if depth_err is not None else 0.0
            slow = clamp(1.0 - max(ex_term, sq_term), 0.0, 1.0)
            fwd = FWD_MIN + (FWD_APPROACH - FWD_MIN) * slow

            aligned = abs(ex) <= EX_PASS
            squared = (abs(depth_err) <= DEPTH_ERR_OK) if depth_err is not None else True

            if frac >= GATE_CLOSE_FRAC:
                if aligned and squared:
                    print(f"  close+aligned (frac={frac:.2f} ex={ex:+.2f}"
                          + (f" dErr={depth_err:+.2f}m" if depth_err is not None else "")
                          + ") → pass-through")
                    break
                if close_unaligned_t0 is None:
                    close_unaligned_t0 = time.time()
                elif time.time() - close_unaligned_t0 > CLOSE_ALIGN_TIMEOUT:
                    print("  close-align timeout → pass-through anyway")
                    break
                # at the gate mouth but not square/centered: fix it, don't push in
                send_manual_control(master, 0, strafe, z, yaw)
                time.sleep(PERIOD)
                continue
            else:
                close_unaligned_t0 = None

            send_manual_control(master, fwd, strafe, z, yaw)

        else:
            if acquired:
                lost += 1
                if lost >= LOST_CLOSE_FRAMES:
                    print("  gate lost after approach → pass-through")
                    break
                send_manual_control(master, FWD_MIN, 0, z,
                                    heading_hold_yaw(cur, target_heading))
            else:
                if time.time() - search_t0 > SEARCH_TIMEOUT:
                    print("  SEARCH TIMEOUT — no gate found, holding.")
                    send_manual_control(master, 0, 0, z, 0)
                    return False
                if time.time() - sweep_t0 > SEARCH_SWEEP_S:
                    sweep_dir *= -1
                    sweep_t0 = time.time()
                send_manual_control(master, 0, 0, z, SEARCH_YAW * sweep_dir)
        time.sleep(PERIOD)

    # Commit: straight, timed pass-through, HOLDING the squared heading so we go
    # through perpendicular even though vision is now useless inside the gate.
    print(f"  PASS-THROUGH  ({pass_time}s @ x={FWD_PASS}, heading held)")
    start = time.time()
    while time.time() - start < pass_time:
        cur = get_heading() if USE_HEADING_LOCK else None
        send_manual_control(master, FWD_PASS, 0, z,
                            heading_hold_yaw(cur, target_heading))
        time.sleep(PERIOD)
    return True


# ── Main ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\nVISION GATE RUN — THRUSTERS WILL SPIN, SUB WILL MOVE.")
    print(f"  mode={FLIGHT_MODE}  depth_z={cruise_z()}  model={MODEL_PATH}")
    print(f"  heading-lock {'ON' if USE_HEADING_LOCK else 'OFF'} | "
          f"depth-squaring {'ON' if USE_DEPTH_SQUARING else 'OFF'}")
    print(f"  gate1 pass={GATE1_PASS_TIME}s   gate2 pass={GATE2_PASS_TIME}s "
          f"(incl. ~{CLEAR_FT} ft clearance)")
    if CONFIRM:
        if input('Props clear? type "go" to run: ').strip().lower() != "go":
            print("Aborted.")
            raise SystemExit(1)

    print("Loading YOLO model...")
    from ultralytics import YOLO
    model = YOLO(MODEL_PATH)
    _frame_source = make_frame_source()
    _depth_source = make_depth_source()
    if USE_HEADING_LOCK:
        try:
            _heading_source = HeadingSource(ODOM_TOPIC)
        except Exception as exc:
            print(f"[heading] odometry unavailable ({exc}) — "
                  f"heading-hold disabled, strafe-centering only.")
            _heading_source = None
    _detector_run.set()
    threading.Thread(target=detector_loop, args=(model,), daemon=True).start()
    print("Detector running — waiting for first frames...")
    time.sleep(2.0)

    master = connect()
    set_mode(master, FLIGHT_MODE)
    arm(master)

    try:
        submerge(master)
        approach_gate(master, "GATE 1", GATE1_PASS_TIME)
        run_marker_sequence(master)
        approach_gate(master, "GATE 2", GATE2_PASS_TIME)
        print("\nCourse complete.")
    except KeyboardInterrupt:
        print("\nInterrupted — stopping and disarming.")
    finally:
        try:
            send_manual_control(master, 0, 0, cruise_z(), 0)
            disarm(master)
        except Exception:
            pass
        _detector_run.clear()
        for src in (_frame_source, _depth_source, _heading_source):
            if src is not None:
                try:
                    src.close()
                except Exception:
                    pass
