#!/usr/bin/env python3
"""Shared engine for the isolated-action / stage water-test scripts.

Every ``act_*.py`` and ``stage_*.py`` tool at the repo root imports this. It
provides one consistent path to the real Pixhawk:

  * ``RampedDriver`` — publishes ``auv_msgs/MovementCommand`` on
    ``movement_command`` at 10 Hz, with **linear speed ramping** so the
    thrusters spin up/down smoothly instead of stepping to full power.
    Exposes the mission's basic-function API (imported by the behavior tree):
    ``move_forward() / move_backward() / strafe_left() / strafe_right() /
    move_up() / move_down()`` — condition-bounded continuous moves, ended by
    ``stop_move()`` — and ``turn_left(degrees=90) / turn_right(degrees=90)``,
    closed-loop on the ZED heading (timed fallback without a fix).
  * ``DetectionMonitor`` / ``CoordMonitor`` — subscribe to ``vision/detections``
    and ``vslam/odometry`` for the detection-gated and coordinate tools.
  * ``session(...)`` — context manager that runs the production
    ``mavlink_thruster_control.ThrusterController`` in-process (arm, ALT_HOLD
    mode by default — the autopilot holds depth between/under moves — heartbeat,
    watchdog) alongside the driver and any extra nodes, then guarantees a
    stop + disarm on exit (including Ctrl+C).

All movement tools share the same tuning flags via ``add_move_args``:
``--speed``, ``--ramp-up``, ``--ramp-down``, ``--duration`` (hold at target),
and ``--pause`` (neutral depth-hold between actions).

⚠ SAFETY: importing tools ARM the Pixhawk and drive the REAL thrusters. Stop
``thruster_node`` first (single owner of the serial port). Clear the props,
run on a tether, keep the kill switch reachable. Ctrl+C → stop + disarm.

Requires the ROS 2 workspace to be sourced:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
"""

import math
import time
import threading
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from auv_msgs.msg import MovementCommand, ObjectDetectionArray

# Production thruster driver — reused so these tools drive the real Pixhawk
# exactly like the mission does (arm, ALT_HOLD mode, 10 Hz loop, heartbeat,
# watchdog). Requires the workspace to be sourced.
from mavlink_thruster_control.thruster_node import ThrusterController

RATE_HZ = 10                 # command cadence (matches the thruster loop)
FEET_TO_M = 0.3048

# Open-loop turn calibration: seconds of 'rotate_*' at TURN_SPEED for ~90°.
# Used only when the ZED heading fix is unavailable — CALIBRATE IN WATER.
TURN_90_SECONDS = 3.0
TURN_SPEED = 0.3             # default rotate effort for turn_left/right
ZED_STALE_S = 1.0            # heading fix considered lost after this long
VERTICAL_AXIS = 'y'          # ZED world-up axis (Y_UP → heading is about Y)


# ─── Heading helpers (shared convention with run_course.py) ──────────────────

def heading_about_axis(x, y, z, w, axis=VERTICAL_AXIS):
    """Heading (rad) about the world-up axis from a quaternion (Y_UP → 'y').

    Turns only need a continuous, consistent angle (we integrate deltas), so
    the exact sign convention doesn't matter.
    """
    if axis == 'y':
        return math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))
    if axis == 'z':
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def wrap(angle):
    """Wrap radians to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class HeadingMonitor(Node):
    """Latest ZED heading (rad) from vslam/odometry — feedback for turns."""

    def __init__(self):
        super().__init__('field_heading_monitor')
        self._h = None
        self._t = None
        self.create_subscription(Odometry, 'vslam/odometry', self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        h = heading_about_axis(q.x, q.y, q.z, q.w)
        if h is not None and math.isfinite(h):
            self._h = h
            self._t = time.monotonic()

    def heading(self, stale_s=ZED_STALE_S):
        """Heading in rad, or None if no fresh fix."""
        if self._t is None or time.monotonic() - self._t > stale_s:
            return None
        return self._h


# ─── Movement driver with ramping ───────────────────────────────────────────

class RampedDriver(Node):
    """Publishes MovementCommand at RATE_HZ with linear speed ramping."""

    def __init__(self):
        super().__init__('field_test_driver')
        self.pub = self.create_publisher(MovementCommand, 'movement_command', 10)
        self._period = 1.0 / RATE_HZ
        # Continuous-move streamer state (see move_forward() etc.). The
        # streamer keeps publishing at RATE_HZ between BT ticks so the
        # thruster watchdog / pilot-input failsafe never starves.
        self._heading_mon = None
        self._stream_lock = threading.Lock()
        self._stream_thread = None
        self._stream_cmd = None        # command being streamed (None = idle)
        self._stream_speed = 0.0       # current (ramped) effort
        self._stream_target = 0.0      # effort we are ramping toward
        self._stream_rate = 0.0        # effort change per second (ramp slope)

    def send(self, command, speed):
        """One raw MovementCommand (held until the next — we stream at 10 Hz)."""
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(max(0.0, min(1.0, speed)))
        msg.duration = 0.0
        self.pub.publish(msg)

    def ramp(self, command, s_from, s_to, secs):
        """Linearly walk `command` speed from s_from→s_to over `secs` at 10 Hz."""
        if secs <= 0:
            self.send(command, s_to)
            return
        steps = max(1, int(secs * RATE_HZ))
        for i in range(1, steps + 1):
            self.send(command, s_from + (s_to - s_from) * i / steps)
            time.sleep(self._period)

    def ramp_move(self, command, target, ramp_up, hold, ramp_down=0.0):
        """Ramp up → hold at target → ramp down → stop. The core action verb."""
        self.get_logger().info(
            f'{command}: ramp{ramp_up:.1f}s→{target:.2f}, hold {hold:.1f}s, '
            f'ramp-down {ramp_down:.1f}s')
        self.ramp(command, 0.0, target, ramp_up)
        deadline = time.time() + hold
        while time.time() < deadline:
            self.send(command, target)
            time.sleep(self._period)
        self.ramp(command, target, 0.0, ramp_down)
        self.stop()

    def hold_depth(self, seconds):
        """Neutral horizontal, hold current depth — the 'pause' between moves."""
        if seconds <= 0:
            return
        self.get_logger().info(f'pause: depth-hold {seconds:.1f}s')
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.send('depth_hold', 0.0)
            time.sleep(self._period)

    pause = hold_depth          # alias — reads better at call sites

    def stop(self):
        self.send('stop', 0.0)

    # ─── Basic-function API (condition-bounded moves + degree turns) ─────────
    #
    # move_* / strafe_* are NOT time-based: they start motion and return
    # immediately; motion continues (streamed at RATE_HZ by a background
    # thread) until stop_move() — the caller bounds them with its own
    # condition (object detected, marker dropped, …). turn_left/right are
    # closed-loop on the ZED heading and block until the angle is done.

    def attach_heading(self, monitor):
        """Give the driver a HeadingMonitor — enables closed-loop turns."""
        self._heading_mon = monitor

    def _ensure_streamer(self):
        if self._stream_thread is not None:
            return
        self._stream_thread = threading.Thread(target=self._stream_loop,
                                               daemon=True)
        self._stream_thread.start()

    def _stream_loop(self):
        while True:
            with self._stream_lock:
                cmd = self._stream_cmd
                if cmd is not None:
                    step = self._stream_rate * self._period
                    if self._stream_speed < self._stream_target:
                        self._stream_speed = min(self._stream_target,
                                                 self._stream_speed + step)
                    elif self._stream_speed > self._stream_target:
                        self._stream_speed = max(self._stream_target,
                                                 self._stream_speed - step)
                    speed = self._stream_speed
            if cmd is not None:
                self.send(cmd, speed)
            time.sleep(self._period)

    def _begin(self, command, speed, ramp):
        """Start (or retarget) the continuous stream. Non-blocking."""
        speed = float(max(0.0, min(1.0, speed)))
        self._ensure_streamer()
        with self._stream_lock:
            if self._stream_cmd != command:
                self._stream_speed = 0.0      # new axis — ramp from zero
            self._stream_cmd = command
            self._stream_target = speed
            self._stream_rate = (speed / ramp) if ramp > 0 else float('inf')
        self.get_logger().info(f'{command}: continuous @ {speed:.2f} '
                               f'(ramp {ramp:.1f}s) — until stop_move()')

    def move_forward(self, speed=0.3, ramp=1.0):
        self._begin('surge_forward', speed, ramp)

    def move_backward(self, speed=0.3, ramp=1.0):
        self._begin('surge_backward', speed, ramp)

    def strafe_left(self, speed=0.3, ramp=1.0):
        self._begin('strafe_left', speed, ramp)

    def strafe_right(self, speed=0.3, ramp=1.0):
        self._begin('strafe_right', speed, ramp)

    def move_up(self, speed=0.3, ramp=1.0):
        self._begin('emerge', speed, ramp)

    def move_down(self, speed=0.3, ramp=1.0):
        self._begin('submerge', speed, ramp)

    def stop_move(self, ramp=0.5):
        """End the current continuous move: ramp to zero, then stream
        depth_hold so ALT_HOLD locks depth and the failsafe stays fed."""
        with self._stream_lock:
            cmd = self._stream_cmd
            if cmd in (None, 'depth_hold'):
                self._stream_cmd = 'depth_hold'
                self._stream_speed = self._stream_target = 0.0
                return
            self._stream_target = 0.0
            self._stream_rate = ((self._stream_speed / ramp) if ramp > 0
                                 else float('inf'))
        deadline = time.time() + max(ramp, 0.0) + 0.5
        while time.time() < deadline:
            with self._stream_lock:
                if self._stream_speed <= 0.0:
                    break
            time.sleep(self._period)
        with self._stream_lock:
            self._stream_cmd = 'depth_hold'
            self._stream_speed = self._stream_target = 0.0
        self.get_logger().info('stop_move: neutral — depth-holding')

    def idle(self):
        """Silence the streamer entirely (e.g. before legacy ramp_move())."""
        with self._stream_lock:
            self._stream_cmd = None
            self._stream_speed = self._stream_target = 0.0

    def turn_right(self, degrees=None, speed=TURN_SPEED, timeout=None):
        """Turn right (CW). degrees=None → 90. Blocks until done."""
        return self._turn('rotate_cw', degrees, speed, timeout)

    def turn_left(self, degrees=None, speed=TURN_SPEED, timeout=None):
        """Turn left (CCW). degrees=None → 90. Blocks until done."""
        return self._turn('rotate_ccw', degrees, speed, timeout)

    def _turn(self, command, degrees, speed, timeout):
        """Closed-loop turn on the ZED heading; timed fallback without a fix.

        Integrates wrapped heading deltas, so it works for >180° and is
        immune to the quaternion sign convention. Returns True if the angle
        was closed-loop verified, False if it fell back to timed.
        """
        degrees = 90.0 if degrees is None else abs(float(degrees))
        if degrees == 0:
            return True
        target = math.radians(degrees)
        est = (degrees / 90.0) * TURN_90_SECONDS
        if timeout is None:
            timeout = max(5.0, 3.0 * est)

        mon = self._heading_mon
        h_prev = mon.heading() if mon is not None else None
        closed_loop = h_prev is not None
        if not closed_loop:
            self.get_logger().warn(
                f'turn: no ZED heading fix — timed fallback '
                f'({est:.1f}s for {degrees:.0f}°). CALIBRATE TURN_90_SECONDS.')

        self.get_logger().info(f'{command}: {degrees:.0f}° @ {speed:.2f} '
                               f'({"closed-loop" if closed_loop else "timed"})')
        self._begin(command, speed, ramp=0.5)
        turned = 0.0
        t0 = time.time()
        try:
            while time.time() - t0 < (timeout if closed_loop else est):
                if closed_loop:
                    h = mon.heading()
                    if h is None:
                        # fix lost mid-turn — finish the remainder by time
                        remain = max(0.0, 1.0 - turned / target)
                        self.get_logger().warn(
                            f'turn: heading lost at {math.degrees(turned):.0f}° '
                            f'— timed remainder {remain * est:.1f}s')
                        time.sleep(remain * est)
                        closed_loop = False
                        break
                    turned += abs(wrap(h - h_prev))
                    h_prev = h
                    if turned >= target:
                        break
                time.sleep(self._period)
            else:
                if closed_loop:
                    self.get_logger().warn(
                        f'turn: timeout at {math.degrees(turned):.0f}° of '
                        f'{degrees:.0f}°')
        finally:
            self.stop_move(ramp=0.3)
        if closed_loop:
            self.get_logger().info(f'✓ turned {math.degrees(turned):.0f}°')
        return closed_loop


# ─── Vision / pose monitors ──────────────────────────────────────────────────

class DetectionMonitor(Node):
    """Tracks the freshest detection per label from vision/detections."""

    def __init__(self):
        super().__init__('field_test_detections')
        self._latest = {}        # label -> (ObjectDetection, monotonic_time)
        self.create_subscription(
            ObjectDetectionArray, 'vision/detections', self._on_dets, 10)

    def _on_dets(self, msg: ObjectDetectionArray):
        now = time.monotonic()
        for det in msg.detections:
            self._latest[det.label] = (det, now)

    def best(self, label, min_conf=0.5, stale_s=1.0):
        """Freshest detection of `label` above conf, seen within stale_s — or None."""
        entry = self._latest.get(label)
        if entry is None:
            return None
        det, t = entry
        if time.monotonic() - t > stale_s:
            return None
        if det.confidence < min_conf:
            return None
        return det

    def seen(self, label, min_conf=0.5, stale_s=1.0):
        return self.best(label, min_conf, stale_s) is not None


class DepthMonitor(Node):
    """Latest sub depth (m, +down) from depth/sub_depth (ZED-derived)."""

    def __init__(self):
        super().__init__('field_depth_monitor')
        self._d = None
        self._t = None
        self.create_subscription(Float32, 'depth/sub_depth', self._on_depth, 10)

    def _on_depth(self, msg: Float32):
        self._d = float(msg.data)
        self._t = time.monotonic()

    def depth(self, stale_s=2.0):
        """Depth in m, or None if nothing fresh (vision/ZED not publishing)."""
        if self._t is None or time.monotonic() - self._t > stale_s:
            return None
        return self._d


class CoordMonitor(Node):
    """Latest ZED positional-tracking pose from vslam/odometry."""

    def __init__(self):
        super().__init__('field_test_coords')
        self.x = self.y = self.z = None
        self._t = None
        self.create_subscription(Odometry, 'vslam/odometry', self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y, self.z = float(p.x), float(p.y), float(p.z)
        self._t = time.monotonic()

    def have_fix(self, stale_s=1.0):
        return self._t is not None and time.monotonic() - self._t <= stale_s


# ─── Detection-gated descent (shared by the *_detect stage tools) ────────────

def descend_until(driver, monitor, label, speed, ramp_up, ramp_down,
                  timeout, conf=0.5):
    """Ramp into a descent, hold it until `label` is detected or `timeout`.

    Returns True if the target was detected, False on timeout. Always ends
    ramped down + depth-holding.
    """
    driver.get_logger().info(
        f'Submerging until "{label}" detected (timeout {timeout:.0f}s)…')
    driver.ramp('submerge', 0.0, speed, ramp_up)
    detected = False
    deadline = time.time() + timeout
    period = 1.0 / RATE_HZ
    while time.time() < deadline:
        if monitor.seen(label, conf):
            driver.get_logger().info(f'✓ "{label}" detected — stopping descent.')
            detected = True
            break
        driver.send('submerge', speed)
        time.sleep(period)
    if not detected:
        driver.get_logger().warn(
            f'"{label}" not seen in {timeout:.0f}s — timeout, proceeding.')
    driver.ramp('submerge', speed, 0.0, ramp_down)
    return detected


# ─── Around-the-marker maneuver (shared by both marker stage tools) ──────────

def around_marker(driver, speed, ramp_up, ramp_down, leg, turn):
    """Open-loop scripted circle around a marker, mirroring the prequal path.

    strafe right → forward → turn left → forward → turn left → forward.
    `leg` = seconds per straight/strafe leg, `turn` = seconds per ~90° turn.
    """
    driver.get_logger().info('Around-marker maneuver…')
    driver.ramp_move('strafe_right', speed, ramp_up, leg, ramp_down)
    driver.ramp_move('surge_forward', speed, ramp_up, leg, ramp_down)
    driver.ramp_move('rotate_ccw', speed, ramp_up, turn, ramp_down)   # turn left
    driver.ramp_move('surge_forward', speed, ramp_up, leg, ramp_down)
    driver.ramp_move('rotate_ccw', speed, ramp_up, turn, ramp_down)
    driver.ramp_move('surge_forward', speed, ramp_up, leg, ramp_down)


# ─── Session / argument plumbing ─────────────────────────────────────────────

def find_node(nodes, cls):
    """Return the first node in `nodes` that is an instance of `cls`."""
    return next((n for n in nodes if isinstance(n, cls)), None)


@contextmanager
def session(extra_factory=None, *, confirm_msg=None, skip_confirm=False):
    """Run ThrusterController + RampedDriver (+ extra nodes) for the duration.

    `extra_factory` is called AFTER rclpy.init() and must return a list of
    extra rclpy Nodes (e.g. monitors, in-process vision/vslam). Yields
    ``(driver, extra_nodes)``. Guarantees stop + disarm on exit.
    """
    if confirm_msg and not skip_confirm:
        print('\n' + confirm_msg)
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            raise SystemExit(1)

    rclpy.init()
    driver = RampedDriver()
    thrusters = ThrusterController()
    heading = HeadingMonitor()
    driver.attach_heading(heading)     # enables closed-loop turn_left/right
    driver.thrusters = thrusters       # e.g. dropper shares thrusters.master
    extra = list(extra_factory()) if extra_factory else []
    nodes = [driver, thrusters, heading, *extra]

    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('\n⚠ ARMING PIXHAWK — thrusters WILL drive. Keep clear. '
          'Ctrl+C to stop + disarm.\n')
    try:
        yield driver, extra
    except KeyboardInterrupt:
        print('\nInterrupted — stopping.')
    finally:
        try:
            driver.idle()              # silence the continuous-move streamer
            driver.stop()
            time.sleep(0.1)
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        # Destroy the thruster node first so its stop+disarm cleanup runs while
        # the MAVLink link is still up.
        for n in (thrusters, *extra, heading, driver):
            try:
                n.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
        print('Done. Neutral + disarmed.')


def add_move_args(ap, *, speed=0.3, ramp_up=1.0, ramp_down=0.5, duration=3.0,
                  pause=2.0):
    """Attach the common speed/ramp tuning flags shared by every tool."""
    ap.add_argument('--speed', type=float, default=speed,
                    help=f'target effort 0-1 (default {speed})')
    ap.add_argument('--ramp-up', type=float, default=ramp_up,
                    help=f'seconds to ramp 0→speed (default {ramp_up})')
    ap.add_argument('--ramp-down', type=float, default=ramp_down,
                    help=f'seconds to ramp speed→0 (default {ramp_down})')
    ap.add_argument('--duration', type=float, default=duration,
                    help=f'seconds to hold at target speed (default {duration})')
    ap.add_argument('--pause', type=float, default=pause,
                    help=f'depth-hold pause after the action (default {pause})')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    return ap


def spawn_vision_factory(monitor_extra=None, model_onnx=None):
    """extra_factory that spawns the TensorRT detector in-process + a monitor.

    Starts the detector's ZED-capture + inference loop (``run_detector``) in a
    daemon thread — without it VisionNode never publishes a detection.
    ``model_onnx`` overrides the forward-camera model path (default:
    vision/ffc_rs_26.onnx next to detector.py).

    Movement-only tools should NOT use this (no GPU/engine load needed).
    """
    def factory():
        import atexit
        import sys
        from vision import detector as det_mod

        vision_node = det_mod.VisionNode()
        # run_detector() parses sys.argv — replace it so the caller's own
        # CLI flags don't crash the detector's parser. Callers are done
        # parsing by the time session() invokes this factory.
        argv = [sys.argv[0]]
        if model_onnx:
            argv += ['--onnx', model_onnx]
        sys.argv = argv
        threading.Thread(target=det_mod.run_detector, args=(vision_node,),
                         daemon=True).start()

        def _release_camera():
            # let run_detector's finally close the ZED — otherwise the camera
            # stays locked and the next run fails until a power cycle
            det_mod.exit_signal = True
            time.sleep(1.0)
        atexit.register(_release_camera)

        nodes = [DetectionMonitor(), vision_node]
        if monitor_extra:
            nodes.extend(monitor_extra())
        return nodes
    return factory
