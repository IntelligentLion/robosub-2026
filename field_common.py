#!/usr/bin/env python3
"""Shared engine for the isolated-action / stage water-test scripts.

Every ``act_*.py`` and ``stage_*.py`` tool at the repo root imports this. It
provides one consistent path to the real Pixhawk:

  * ``RampedDriver`` — publishes ``auv_msgs/MovementCommand`` on
    ``movement_command`` at 10 Hz, with **linear speed ramping** so the
    thrusters spin up/down smoothly instead of stepping to full power.
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

import time
import threading
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from auv_msgs.msg import MovementCommand, ObjectDetectionArray

# Production thruster driver — reused so these tools drive the real Pixhawk
# exactly like the mission does (arm, ALT_HOLD mode, 10 Hz loop, heartbeat,
# watchdog). Requires the workspace to be sourced.
from mavlink_thruster_control.thruster_node import ThrusterController

RATE_HZ = 10                 # command cadence (matches the thruster loop)
FEET_TO_M = 0.3048


# ─── Movement driver with ramping ───────────────────────────────────────────

class RampedDriver(Node):
    """Publishes MovementCommand at RATE_HZ with linear speed ramping."""

    def __init__(self):
        super().__init__('field_test_driver')
        self.pub = self.create_publisher(MovementCommand, 'movement_command', 10)
        self._period = 1.0 / RATE_HZ

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
    extra = list(extra_factory()) if extra_factory else []
    nodes = [driver, thrusters, *extra]

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
        for n in (thrusters, *extra, driver):
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


def spawn_vision_factory(monitor_extra=None):
    """extra_factory that spawns the TensorRT detector in-process + a monitor.

    Movement-only tools should NOT use this (no GPU/engine load needed).
    """
    def factory():
        from vision.detector import VisionNode
        nodes = [DetectionMonitor(), VisionNode()]
        if monitor_extra:
            nodes.extend(monitor_extra())
        return nodes
    return factory
