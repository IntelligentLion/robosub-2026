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

# F2 guard: symlink-install makes Python edits live but auv_msgs is GENERATED
# code in install/ — after any pull touching msg/, DepthKeeper._send would die
# on the first tick with AttributeError inside a daemon thread. Fail at import
# instead, with the fix in the message.
if not hasattr(MovementCommand(), 'pitch_rate'):
    raise ImportError(
        'auv_msgs is STALE (MovementCommand has no pitch_rate). Rebuild:\n'
        '  colcon build --symlink-install --packages-select auv_msgs '
        'mavlink_thruster_control bt_mission\n'
        '  source install/setup.bash')

# pymavlink 2.4.49 add_message bug: a MAVLink1 packet (no instance field)
# stores a message with _instances=None; a later MAVLink2 packet of the same
# type with an instance field then does messages[mtype]._instances[i] = msg
# → TypeError, killing the whole recv path. Guard: drop the stale entry so
# add_message re-runs its init branch. Must be installed before any
# connection is created (ThrusterController import below connects lazily).
from pymavlink import mavutil as _mavutil

_orig_add_message = _mavutil.add_message


def _safe_add_message(messages, mtype, msg):
    stored = messages.get(mtype)
    if (stored is not None
            and getattr(stored, '_instances', None) is None
            and msg._instance_field is not None
            and getattr(msg, msg._instance_field, None) is not None):
        del messages[mtype]
    _orig_add_message(messages, mtype, msg)


_mavutil.add_message = _safe_add_message

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
# Same idea for roll/pitch (6DOF frame): timed fallback when no ATTITUDE
# feedback. CALIBRATE IN WATER — vertical thrusters, different authority.
ROTATE_90_SECONDS = 3.0
ROTATE_SPEED = 0.35          # default roll/pitch effort for DepthKeeper.rotate
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


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


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

    def fresh(self, stale_s=1.0):
        """All labels seen within stale_s: sorted list of (label, conf)."""
        now = time.monotonic()
        return sorted((label, det.confidence)
                      for label, (det, t) in self._latest.items()
                      if now - t <= stale_s)


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


# ─── Bar02 depth + DepthKeeper (ported from depth_field_test.py) ─────────────
#
# The proven field-movement engine: one 'axes' MovementCommand per tick
# (single writer — never two writers per tick), Bar02 closed-loop depth hold
# UNDER every surge/strafe/turn, ZED course hold on straight legs.

class Bar02DepthSource:
    """Background reader on the shared MAVLink master: Bar02 depth + ATTITUDE.

    Reuses depth_hold_bar02_test's pipeline (stream request, baro detection,
    surface latch + sanity check). ATTITUDE rides along for free.
    """

    def __init__(self, master, rho=1000.0):
        import depth_hold_bar02_test as dhb
        self._dhb = dhb
        self.master = master
        # Claim the serial recv path: two threads reading one pyserial port
        # race between select() and read() ("device reports readiness to
        # read but returned no data") and steal each other's messages.
        # ThrusterController sees this flag and switches its armed-status
        # check to passive master.messages reads (pymavlink stashes every
        # message there no matter which thread recv'd it).
        master._external_recv_reader = True
        self.rho = rho
        self.ptype = None
        self.surface_hpa = None
        self._lock = threading.Lock()
        self._d = None
        self._dt = None
        self._att = None          # (roll, pitch, yaw) rad
        self._att_rates = None    # (p, q, r) rad/s
        self._att_t = None
        self._stop = False
        self._thread = None

    def setup(self):
        """Detect baro + latch surface. Returns Bar02 depth limit (m) or None.

        On failure the source can still be start()ed for ATTITUDE-only
        streaming (depth() stays None) — roll/pitch feedback survives a
        Bar02 dropout.
        """
        dhb = self._dhb
        dhb.request_streams(self.master)
        print('Detecting depth/pressure source (shared link)…')
        self.ptype, first = dhb.detect_pressure_source(self.master)
        if self.ptype is None:
            print('No SCALED_PRESSURE/2/3 — check Bar02 wiring / '
                  'BARO_PROBE_EXT=512 / BARO_EXT_BUS=1.')
            self.ptype = 'SCALED_PRESSURE2'   # keep recv filter valid (ATT-only)
            return None
        print(f'Using depth source: {self.ptype} '
              f'(first {first.press_abs:.1f} hPa)')

        print('Latching surface pressure (keep sub at surface, still)…')
        self.surface_hpa = dhb.latch_surface(self.master, self.ptype)
        if self.surface_hpa is None:
            print(f'{self.ptype} stopped streaming during latch — aborting.')
            return None
        if not dhb.surface_sane(self.surface_hpa):
            ratio = self.surface_hpa / 1013.25
            print(f'ABORT: surface {self.surface_hpa:.0f} hPa not atmospheric '
                  f'({ratio:.1f}x). Bad Bar02 scaling — do not trust depth.')
            self.surface_hpa = None           # never compute depth from this
            return None
        print(f'Surface baseline = {self.surface_hpa:.1f} hPa (sane).')
        return ((dhb.BAR02_FULL_SCALE_PA - 101325.0) / (self.rho * dhb.G)
                - dhb.BAR02_MARGIN_M)

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        warned = False
        while not self._stop:
            try:
                msg = self.master.recv_match(type=[self.ptype, 'ATTITUDE'],
                                             blocking=True, timeout=1.0)
            except Exception as e:
                # Serial hiccup (e.g. ttyACM0 "readiness but no data") must
                # not kill the streamer permanently — keep retrying so depth
                # returns when the link recovers.
                if not warned:
                    print(f'[streamer] recv error ({e}) — retrying…')
                    warned = True
                time.sleep(0.2)
                continue
            warned = False
            if msg is None:
                continue
            now = time.monotonic()
            if msg.get_type() == 'ATTITUDE':
                with self._lock:
                    self._att = (msg.roll, msg.pitch, msg.yaw)
                    self._att_rates = (msg.rollspeed, msg.pitchspeed,
                                       msg.yawspeed)
                    self._att_t = now
            elif self.surface_hpa is not None:
                d = ((msg.press_abs - self.surface_hpa) * 100.0
                     / (self.rho * self._dhb.G))
                with self._lock:
                    self._d = d
                    self._dt = now

    def depth(self, stale_s=2.0):
        with self._lock:
            if self._dt is None or time.monotonic() - self._dt > stale_s:
                return None
            return self._d

    def attitude(self, stale_s=2.0):
        """((roll, pitch, yaw) rad, (p, q, r) rad/s) or (None, None)."""
        with self._lock:
            if self._att_t is None or time.monotonic() - self._att_t > stale_s:
                return None, None
            return self._att, self._att_rates

    def stop(self):
        self._stop = True


class DepthKeeper:
    """Single-writer 10 Hz control loop: Bar02 depth hold UNDER moves.

    Publishes exactly one ``axes`` MovementCommand per tick:

      * heave — P law on (target − Bar02 depth), or a manual override for
        open-loop verticals. Always-active P inside the deadband so buoyancy
        is countered even if ALT_HOLD drops to MANUAL (Bar02 dropout).
      * surge / strafe / yaw — linearly ramped setpoints. When no turn is
        commanded, a P law on the attached heading monitor holds the latched
        course so straight legs don't crab (weak motor 6).

    The continuous-move streamer stays idle for the keeper's whole lifetime
    (``start()`` idles it once) — the keeper is the ONLY writer.
    ``src=None`` is tolerated: heave stays neutral / override-only.
    """

    def __init__(self, driver, src, max_depth_m,
                 kp=2.0, min_speed=0.15, max_speed=0.6, deadband=0.07,
                 yaw_kp=1.0, yaw_hold_max=0.25, yaw_hold_sign=1.0):
        self.driver = driver
        self.src = src
        self.max_depth_m = max_depth_m
        self.kp, self.min_speed, self.max_speed = kp, min_speed, max_speed
        self.deadband = deadband
        self.yaw_kp, self.yaw_hold_max = yaw_kp, yaw_hold_max
        self.yaw_hold_sign = yaw_hold_sign
        self._lock = threading.Lock()
        self._target = None          # depth (m) to hold, None → heave 0
        self._override = None        # manual heave, signed +down
        # setpoints: surge, strafe, yaw, pitch, roll (signed). pitch/roll
        # only act on a 6DOF frame (MANUAL_CONTROL extension axes).
        self._sp = [0.0] * 5
        self._cur = [0.0] * 5        # ramped current values
        self._rate = 0.6             # effort change per second toward sp
        self._hold_heading = None    # latched course while yaw sp == 0
        self._stop = False
        self._no_data = 0
        self._thread = None

    # ── lifecycle ──

    def start(self):
        self.driver.idle()           # keeper is the ONLY writer from here
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True

    # ── setpoints ──

    def depth(self, stale_s=2.0):
        return self.src.depth(stale_s) if self.src is not None else None

    def set_target(self, depth_m):
        """Retarget the depth hold (m, +down). None → neutral heave."""
        with self._lock:
            self._target = depth_m
            self._override = None

    def target(self):
        with self._lock:
            return self._target

    def hold_here(self):
        """Re-latch the hold target to the current Bar02 depth."""
        d = self.depth()
        self.set_target(d)
        return d

    def set_move(self, surge=0.0, strafe=0.0, yaw=0.0, pitch=0.0, roll=0.0,
                 ramp=1.0):
        """Move setpoints, signed [-1, 1]. Depth hold continues (except: while
        pitch/roll are commanded, heave goes neutral — body-frame heave points
        the wrong way when the sub is rotated; ALT_HOLD does what it can).
        yaw > 0 → CW/right (thruster_node convention); pitch > 0 → nose up;
        roll > 0 → right-side down."""
        with self._lock:
            self._sp = [clamp(float(surge), -1.0, 1.0),
                        clamp(float(strafe), -1.0, 1.0),
                        clamp(float(yaw), -1.0, 1.0),
                        clamp(float(pitch), -1.0, 1.0),
                        clamp(float(roll), -1.0, 1.0)]
            biggest = max(abs(s - c) for s, c in zip(self._sp, self._cur))
            self._rate = (biggest / ramp) if ramp > 0 else float('inf')
            if self._sp[2] != 0.0 or self._sp[3] != 0.0 or self._sp[4] != 0.0:
                self._hold_heading = None   # intentional rotation — drop latch

    def clear_move(self, ramp=0.5):
        self.set_move(0.0, 0.0, 0.0, ramp=ramp)

    def set_heave_override(self, heave):
        """Manual open-loop heave (signed, +down). None → back to P law."""
        with self._lock:
            self._override = heave

    # ── blocking turn (port of depth_field_test.keeper_turn) ──

    def turn(self, direction, degrees, speed=TURN_SPEED):
        """Closed-loop turn THROUGH the keeper — depth hold never stops.

        direction: +1 → CW/right, -1 → CCW/left. Heading feedback: ZED
        monitor first; if the fix is gone (VSLAM often loses tracking after
        style rolls/loops), the Pixhawk gyro yaw rate (ATTITUDE yawspeed)
        is integrated instead — orientation-proof; timed fallback only when
        both are missing. Blocks.
        """
        degrees = abs(float(degrees))
        if degrees == 0:
            return True
        target = math.radians(degrees)
        est = (degrees / 90.0) * TURN_90_SECONDS
        timeout = max(5.0, 3.0 * est)

        mon = getattr(self.driver, '_heading_mon', None)
        h_prev = mon.heading() if mon is not None else None
        use_zed = h_prev is not None

        def gyro():
            return self.src.attitude()[1] if self.src is not None else None

        use_gyro = not use_zed and gyro() is not None
        closed_loop = use_zed or use_gyro
        if use_gyro:
            print('turn: no ZED heading — using Pixhawk gyro yaw feedback.')
        elif not closed_loop:
            print(f'turn: no heading fix — timed fallback '
                  f'({est:.1f}s for {degrees:.0f}°). CALIBRATE TURN_90_SECONDS.')

        self.set_move(yaw=direction * speed, ramp=0.5)
        turned = 0.0
        t_prev = time.monotonic()
        t0 = time.time()
        try:
            while time.time() - t0 < (timeout if closed_loop else est):
                time.sleep(1.0 / RATE_HZ)
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                if use_zed:
                    h = mon.heading()
                    if h is None:
                        remain = max(0.0, 1.0 - turned / target)
                        if gyro() is not None:
                            print(f'turn: ZED heading lost at '
                                  f'{math.degrees(turned):.0f}° — switching '
                                  f'to gyro yaw feedback')
                            use_zed = False
                            use_gyro = True
                            continue
                        print(f'turn: heading lost at '
                              f'{math.degrees(turned):.0f}° — timed remainder '
                              f'{remain * est:.1f}s')
                        time.sleep(remain * est)
                        closed_loop = False
                        break
                    turned += abs(wrap(h - h_prev))
                    h_prev = h
                elif use_gyro:
                    rates = gyro()
                    if rates is None:
                        remain = max(0.0, 1.0 - turned / target)
                        print(f'turn: ATTITUDE lost at '
                              f'{math.degrees(turned):.0f}° — timed remainder '
                              f'{remain * est:.1f}s')
                        time.sleep(remain * est)
                        closed_loop = False
                        break
                    turned += abs(rates[2]) * dt
                if closed_loop and turned >= target:
                    break
            else:
                if closed_loop:
                    print(f'turn: timeout at {math.degrees(turned):.0f}° of '
                          f'{degrees:.0f}°')
        finally:
            self.clear_move(ramp=0.3)
        if closed_loop:
            print(f'✓ turned {math.degrees(turned):.0f}° (depth held)')
        return closed_loop

    # ── blocking roll/pitch rotation (6DOF frame — style points) ──

    def rotate(self, axis, direction, degrees, speed=ROTATE_SPEED):
        """Closed-loop roll or pitch rotation on the Pixhawk gyro stream.

        axis: 'roll' or 'pitch'. direction: +1 → roll right-side-down /
        pitch nose-up, -1 → the opposite. Rotation is measured by
        integrating the BODY GYRO RATE (ATTITUDE rollspeed/pitchspeed) —
        orientation-proof, unlike Euler angles, which glitch at ±90° pitch
        (gimbal lock flips roll/yaw by 180°) and wrap at ±180° roll while
        inverted. Timed fallback without ATTITUDE. Heave is neutral for the
        whole rotation (see set_move); expect some depth excursion. Blocks.

        NOTE: the caller must put the Pixhawk in MANUAL/ACRO first —
        ALT_HOLD and STABILIZE self-level and fight the spin (2026-07-10
        run: roll stalled at 31° while the tilted depth thrusters shoved
        the sub sideways). See gate_task.style_spin.

        Returns True only if the full angle was gyro-verified.
        """
        assert axis in ('roll', 'pitch')
        degrees = abs(float(degrees))
        if degrees == 0:
            return True
        idx = 0 if axis == 'roll' else 1        # index into (p, q, r)
        target = math.radians(degrees)
        est = (degrees / 90.0) * ROTATE_90_SECONDS
        timeout = max(6.0, 3.0 * est)

        def gyro():
            return self.src.attitude()[1] if self.src is not None else None

        closed_loop = gyro() is not None
        if not closed_loop:
            print(f'{axis}: no ATTITUDE feedback — timed fallback '
                  f'({est:.1f}s for {degrees:.0f}°). CALIBRATE '
                  f'ROTATE_90_SECONDS.')

        kw = {axis: direction * speed}
        self.set_move(ramp=0.5, **kw)
        rotated = 0.0
        ok = not closed_loop      # timed fallback is "done" but unverified
        t_prev = time.monotonic()
        t0 = time.time()
        try:
            while time.time() - t0 < (timeout if closed_loop else est):
                time.sleep(1.0 / RATE_HZ)
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                if closed_loop:
                    rates = gyro()
                    if rates is None:
                        remain = max(0.0, 1.0 - rotated / target)
                        print(f'{axis}: ATTITUDE lost at '
                              f'{math.degrees(rotated):.0f}° — timed '
                              f'remainder {remain * est:.1f}s')
                        time.sleep(remain * est)
                        closed_loop = False
                        break
                    rotated += abs(rates[idx]) * dt
                    if rotated >= target:
                        ok = True
                        break
            else:
                if closed_loop:
                    print(f'{axis}: timeout at {math.degrees(rotated):.0f}° '
                          f'of {degrees:.0f}°')
        finally:
            self.clear_move(ramp=0.3)
        if closed_loop:
            mark = '✓' if ok else '✗'
            print(f'{mark} {axis}ed {math.degrees(rotated):.0f}° '
                  f'of {degrees:.0f}° (gyro-verified)')
        return closed_loop and ok

    # ── control loop ──

    def _send(self, surge, strafe, heave, yaw, pitch=0.0, roll=0.0):
        msg = MovementCommand()
        msg.command = 'axes'
        msg.speed = 0.0
        msg.duration = 0.0
        msg.surge = float(surge)
        msg.strafe = float(strafe)
        msg.heave = float(heave)
        msg.yaw_rate = float(yaw)
        msg.pitch_rate = float(pitch)
        msg.roll_rate = float(roll)
        self.driver.pub.publish(msg)

    def _loop(self):
        period = 1.0 / RATE_HZ
        while not self._stop:
            with self._lock:
                target = self._target
                override = self._override
                step = self._rate * period
                for i in range(5):
                    c, s = self._cur[i], self._sp[i]
                    if c < s:
                        self._cur[i] = min(s, c + step)
                    elif c > s:
                        self._cur[i] = max(s, c - step)
                surge, strafe, yaw, pitch, roll = self._cur
                yaw_sp = self._sp[2]
                rotating = (self._sp[3] != 0.0 or self._sp[4] != 0.0
                            or pitch != 0.0 or roll != 0.0)
                hold_heading = self._hold_heading

            # Course hold: no rotation commanded (setpoints zero, ramps done)
            # → steer back to the latched heading so legs run straight.
            if yaw_sp == 0.0 and yaw == 0.0 and not rotating:
                mon = getattr(self.driver, '_heading_mon', None)
                h = mon.heading() if mon is not None else None
                if h is not None:
                    if hold_heading is None:
                        with self._lock:
                            self._hold_heading = h
                    else:
                        err = wrap(h - hold_heading)
                        yaw = clamp(self.yaw_hold_sign * self.yaw_kp * err,
                                    -self.yaw_hold_max, self.yaw_hold_max)

            depth = self.depth()
            heave = 0.0
            if rotating:
                # Rolled/pitched: body-frame heave no longer points down —
                # a depth correction would shove the sub sideways. Neutral
                # heave; ALT_HOLD's own controller rides through the spin.
                pass
            elif override is not None:
                heave = clamp(override, -1.0, 1.0)
            elif target is not None:
                if depth is None:
                    self._no_data += 1
                    if self._no_data == RATE_HZ * 3:
                        self.driver.get_logger().warn(
                            'DepthKeeper: no Bar02 depth for 3s — heave '
                            'neutral (ALT_HOLD only) until data returns.')
                else:
                    self._no_data = 0
                    error = target - depth          # +ve → need deeper
                    # Always-active P: inside the deadband a small effort
                    # still counters buoyancy (never trust ALT_HOLD alone —
                    # Bar02 dropout forces MANUAL and neutral heave floats
                    # up). min_speed floor only outside the deadband so the
                    # hold doesn't buzz around zero error.
                    effort = clamp(self.kp * abs(error), 0.0, self.max_speed)
                    if abs(error) > self.deadband:
                        effort = max(effort, self.min_speed)
                    heave = effort if error > 0 else -effort

            # Safety envelope beats everything, including overrides.
            if depth is not None and (depth > self.max_depth_m
                                      or depth < -1.0):
                with self._lock:
                    self._sp = [0.0] * 5
                    self._cur = [0.0] * 5
                    self._override = None
                    self._target = 0.0
                surge = strafe = yaw = pitch = roll = 0.0
                heave = -self.max_speed
                self.driver.get_logger().error(
                    f'ABORT: depth {depth:.2f} m outside envelope '
                    f'(max {self.max_depth_m:.2f} m) — surfacing.')

            self._send(surge, strafe, heave, yaw, pitch, roll)
            time.sleep(period)


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


def spawn_vision_factory(monitor_extra=None, model_onnx=None,
                         conf_thres=None, save_frames=None):
    """extra_factory that spawns the TensorRT detector in-process + a monitor.

    Starts the detector's ZED-capture + inference loop (``run_detector``) in a
    daemon thread — without it VisionNode never publishes a detection.
    ``model_onnx`` overrides the forward-camera model path (default:
    vision/ffc_rs_26.onnx next to detector.py). ``conf_thres`` overrides the
    detector's publish gate (its default 0.4 — detections below it never
    reach vision/detections).

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
        if conf_thres is not None:
            argv += ['--conf_thres', str(conf_thres)]
        if save_frames:
            argv += ['--save_frames', save_frames]
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
