#!/usr/bin/env python3
"""Field-movement test: continuous Bar02 depth hold UNDER moves + IMU.

Combines three engines:

  * ``field_common`` — production ``ThrusterController`` session (ALT_HOLD,
    10 Hz, heartbeat, watchdog, arm/disarm on exit). Owns the Pixhawk serial
    port.

  * ``DepthKeeper`` — THE single writer while active. One ``axes``
    MovementCommand per tick (10 Hz): heave from a P law on Bar02 depth
    error, surge/strafe/yaw from the current move setpoints. So the sub
    actively holds the target depth WHILE surging/strafing/TURNING, and
    ``depth`` retargets on the fly. Turns run through the keeper too
    (closed-loop on the ZED IMU heading) — every move happens at the held
    depth. (v1 bug lesson: never two writers per tick — the continuous-move
    streamer stays idle whenever the keeper runs.)

  * ``depth_hold_bar02_test`` — Bar02 pressure pipeline: request streams,
    pick the external SCALED_PRESSURE2 baro, latch a sane surface baseline,
    pressure → depth.

  * ZED 2i front-camera IMU — opened directly via pyzed (IMU only). Absolute
    orientation + angular velocity; also the driver's heading source, so
    turns are closed-loop even without vslam. Pixhawk ATTITUDE read off the
    shared MAVLink link for comparison.

Modes:
    python3 depth_field_test.py --manual           # interactive REPL
    python3 depth_field_test.py --depth 3          # auto: dive, hold, leg, surface
    python3 depth_field_test.py --depth 3 --no-demo  # auto: depth only

Manual commands (one per line):
    f/b/sl/sr [speed] [secs]   surge fwd/back, strafe left/right (0.3, 2s)
                               — DEPTH IS HELD closed-loop during the move
    u/d [speed] [secs]         open-loop up/down, then re-hold at new depth
    tl/tr [deg]                turn left/right, ZED-IMU closed loop (90)
    depth <ft>                 retarget closed-loop Bar02 depth (feet)
    surface                    closed-loop climb to surface
    hold [secs]                keep holding depth, print telemetry (2s)
    imu / i                    print depth + ZED IMU + Pixhawk attitude
    stop / s                   zero moves, re-hold at current depth
    quit / q                   surface NOT automatic — quit disarms where you are

⚠ SHARED SERIAL: Bar02/ATTITUDE reader pulls messages off the SAME
``thrusters.master`` link the ThrusterController writes on. One serial owner.
⚠ CAMERA: this opens the ZED itself — do NOT run vslam_node simultaneously.
SAFETY: ARMS the Pixhawk, drives REAL thrusters. Stop thruster_node first.
Clear props, tether, kill switch reachable. Ctrl+C → stop + disarm.

Requires sourced workspace:
    source /opt/ros/humble/setup.bash && source install/setup.bash
"""

import argparse
import math
import shlex
import threading
import time

import rclpy.logging

import field_common as fc
import depth_hold_bar02_test as dhb

try:
    import pyzed.sl as sl
except Exception:
    sl = None


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ─── ZED 2i front-camera IMU ──────────────────────────────────────────────────

class ZedImu:
    """Front ZED IMU: absolute orientation + angular velocity, no vslam.

    Opens the camera with DEPTH_MODE.NONE (IMU only — no GPU depth work) and
    polls ``get_sensors_data`` on a background thread. Exposes:

      * ``heading()`` — yaw (rad) about the Y-up axis, duck-type compatible
        with field_common's HeadingMonitor so ``driver.attach_heading(zed_imu)``
        makes turn_left/right closed-loop on this IMU.
      * ``euler()`` — (rx, ry, rz) rad: pitch(about X), yaw(about Y),
        roll(about Z) in the Y-up camera frame.
      * ``gyro()`` — angular velocity (deg/s, [x, y, z]).
    """

    def __init__(self, fps=30):
        self.available = sl is not None
        self._lock = threading.Lock()
        self._q = None            # (qx, qy, qz, qw)
        self._av = None           # angular velocity deg/s [x, y, z]
        self._t = None
        self._stop = False
        self._thread = None
        self.fps = fps

    def start(self, timeout=8.0):
        if not self.available:
            return False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._t is not None:
                    return True
            time.sleep(0.1)
        return False

    def stop(self):
        self._stop = True

    def _loop(self):
        cam = None
        try:
            init = sl.InitParameters()
            init.coordinate_units = sl.UNIT.METER
            init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            init.camera_fps = self.fps
            init.depth_mode = sl.DEPTH_MODE.NONE      # IMU only — cheap
            cam = sl.Camera()
            if cam.open(init) != sl.ERROR_CODE.SUCCESS:
                print('ZedImu: camera open failed — IMU telemetry OFF, '
                      'turns fall back to timed.')
                self.available = False
                return
            sensors = sl.SensorsData()
            while not self._stop:
                if cam.get_sensors_data(
                        sensors, sl.TIME_REFERENCE.CURRENT) \
                        == sl.ERROR_CODE.SUCCESS:
                    imu = sensors.get_imu_data()
                    o = imu.get_pose().get_orientation().get()  # qx qy qz qw
                    av = imu.get_angular_velocity()
                    with self._lock:
                        self._q = (float(o[0]), float(o[1]),
                                   float(o[2]), float(o[3]))
                        self._av = (float(av[0]), float(av[1]), float(av[2]))
                        self._t = time.monotonic()
                time.sleep(0.02)                       # ~50 Hz poll
        except Exception as e:
            print(f'ZedImu: loop error: {e}')
            self.available = False
        finally:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass

    def _fresh(self, stale_s):
        return self._t is not None and time.monotonic() - self._t <= stale_s

    def heading(self, stale_s=1.0):
        """Yaw (rad) about Y-up, or None. HeadingMonitor-compatible."""
        with self._lock:
            if not self._fresh(stale_s) or self._q is None:
                return None
            x, y, z, w = self._q
        return fc.heading_about_axis(x, y, z, w, axis='y')

    def euler(self, stale_s=1.0):
        """(rx, ry, rz) rad — rotations about X (pitch), Y (yaw), Z (roll)
        in the Y-up frame — or None."""
        with self._lock:
            if not self._fresh(stale_s) or self._q is None:
                return None
            x, y, z, w = self._q
        rx = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sy = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
        ry = math.asin(sy)
        rz = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return rx, ry, rz

    def gyro(self, stale_s=1.0):
        """Angular velocity deg/s [x, y, z], or None."""
        with self._lock:
            return self._av if self._fresh(stale_s) else None


# ─── Bar02 depth + Pixhawk ATTITUDE off the shared MAVLink link ───────────────

class Bar02DepthSource:
    """Background reader on the shared master: Bar02 depth + Pixhawk ATTITUDE.

    Reuses depth_hold_bar02_test's pipeline (stream request, baro detection,
    surface latch + sanity check). ATTITUDE rides along for free — it is the
    Pixhawk's own IMU/EKF orientation, printed next to the ZED IMU so compass
    /EKF drift is visible at a glance.
    """

    def __init__(self, master, rho):
        self.master = master
        self.rho = rho
        self.ptype = None
        self.surface_hpa = None
        self._lock = threading.Lock()
        self._d = None
        self._dt = None
        self._att = None          # (roll, pitch, yaw) rad
        self._att_rates = None    # (rollspeed, pitchspeed, yawspeed) rad/s
        self._att_t = None
        self._stop = False
        self._thread = None

    def setup(self):
        """Detect baro + latch surface. Returns Bar02 depth limit (m) or None."""
        dhb.request_streams(self.master)
        print('Detecting depth/pressure source (shared link)…')
        self.ptype, first = dhb.detect_pressure_source(self.master)
        if self.ptype is None:
            print('No SCALED_PRESSURE/2/3 — check Bar02 wiring / '
                  'BARO_PROBE_EXT=512 / BARO_EXT_BUS=1. Aborting depth.')
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
            return None
        print(f'Surface baseline = {self.surface_hpa:.1f} hPa (sane).')
        return ((dhb.BAR02_FULL_SCALE_PA - 101325.0) / (self.rho * dhb.G)
                - dhb.BAR02_MARGIN_M)

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop:
            msg = self.master.recv_match(type=[self.ptype, 'ATTITUDE'],
                                         blocking=True, timeout=1.0)
            if msg is None:
                continue
            now = time.monotonic()
            if msg.get_type() == 'ATTITUDE':
                with self._lock:
                    self._att = (msg.roll, msg.pitch, msg.yaw)
                    self._att_rates = (msg.rollspeed, msg.pitchspeed,
                                       msg.yawspeed)
                    self._att_t = now
            else:
                d = ((msg.press_abs - self.surface_hpa) * 100.0
                     / (self.rho * dhb.G))
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


# ─── Telemetry line ───────────────────────────────────────────────────────────

def telemetry_line(src, zed):
    """One status line: Bar02 depth, ZED IMU orientation + gyro, Pixhawk att."""
    d = src.depth() if src else None
    parts = [f'depth={d:5.2f} m' if d is not None else 'depth=  n/a']
    e = zed.euler() if zed else None
    g = zed.gyro() if zed else None
    if e is not None:
        rx, ry, rz = (math.degrees(a) for a in e)
        parts.append(f'zed rpy=({rz:+6.1f},{rx:+6.1f},{ry:+7.1f})°')
    else:
        parts.append('zed imu=n/a')
    if g is not None:
        parts.append(f'gyro=({g[0]:+6.1f},{g[1]:+6.1f},{g[2]:+6.1f})°/s')
    att, rates = src.attitude() if src else (None, None)
    if att is not None:
        r, p, y = (math.degrees(a) for a in att)
        parts.append(f'pix rpy=({r:+6.1f},{p:+6.1f},{y:+7.1f})°')
    return '  '.join(parts)


# ─── DepthKeeper: depth P-hold + horizontal moves, ONE writer ────────────────

class DepthKeeper:
    """Single-writer 10 Hz control loop: Bar02 depth hold UNDER moves.

    Publishes exactly one ``axes`` MovementCommand per tick:

      * heave — P law on (target − Bar02 depth): submerge/emerge effort
        outside the deadband, 0 (ALT_HOLD assists) inside it. Or a manual
        override for open-loop u/d.
      * surge / strafe / yaw — current move setpoints, linearly ramped.
        Whenever the commanded yaw is zero, a P law on the ZED heading
        holds the latched course instead — so surge/strafe legs no longer
        crab off at an angle when one thruster is weak (motor 6). An
        intentional turn (nonzero yaw setpoint) clears the latch; the new
        heading is re-latched once the turn's yaw ramps back to zero.

    So forward/back/strafe/turn all run WHILE the depth loop keeps
    correcting — the thing the old design could not do (streamer + depth
    loop = two writers). The continuous-move streamer stays idle for the
    keeper's whole lifetime (``start()`` idles it once).
    """

    def __init__(self, driver, src, args, max_depth_m):
        self.driver = driver
        self.src = src
        self.args = args
        self.max_depth_m = max_depth_m
        self._lock = threading.Lock()
        self._target = None          # depth (m) to hold, None → heave 0
        self._override = None        # manual heave, signed +down (u/d cmds)
        self._sp = [0.0, 0.0, 0.0]   # surge, strafe, yaw setpoints (signed)
        self._cur = [0.0, 0.0, 0.0]  # ramped current values
        self._rate = 0.6             # effort change per second toward sp
        self._hold_heading = None    # latched ZED course while yaw sp == 0
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
        d = self.src.depth()
        self.set_target(d)
        return d

    def set_move(self, surge=0.0, strafe=0.0, yaw=0.0, ramp=1.0):
        """Horizontal move setpoints, signed [-1, 1]. Depth hold continues."""
        with self._lock:
            self._sp = [clamp(float(surge), -1.0, 1.0),
                        clamp(float(strafe), -1.0, 1.0),
                        clamp(float(yaw), -1.0, 1.0)]
            biggest = max(abs(s - c) for s, c in zip(self._sp, self._cur))
            self._rate = (biggest / ramp) if ramp > 0 else float('inf')
            if self._sp[2] != 0.0:
                self._hold_heading = None   # intentional turn — drop the latch

    def clear_move(self, ramp=0.5):
        self.set_move(0.0, 0.0, 0.0, ramp=ramp)

    def set_heave_override(self, heave):
        """Manual open-loop heave (signed, +down). None → back to P law."""
        with self._lock:
            self._override = heave

    # ── control loop ──

    def _send(self, surge, strafe, heave, yaw):
        msg = fc.MovementCommand()
        msg.command = 'axes'
        msg.speed = 0.0
        msg.duration = 0.0
        msg.surge = float(surge)
        msg.strafe = float(strafe)
        msg.heave = float(heave)
        msg.yaw_rate = float(yaw)
        self.driver.pub.publish(msg)

    def _loop(self):
        period = 1.0 / fc.RATE_HZ
        while not self._stop:
            with self._lock:
                target = self._target
                override = self._override
                step = self._rate * period
                for i in range(3):
                    c, s = self._cur[i], self._sp[i]
                    if c < s:
                        self._cur[i] = min(s, c + step)
                    elif c > s:
                        self._cur[i] = max(s, c - step)
                surge, strafe, yaw = self._cur
                yaw_sp = self._sp[2]
                hold_heading = self._hold_heading

            # Course hold: whenever no turn is commanded (setpoint zero and
            # the ramp has finished), steer back to the latched ZED heading
            # so surge/strafe legs run straight despite asymmetric thrust.
            if yaw_sp == 0.0 and yaw == 0.0:
                mon = getattr(self.driver, '_heading_mon', None)
                h = mon.heading() if mon is not None else None
                if h is not None:
                    if hold_heading is None:
                        with self._lock:
                            self._hold_heading = h
                    else:
                        err = fc.wrap(h - hold_heading)
                        yaw = clamp(
                            self.args.yaw_hold_sign * self.args.yaw_kp * err,
                            -self.args.yaw_hold_max, self.args.yaw_hold_max)

            depth = self.src.depth()
            heave = 0.0
            if override is not None:
                heave = clamp(override, -1.0, 1.0)
            elif target is not None:
                if depth is None:
                    self._no_data += 1
                    if self._no_data == fc.RATE_HZ * 3:
                        self.driver.get_logger().warn(
                            'DepthKeeper: no Bar02 depth for 3s — heave '
                            'neutral (ALT_HOLD only) until data returns.')
                else:
                    self._no_data = 0
                    error = target - depth          # +ve → need deeper
                    # Always-active P: inside the deadband a small
                    # proportional effort still counters buoyancy (never
                    # trust ALT_HOLD alone — Bar02 dropout forces MANUAL
                    # and neutral heave lets the sub float up). The
                    # min_speed floor only applies outside the deadband so
                    # the hold doesn't buzz around zero error.
                    effort = clamp(self.args.kp * abs(error),
                                   0.0, self.args.max_speed)
                    if abs(error) > self.args.deadband:
                        effort = max(effort, self.args.min_speed)
                    heave = effort if error > 0 else -effort

            # Safety envelope beats everything, including overrides.
            if depth is not None and (depth > self.max_depth_m
                                      or depth < -1.0):
                with self._lock:
                    self._sp = [0.0, 0.0, 0.0]
                    self._cur = [0.0, 0.0, 0.0]
                    self._override = None
                    self._target = 0.0
                surge = strafe = yaw = 0.0
                heave = -self.args.max_speed
                self.driver.get_logger().error(
                    f'ABORT: depth {depth:.2f} m outside envelope '
                    f'(max {self.max_depth_m:.2f} m) — surfacing.')

            self._send(surge, strafe, heave, yaw)
            time.sleep(period)


def wait_for_depth(keeper, src, zed, target_m, args, hold_s, timeout=120.0):
    """Block (printing 1 Hz telemetry) until the keeper reaches target and
    holds it CONTINUOUSLY for hold_s — leaving settle_tol resets the clock,
    so one noisy sample near the threshold can't fake a completed hold.
    The keeper does the driving. Returns True on hold."""
    t0 = time.monotonic()
    reached_at = None
    last_print = 0.0
    while time.monotonic() - t0 < timeout:
        depth = src.depth()
        error = None if depth is None else target_m - depth
        if error is not None and abs(error) <= args.settle_tol:
            if reached_at is None:
                reached_at = time.monotonic()
                print(f'✓ Reached {depth:.2f} m — holding {hold_s:.0f}s.')
        elif reached_at is not None:
            held = time.monotonic() - reached_at
            err_s = f'{error:+.2f} m' if error is not None else 'no data'
            print(f'✗ Left settle band after {held:.1f}s (err {err_s}) '
                  f'— hold clock reset.')
            reached_at = None
        if reached_at is not None \
                and time.monotonic() - reached_at >= hold_s:
            return True
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            if error is None:
                state, err_s = 'NO-DATA', '  n/a'
            else:
                state = ('HOLD' if abs(error) <= args.deadband
                         else 'DESCEND' if error > 0 else 'ASCEND')
                err_s = f'{error:+.2f}'
            print(f'[{state}] tgt={target_m:.2f} err={err_s}  '
                  f'{telemetry_line(src, zed)}')
        time.sleep(0.2)
    print(f'wait_for_depth: timeout after {timeout:.0f}s '
          f'(target {target_m:.2f} m) — keeper keeps holding.')
    return False


def surface(keeper, src, args, timeout=30.0):
    """Retarget to surface, wait until inside the deadband."""
    print('Surfacing…')
    keeper.clear_move()
    keeper.set_target(0.0)
    deadline = time.time() + timeout
    last_print = 0.0
    while time.time() < deadline:
        depth = src.depth()
        if depth is not None and depth <= args.deadband:
            break
        if time.time() - last_print >= 1.0:
            last_print = time.time()
            d = f'{depth:.2f}' if depth is not None else 'n/a'
            print(f'[SURFACE] depth={d} m')
        time.sleep(0.2)


# ─── Manual REPL (depth held continuously by the keeper) ─────────────────────

MANUAL_HELP = """\
Commands (depth target is HELD closed-loop under every move):
  f/b/sl/sr [speed] [secs]   surge fwd/back, strafe left/right (def 0.3, 2s)
  u/d [speed] [secs]         open-loop up/down, then re-hold at new depth
  tl/tr [deg]                turn left/right, ZED-IMU closed loop (def 90)
  depth <ft>                 retarget depth hold (feet)
  surface                    closed-loop climb to surface
  hold [secs]                keep holding depth, telemetry each second (def 2)
  imu / i                    print depth + ZED IMU + Pixhawk attitude
  stop / s                   zero moves, re-hold at current depth
  help / h                   this help
  quit / q                   exit (stop + disarm; does NOT surface first)"""

# cmd -> (surge_sign, strafe_sign)
MOVE_AXES = {'f': (1, 0), 'b': (-1, 0), 'sl': (0, -1), 'sr': (0, 1)}


def keeper_move(keeper, surge_sign, strafe_sign, speed, secs):
    """Timed horizontal move THROUGH the keeper — depth hold never stops."""
    keeper.set_move(surge=surge_sign * speed, strafe=strafe_sign * speed,
                    ramp=min(1.0, secs / 2.0))
    try:
        time.sleep(secs)
    finally:
        keeper.clear_move()


def keeper_vertical(keeper, heave, secs):
    """Timed open-loop vertical, then re-latch the hold at the new depth."""
    keeper.set_heave_override(heave)
    try:
        time.sleep(secs)
    finally:
        keeper.set_heave_override(None)
        d = keeper.hold_here()
        if d is not None:
            print(f're-holding at {d:.2f} m')
        else:
            print('no Bar02 depth — heave neutral (ALT_HOLD only).')


def keeper_turn(keeper, driver, direction, degrees):
    """Closed-loop turn THROUGH the keeper — depth hold never stops.

    direction: +1 → CW/right, -1 → CCW/left (thruster_node: rotate_cw is
    positive yaw_rate). Yaw rides on the keeper's per-tick command so the
    Bar02 depth P law keeps correcting for the whole turn — the old design
    paused the keeper and handed the port to driver.turn_*, which yawed
    with neutral heave and let the sub drift up. Heading feedback comes
    from the driver's attached monitor (ZED IMU); timed fallback without
    a fix, same integration scheme as field_common._turn.
    """
    degrees = abs(float(degrees))
    if degrees == 0:
        return True
    target = math.radians(degrees)
    est = (degrees / 90.0) * fc.TURN_90_SECONDS
    timeout = max(5.0, 3.0 * est)

    mon = getattr(driver, '_heading_mon', None)
    h_prev = mon.heading() if mon is not None else None
    closed_loop = h_prev is not None
    if not closed_loop:
        print(f'turn: no ZED heading fix — timed fallback '
              f'({est:.1f}s for {degrees:.0f}°). CALIBRATE TURN_90_SECONDS.')

    keeper.set_move(yaw=direction * fc.TURN_SPEED, ramp=0.5)
    turned = 0.0
    t0 = time.time()
    try:
        while time.time() - t0 < (timeout if closed_loop else est):
            if closed_loop:
                h = mon.heading()
                if h is None:
                    remain = max(0.0, 1.0 - turned / target)
                    print(f'turn: heading lost at {math.degrees(turned):.0f}°'
                          f' — timed remainder {remain * est:.1f}s')
                    time.sleep(remain * est)
                    closed_loop = False
                    break
                turned += abs(fc.wrap(h - h_prev))
                h_prev = h
                if turned >= target:
                    break
            time.sleep(1.0 / fc.RATE_HZ)
        else:
            if closed_loop:
                print(f'turn: timeout at {math.degrees(turned):.0f}° of '
                      f'{degrees:.0f}°')
    finally:
        keeper.clear_move(ramp=0.3)
    if closed_loop:
        print(f'✓ turned {math.degrees(turned):.0f}° (depth held)')
    return closed_loop


def manual_loop(driver, keeper, src, zed, args, bar02_limit_m):
    # Silence the ThrusterController's periodic INFO logs ("MAVLink TX: …"
    # every 2 s) — they bury the prompt and make the REPL look unresponsive.
    # WARN and above still print.
    rclpy.logging.set_logger_level(
        'thruster_controller', rclpy.logging.LoggingSeverity.WARN)
    print('\nMANUAL MODE — depth target held closed-loop UNDER every move.')
    print('No target yet: heave neutral (ALT_HOLD). Set one with: depth <ft>')
    print(MANUAL_HELP)
    print('\nType a command, then ENTER.')
    while True:
        try:
            line = input('cmd> ').strip().lower()
        except EOFError:
            break
        if not line:
            continue
        try:
            toks = shlex.split(line)
        except ValueError:
            print('parse error')
            continue
        cmd, rest = toks[0], toks[1:]

        def fnum(i, default):
            try:
                return float(rest[i])
            except (IndexError, ValueError):
                return default

        try:
            if cmd in ('q', 'quit'):
                break
            elif cmd in ('h', 'help'):
                print(MANUAL_HELP)
            elif cmd in ('i', 'imu'):
                t = keeper.target()
                tgt = f'{t:.2f} m' if t is not None else 'none'
                print(f'hold tgt={tgt}  {telemetry_line(src, zed)}')
            elif cmd in ('s', 'stop'):
                keeper.clear_move()
                keeper.set_heave_override(None)
                d = keeper.hold_here()
                print(f're-holding at {d:.2f} m' if d is not None
                      else 'no Bar02 depth — heave neutral.')
            elif cmd in MOVE_AXES:
                speed = clamp(fnum(0, 0.3), 0.0, 1.0)
                secs = max(0.0, fnum(1, 2.0))
                print(f'{cmd}: speed={speed:.2f} {secs:.1f}s (depth held)')
                keeper_move(keeper, *MOVE_AXES[cmd], speed, secs)
            elif cmd in ('u', 'd'):
                speed = clamp(fnum(0, 0.3), 0.0, 1.0)
                secs = max(0.0, fnum(1, 2.0))
                heave = speed if cmd == 'd' else -speed
                print(f'{cmd}: speed={speed:.2f} {secs:.1f}s (open loop)')
                keeper_vertical(keeper, heave, secs)
            elif cmd == 'tl':
                keeper_turn(keeper, driver, -1, fnum(0, 90.0))
            elif cmd == 'tr':
                keeper_turn(keeper, driver, +1, fnum(0, 90.0))
            elif cmd == 'hold':
                secs = max(0.0, fnum(0, 2.0))
                end = time.time() + secs
                while time.time() < end:
                    print(telemetry_line(src, zed))
                    time.sleep(min(1.0, max(0.0, end - time.time())))
            elif cmd == 'depth':
                if not rest:
                    print('usage: depth <feet>')
                    continue
                tgt = fnum(0, -1.0) * fc.FEET_TO_M
                if not 0 < tgt <= bar02_limit_m:
                    print(f'depth must be 0–{bar02_limit_m / fc.FEET_TO_M:.0f} ft')
                    continue
                keeper.set_target(tgt)
                wait_for_depth(keeper, src, zed, tgt, args, hold_s=0.0)
            elif cmd == 'surface':
                surface(keeper, src, args)
            else:
                print(f'unknown: {cmd} — h for help')
        except KeyboardInterrupt:
            print('\n^C — stopping move, re-holding depth.')
            keeper.clear_move()
            keeper.set_heave_override(None)
            keeper.hold_here()
    keeper.clear_move()


# ─── Auto demo leg (moves through the keeper — depth held throughout) ────────

def run_movement_leg(driver, keeper, args):
    print('Movement leg (depth held throughout): '
          'fwd → back → strafe L → strafe R → turn L 90 → turn R 90')
    steps = [
        ('forward',      lambda: keeper_move(keeper,  1,  0, args.speed, args.leg)),
        ('backward',     lambda: keeper_move(keeper, -1,  0, args.speed, args.leg)),
        ('strafe left',  lambda: keeper_move(keeper,  0, -1, args.speed, args.leg)),
        ('strafe right', lambda: keeper_move(keeper,  0,  1, args.speed, args.leg)),
        ('turn left 90', lambda: keeper_turn(keeper, driver, -1, 90)),
        ('turn right 90', lambda: keeper_turn(keeper, driver, +1, 90)),
    ]
    for name, step in steps:
        print(f'>> {name}')
        step()
        time.sleep(args.pause)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--manual', action='store_true',
                    help='interactive REPL (skips auto run)')
    ap.add_argument('--depth', type=float, default=3.0,
                    help='auto mode: target depth in FEET (default 3.0)')
    ap.add_argument('--hold-duration', type=float, default=15.0,
                    help='auto mode: seconds to hold at depth (default 15)')
    ap.add_argument('--kp', type=float, default=2.0,
                    help='depth P gain: effort fraction per metre error')
    ap.add_argument('--min-speed', type=float, default=0.15,
                    help='min vertical effort while moving (0-1)')
    ap.add_argument('--max-speed', type=float, default=0.6,
                    help='max vertical effort (0-1)')
    ap.add_argument('--deadband', type=float, default=0.07,
                    help='half-width (m) of neutral hold band')
    ap.add_argument('--settle-tol', type=float, default=0.1,
                    help='error (m) under which target counts reached')
    ap.add_argument('--yaw-kp', type=float, default=1.0,
                    help='course-hold P gain: yaw effort per rad of ZED '
                         'heading error while no turn is commanded')
    ap.add_argument('--yaw-hold-max', type=float, default=0.25,
                    help='cap on course-hold yaw effort (0-1); 0 disables')
    ap.add_argument('--yaw-hold-sign', type=float, default=1.0,
                    choices=[1.0, -1.0],
                    help='flip to -1 if course hold DIVERGES (spins away '
                         'from the latched heading) — depends on ZED mount')
    ap.add_argument('--max-depth', type=float, default=0.0,
                    help='abort+surface past this depth (m). '
                         '0 → 2x target (auto) / Bar02 limit (manual)')
    ap.add_argument('--water-density', type=float, default=1000.0,
                    help='kg/m^3 (fresh ~1000, salt ~1025)')
    ap.add_argument('--no-demo', action='store_true',
                    help='auto mode: depth only, skip movement leg')
    ap.add_argument('--no-zed', action='store_true',
                    help='skip ZED IMU (turns fall back to timed)')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='horizontal effort 0-1 for the demo leg')
    ap.add_argument('--leg', type=float, default=3.0,
                    help='seconds per straight leg in the demo')
    ap.add_argument('--pause', type=float, default=2.0,
                    help='depth-hold pause between demo moves')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if args.depth <= 0:
        ap.error('--depth must be > 0')
    if not 0.0 < args.min_speed <= args.max_speed <= 1.0:
        ap.error('need 0 < --min-speed <= --max-speed <= 1')

    target_m = args.depth * fc.FEET_TO_M
    rho = args.water_density

    if args.manual:
        confirm = ('MANUAL MODE: arms the Pixhawk, closed-loop depth hold '
                   'under your moves. THRUSTERS WILL SPIN.')
    else:
        confirm = (f'WILL SUBMERGE to {target_m:.2f} m ({args.depth:.1f} ft) '
                   f'closed-loop on Bar02, hold {args.hold_duration:.0f}s, '
                   + ('run fwd/back/strafe/turn sequence AT DEPTH, '
                      if not args.no_demo else '')
                   + 'then surface + disarm. THRUSTERS WILL SPIN.')

    with fc.session(confirm_msg=confirm, skip_confirm=args.yes) as (driver, _):
        master = getattr(driver.thrusters, 'master', None)
        if master is None:
            print('No MAVLink master (simulation / no Pixhawk) — cannot read '
                  'Bar02. Aborting.')
            return 1

        # ZED front-camera IMU: telemetry + closed-loop heading for turns.
        zed = None
        if not args.no_zed:
            zed = ZedImu()
            if not zed.available:
                print('ZED IMU: pyzed not importable — IMU telemetry OFF.')
                zed = None
            elif zed.start():
                driver.attach_heading(zed)   # turns now closed-loop on ZED IMU
                print('ZED IMU up — turns closed-loop, IMU in telemetry.')
            else:
                print('ZED IMU: no data in time — telemetry OFF, timed turns.')
                zed.stop()
                zed = None

        src = Bar02DepthSource(master, rho)
        limit = src.setup()
        if limit is None:
            return 1
        if target_m > limit:
            print(f'--depth {args.depth:.1f} ft = {target_m:.2f} m exceeds '
                  f'Bar02 usable range (~{limit:.1f} m). Aborting.')
            return 1
        # Envelope: manual gets the full Bar02 usable range (targets are
        # validated per command); auto stays tight around its one target.
        if args.max_depth > 0:
            max_depth_m = args.max_depth
        elif args.manual:
            max_depth_m = limit
        else:
            max_depth_m = 2.0 * target_m
        if max_depth_m > limit:
            print(f'NOTE: clamping max depth {max_depth_m:.2f} m → '
                  f'{limit:.2f} m (Bar02 saturation limit).')
            max_depth_m = limit
        src.start()

        keeper = DepthKeeper(driver, src, args, max_depth_m)
        keeper.start()
        try:
            if args.manual:
                manual_loop(driver, keeper, src, zed, args, limit)
            else:
                keeper.set_target(target_m)
                if not wait_for_depth(keeper, src, zed, target_m, args,
                                      args.hold_duration):
                    surface(keeper, src, args)
                    return 1
                if not args.no_demo:
                    run_movement_leg(driver, keeper, args)
                surface(keeper, src, args)
        finally:
            keeper.stop()
            src.stop()
            if zed is not None:
                zed.stop()

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
