#!/usr/bin/env python3
"""Depth-hold test for Pixhawk 2.4.8 + Bar02 (MS5837-02BA) via ArduSub ALT_HOLD.

Replacement for depth_hold_pix_test.py, whose pressure detection never worked:
it passed a *tuple* of message types to pymavlink's recv_match(), and
recv_match wraps any non-list in [type] — so it compared get_type() against a
list containing one tuple and matched nothing, aborting with "No
SCALED_PRESSURE/2/3 of any kind" even while the sensor streamed happily
(which is why test_pixhawk.py saw SCALED_PRESSURE2 just fine).

This script:
  1. Requests all data streams (same REQUEST_DATA_STREAM call test_pixhawk.py
     uses, which is known to work on this firmware) plus best-effort
     SET_MESSAGE_INTERVAL for each SCALED_PRESSURE* id.
  2. Detects whichever SCALED_PRESSURE / SCALED_PRESSURE2 / SCALED_PRESSURE3
     is actually streaming by reading every message and matching get_type()
     itself — no recv_match type-filter pitfalls. On failure it prints the
     message types it DID see, so "sensor missing" vs "wrong message name" is
     obvious.
  3. Sanity-checks the surface reading. A Bar02 at the surface must read
     roughly atmospheric (~850-1100 hPa). ArduSub 4.5.x misdetects the Bar02
     as a 30BA and scales pressure ~19.6x, which makes ALT_HOLD depth garbage
     — if the surface value is absurd we abort with that diagnosis instead of
     diving on bad data. (Fixed in ArduSub 4.7.0-beta1+.)
  4. Latches surface pressure as depth-zero, switches to ALT_HOLD, arms, and
     runs a P controller on depth error that commands vertical climb-rate via
     MANUAL_CONTROL (z=500 is neutral; ALT_HOLD holds depth at neutral).
  5. Holds --hold-duration seconds inside tolerance, surfaces, disarms.
     Hard abort + surface if depth ever exceeds --max-depth (default 2x
     target) or the depth reading goes implausible mid-run.

Usage:
    python3 depth_hold_bar02_test.py --depth 3 --hold-duration 20
    --depth          target depth in FEET below surface baseline (default 3)
    --hold-duration  seconds to hold at target (default 20)
    --port           flight-controller serial (default /dev/ttyACM0)
    --dry-run        detect sensor + latch surface, then exit (no arm/motion)

Horizontal drift (ALT_HOLD holds depth + attitude only, NOT position) is
countered by an optional ZED 2i station-keeper: the camera's WORLD-frame pose
feeds a PI controller that commands forward/lateral thrust to hold the latched
start point. The integral term nulls the steady offset that current, tether
pull and thruster asymmetry would otherwise leave. The same keeper also runs a
P controller on ZED heading into the yaw (r) channel to hold the latched
heading — ArduSub's gyro-only yaw (EK3_SRC1_YAW=0) drifts slowly with no
absolute reference, and this test has no other authority over it. Disable
station-keep with --no-station-keep, or just yaw hold with --sk-yaw-kp 0. Needs
visual texture (works in a pool; blank/dark water degrades tracking — the
keeper idles when the ZED reports no fix).

IMPORTANT: stop thruster_node first — one owner of the Pixhawk serial port.
Do NOT run vslam_node at the same time — this script opens the ZED itself and
one process owns the camera.
THRUSTERS WILL SPIN. Props clear, tether/bench, kill switch in reach.
"""

import argparse
import atexit
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime

from pymavlink import mavutil

# pymavlink 2.4.49: add_message() raises TypeError ('NoneType' object does not
# support item assignment) when an instanced message (e.g. SCALED_PRESSURE2)
# arrives after a cached entry whose _instances is None. The exception escapes
# recv_match() and kills every recv after it. Drop the stale cache entry and
# retry instead.
_orig_add_message = mavutil.add_message


def _safe_add_message(messages, mtype, msg):
    try:
        _orig_add_message(messages, mtype, msg)
    except TypeError:
        messages.pop(mtype, None)
        _orig_add_message(messages, mtype, msg)


mavutil.add_message = _safe_add_message

# ZED 2i positional tracking (station-keeping). Optional: if the SDK/camera is
# missing the depth test still runs, just without horizontal hold. Same API and
# coordinate system as src/localization/localization/vslam_node.py.
try:
    import pyzed.sl as sl
except Exception:
    sl = None

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200
ALT_HOLD_MODE = 2              # ArduSub custom_mode for ALT_HOLD
RATE_HZ = 10                   # manual_control + heartbeat send rate
NEUTRAL_Z = 500                # centred vertical stick
THROTTLE_DZ = 100              # Pixhawk THR_DZ: stick counts around neutral
                               # that ArduSub treats as zero climb rate. Any
                               # vertical command must exceed this to act.
RAMP_SECONDS = 1.5             # soft-start: time to go from the min-effort
                               # floor to the full P-controller effort after
                               # a direction change (entering DESCEND/CORRECT
                               # -UP, or reversing). All four vertical
                               # thrusters share one manual_control z value —
                               # ArduSub's mixer applies it to all of them in
                               # the same frame — so ramping z ramps every
                               # thruster identically and in lockstep.
G = 9.80665                    # m/s^2
FEET_TO_M = 0.3048

# NOTE: list, not tuple — pymavlink recv_match(type=...) wraps any non-list
# argument in [arg], so a tuple silently matches nothing.
PRESSURE_TYPES = ['SCALED_PRESSURE2', 'SCALED_PRESSURE', 'SCALED_PRESSURE3']
PRESSURE_MSG_IDS = {'SCALED_PRESSURE': 29,
                    'SCALED_PRESSURE2': 137,
                    'SCALED_PRESSURE3': 143}
SERVO_OUTPUT_RAW_ID = 36       # actual PWM the FC drives each ESC with

# Plausible absolute pressure at the surface (hPa). Outside this the sensor is
# absent, broken, or scaled with the wrong-variant (30BA) math.
SURFACE_HPA_MIN = 850.0
SURFACE_HPA_MAX = 1100.0

# Bar02 is 2 bar ABSOLUTE full scale → ~10 m of fresh water before it
# saturates. Refuse targets near that.
BAR02_FULL_SCALE_PA = 200000.0
BAR02_MARGIN_M = 0.5


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def tee_output_to_log(log_dir='logs'):
    """Mirror stdout+stderr to a timestamped log file for the whole run.

    Tees at the OS file-descriptor level (dup2 onto a pipe drained by a
    background thread), not by wrapping sys.stdout — the ZED SDK logs from
    native code straight to fd 1/2, and a Python-level wrapper would miss it.
    Returns the log file path.
    """
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(
        log_dir, f'depth_hold_{datetime.now():%Y%m%d_%H%M%S}.log')
    log_f = open(path, 'ab', buffering=0)

    def pump(read_fd, orig_fd):
        while True:
            data = os.read(read_fd, 4096)
            if not data:
                break
            os.write(orig_fd, data)
            log_f.write(data)
            # fsync every chunk: a power cut (kill switch / brownout) must not
            # eat the last seconds of the log — that tail is exactly the part
            # that explains what the sub was doing when power died.
            os.fsync(log_f.fileno())

    saved = []
    threads = []
    for fd in (1, 2):
        orig = os.dup(fd)
        r, w = os.pipe()
        os.dup2(w, fd)
        os.close(w)
        t = threading.Thread(target=pump, args=(r, orig), daemon=True)
        t.start()
        saved.append((fd, orig))
        threads.append(t)
    # fd 1 is now a pipe (not a tty) → Python would switch to block
    # buffering and telemetry lines would lag; force line buffering back.
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)

    def restore():
        # Point fds back at the terminal; closing the pipe write ends EOFs
        # the pump threads so the log gets the final lines before exit.
        sys.stdout.flush()
        sys.stderr.flush()
        for fd, orig in saved:
            os.dup2(orig, fd)
        for t in threads:
            t.join(timeout=1.0)
        log_f.close()

    atexit.register(restore)
    return path


class StationKeeper:
    """Hold horizontal position over a latched point using the ZED 2i.

    Opens the ZED directly on a background thread (RIGHT_HANDED_Y_UP, metres —
    same as vslam_node) and continuously reads the WORLD-frame pose. In this
    frame the horizontal plane is X (right) and Z (backward); Y is vertical and
    is left to ALT_HOLD / the depth controller. The control loop calls
    compute() each tick and gets back (x_cmd, y_cmd) in MANUAL_CONTROL units
    (forward, right; -1000..1000) that steer the sub back to the reference.

    A PI controller is used on purpose. The P term corrects displacement; the
    I term removes the *steady-state* offset that a constant disturbance —
    water current, tether drag, thruster asymmetry — leaves behind, which pure
    proportional control cannot. Integral is clamped (anti-windup) and frozen
    whenever tracking is not OK so a lost fix can't wind it up.
    """

    def __init__(self, kp, ki, i_limit, out_max, yaw_kp, yaw_max, fps=30):
        self.kp = kp
        self.ki = ki
        self.i_limit = i_limit
        self.out_max = out_max
        self.yaw_kp = yaw_kp            # r-channel P gain (units per radian err)
        self.yaw_max = yaw_max         # r-channel clamp, MANUAL_CONTROL units
        self.fps = fps
        self.available = sl is not None
        self.ref = None                 # (rx, rz) latched world reference
        self.ref_yaw = None             # latched heading (rad) to hold
        self._lock = threading.Lock()
        self._pose = None               # (x, z, yaw) world, or None
        self._ok = False                # tracking state == OK
        self._stop = False
        self._if = 0.0                  # integral accumulator, forward
        self._ir = 0.0                  # integral accumulator, right
        self._last_t = None
        self._thread = None

    def start(self, first_fix_timeout=8.0):
        """Spin up the camera thread; wait briefly for the first good fix."""
        if not self.available:
            return False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        t0 = time.time()
        while time.time() - t0 < first_fix_timeout:
            with self._lock:
                if self._ok:
                    return True
            time.sleep(0.1)
        with self._lock:
            return self._ok

    def stop(self):
        self._stop = True

    @staticmethod
    def _yaw_from_quat(qx, qy, qz, qw):
        """Heading about the Y-up axis: angle of body-forward (0,0,-1) rotated
        into the world, measured as atan2(fx, -fz). Positive = turned right."""
        fx = -2.0 * (qx * qz + qy * qw)
        neg_fz = 1.0 - 2.0 * (qx * qx + qy * qy)
        return math.atan2(fx, neg_fz)

    def _loop(self):
        cam = None
        try:
            init = sl.InitParameters()
            init.coordinate_units = sl.UNIT.METER
            init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            init.camera_fps = self.fps
            cam = sl.Camera()
            if cam.open(init) != sl.ERROR_CODE.SUCCESS:
                with self._lock:
                    self.available = False
                return
            cam.enable_positional_tracking(sl.PositionalTrackingParameters())
            pose = sl.Pose()
            while not self._stop:
                if cam.grab() != sl.ERROR_CODE.SUCCESS:
                    time.sleep(0.005)
                    continue
                state = cam.get_position(pose, sl.REFERENCE_FRAME.WORLD)
                ok = (state == sl.POSITIONAL_TRACKING_STATE.OK)
                t = pose.get_translation(sl.Translation()).get()
                o = pose.get_orientation(sl.Orientation()).get()  # qx,qy,qz,qw
                yaw = self._yaw_from_quat(o[0], o[1], o[2], o[3])
                with self._lock:
                    self._pose = (float(t[0]), float(t[2]), yaw)
                    self._ok = bool(ok)
        except Exception as e:
            print(f'StationKeeper: ZED loop error: {e}')
            with self._lock:
                self._ok = False
        finally:
            if cam is not None:
                try:
                    cam.disable_positional_tracking()
                except Exception:
                    pass
                try:
                    cam.close()
                except Exception:
                    pass

    def latch(self):
        """Latch the current pose as the hold reference. Returns True if a good
        fix was available."""
        with self._lock:
            if self._pose is not None and self._ok:
                self.ref = (self._pose[0], self._pose[1])
                self.ref_yaw = self._pose[2]
                self._if = self._ir = 0.0
                self._last_t = None
                return True
        return False

    def yaw(self):
        """Latest ZED heading (rad, +right) or None. Visual ground truth for
        the yaw-rotation diagnosis: independent of compass and EKF."""
        with self._lock:
            if self._pose is not None and self._ok:
                return self._pose[2]
        return None

    def compute(self):
        """(x_cmd, y_cmd) in MANUAL_CONTROL units toward the reference. Returns
        (0, 0) — and freezes the integral — when there is no fix / no ref."""
        with self._lock:
            pose, ok, ref = self._pose, self._ok, self.ref
        if pose is None or not ok or ref is None:
            self._last_t = None
            return 0, 0
        x, z, yaw = pose
        rx, rz = ref
        ex, ez = rx - x, rz - z         # world vector: current -> reference
        sh, ch = math.sin(yaw), math.cos(yaw)
        fwd_err = ex * sh - ez * ch     # project onto body forward / right
        right_err = ex * ch + ez * sh
        now = time.monotonic()
        dt = 0.0 if self._last_t is None else now - self._last_t
        self._last_t = now
        self._if = clamp(self._if + fwd_err * dt, -self.i_limit, self.i_limit)
        self._ir = clamp(self._ir + right_err * dt, -self.i_limit, self.i_limit)
        xf = clamp(self.kp * fwd_err + self.ki * self._if, -1.0, 1.0)
        yf = clamp(self.kp * right_err + self.ki * self._ir, -1.0, 1.0)
        return int(xf * self.out_max), int(yf * self.out_max)

    def compute_yaw(self):
        """r-channel command (MANUAL_CONTROL units, +right/CW) to hold the
        latched heading. Returns 0 with no fix / no ref. Pure P on the shortest
        angular error — enough to null the slow gyro-only ALT_HOLD yaw drift
        that this script has no other authority over."""
        with self._lock:
            pose, ok, ref_yaw = self._pose, self._ok, self.ref_yaw
        if pose is None or not ok or ref_yaw is None:
            return 0
        err = math.atan2(math.sin(pose[2] - ref_yaw),
                         math.cos(pose[2] - ref_yaw))   # +ve = turned right
        # Turned right of ref → command left (negative r) to come back.
        r = clamp(-self.yaw_kp * err, -1.0, 1.0)
        return int(r * self.yaw_max)


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    hb = master.wait_heartbeat(timeout=10)
    if hb is None:
        print('No heartbeat in 10 s — is the Pixhawk on this port?')
        sys.exit(1)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def request_streams(master, hz=10):
    """Request telemetry the way test_pixhawk.py does (known-good on this
    firmware), then best-effort SET_MESSAGE_INTERVAL per pressure id."""
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, hz, 1)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, hz, 1)
    interval_us = int(1e6 / hz)
    for msg_id in list(PRESSURE_MSG_IDS.values()) + [SERVO_OUTPUT_RAW_ID]:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0)


def detect_pressure_source(master, timeout=8.0, settle=3.0):
    """Find which SCALED_PRESSURE* messages stream and pick the external baro.

    On ArduSub, SCALED_PRESSURE is baro instance 0 — the barometer on the
    flight-controller board, inside the hull (reads hull air pressure, not
    water). The external Bar02 shows up as instance 1+ (SCALED_PRESSURE2/3).
    So we collect for `settle` seconds after the first pressure message and
    prefer SCALED_PRESSURE2 > SCALED_PRESSURE3 > SCALED_PRESSURE, rather than
    latching whichever arrives first. Prints every source seen with its
    reading; prints all message types seen if nothing pressure-like arrives.
    """
    preference = ['SCALED_PRESSURE2', 'SCALED_PRESSURE3', 'SCALED_PRESSURE']
    seen = {}
    pressure = {}                       # type -> latest msg
    deadline = time.time() + timeout
    settle_deadline = None
    while time.time() < deadline:
        if settle_deadline is not None and time.time() >= settle_deadline:
            break
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        mtype = msg.get_type()
        seen[mtype] = seen.get(mtype, 0) + 1
        if mtype in PRESSURE_TYPES:
            pressure[mtype] = msg
            if settle_deadline is None:
                settle_deadline = time.time() + settle
    if not pressure:
        print(f'No pressure message within {timeout:.0f}s. Types seen '
              'instead:')
        for mtype in sorted(seen, key=seen.get, reverse=True):
            print(f'    {mtype:30s} x{seen[mtype]}')
        return None, None
    for mtype in preference:
        if mtype in pressure:
            m = pressure[mtype]
            note = (' <- internal FMU baro (hull air, NOT water depth)'
                    if mtype == 'SCALED_PRESSURE' else ' <- external baro')
            print(f'    {mtype:20s} {m.press_abs:8.1f} hPa '
                  f'{m.temperature / 100.0:5.1f} °C{note}')
    chosen = next(t for t in preference if t in pressure)
    if chosen == 'SCALED_PRESSURE' and len(pressure) == 1:
        print('WARNING: only the internal FMU baro is streaming — no '
              'external Bar02 message. Check BARO_PROBE_EXT=768 / '
              'BARO_EXT_BUS=1 and I2C wiring.')
    return chosen, pressure[chosen]


def read_pressure_hpa(master, ptype, timeout=1.0):
    msg = master.recv_match(type=[ptype], blocking=True, timeout=timeout)
    return None if msg is None else msg.press_abs


def latch_surface(master, ptype, samples=10):
    """Median surface pressure (hPa) as depth-zero baseline."""
    vals = []
    deadline = time.time() + 6.0
    while len(vals) < samples and time.time() < deadline:
        p = read_pressure_hpa(master, ptype)
        if p is not None:
            vals.append(p)
    return statistics.median(vals) if vals else None


def surface_sane(hpa):
    return SURFACE_HPA_MIN <= hpa <= SURFACE_HPA_MAX


def set_alt_hold(master):
    """Command ALT_HOLD, then verify the autopilot actually switched by
    watching heartbeat custom_mode — the ACK alone proves nothing (ArduSub can
    silently bounce back to MANUAL, e.g. when the Bar02 drops off I2C).
    Returns True only once a heartbeat reports ALT_HOLD."""
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        ALT_HOLD_MODE)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    print(f'ALT_HOLD ACK: result={ack.result}' if ack
          else 'No ACK for set_mode — verifying via heartbeat…')
    hb = None
    deadline = time.time() + 5.0
    while time.time() < deadline:
        hb = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == ALT_HOLD_MODE:
            print('Mode verified: ALT_HOLD active.')
            return True
    print(f'MODE VERIFY FAILED: autopilot is not in ALT_HOLD '
          f'(last heartbeat custom_mode='
          f'{hb.custom_mode if hb else "none received"}). '
          f'Depth hold would not work — check the Bar02 (mode 19 = MANUAL '
          f'forced because the depth sensor is gone).')
    return False


def arm(master, armed):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if armed else 0, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    label = 'Arm' if armed else 'Disarm'
    if ack is None:
        print(f'{label}: NO ACK')
        return False
    print(f'{label} ACK: result={ack.result}'
          + ('' if ack.result == 0 else '  REJECTED — check pre-arm/safety'))
    return ack.result == 0


def read_param(master, name, timeout=3.0):
    """Read one parameter's current value. None on timeout/no response.

    Read-only by design: this script must never write flight-controller
    params at runtime. Ad-hoc runtime overrides from other diagnostic
    scripts (MOT_x_DIRECTION / SERVOx_REVERSED flips left uncommitted) are
    exactly what caused a vertical thruster to fight the other three during
    a dive — see pixhawk_params_4.5.7_backup_2026-07-08.param for the
    known-good baseline. Persistent tuning changes belong in that param
    file / QGC, not in a per-run write here.
    """
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type=['PARAM_VALUE'], blocking=True, timeout=timeout)
        if msg is not None and msg.param_id == name:
            return msg.param_value
    return None


## MOT_x_DIRECTION for the 4 vertical thrusters (motors 5-8) and their
# SERVOx_FUNCTION/REVERSED, from pixhawk_params_4.5.7_backup_2026-07-08.param
# — the last known-good baseline. This is the one config that decides which
# way the single `z` heave stick actually pushes water: if any one of these
# is flipped relative to the rest, that thruster fights the other three
# instead of adding to the descend/ascend push (this exact drift, on motor 5
# and SERVO5/7_REVERSED, previously made the sub roll/fight during a dive —
# see vertical_thruster_motor_test_red_herring in project memory). Checked
# every run, unconditionally: no flag disables this, matching the
# read-only-param policy on this script (see read_param's docstring).
VERTICAL_MOTORS = (5, 6, 7, 8)
# Horizontal vectored thrusters (motors 1-4), same backup. A single flipped
# direction/function here turns a pure forward (x) command into a yaw torque
# — the sub spins instead of driving straight — so scripts that command x/y
# must gate on these too, not just the verticals.
HORIZONTAL_MOTORS = (1, 2, 3, 4)
# Mixer forward factors for FRAME_CONFIG=2 (VECTORED_6DOF), from ArduSub 4.5
# AP_Motors6DOF.cpp setup_motors(). PWM offset for a pure surge command is
# x * factor * MOT_x_DIRECTION; with the backup directions (-1,-1,+1,+1) the
# product is +1 for all four horizontals, so a pure x command drives every
# horizontal thruster the same way — same side of trim, all pushing the sub
# in the same direction. The PWM watchdog checks this live during a surge.
FWD_FACTOR = {1: -1.0, 2: -1.0, 3: 1.0, 4: 1.0}
EXPECT_MOT_DIRECTION = {1: -1.0, 2: -1.0, 3: 1.0, 4: 1.0,
                        5: -1.0, 6: 1.0, 7: 1.0, 8: -1.0}
EXPECT_SERVO_FUNCTION = {1: 33.0, 2: 34.0, 3: 35.0, 4: 36.0,
                         5: 37.0, 6: 38.0, 7: 39.0, 8: 40.0}
EXPECT_SERVO_REVERSED = {m: 0.0 for m in range(1, 9)}
# PWM endpoints/neutral, same backup. The PWM watchdog uses these to spot
# saturated or dead outputs; the preflight verifies they haven't drifted.
EXPECT_SERVO_TRIM = 1500.0
EXPECT_SERVO_MIN = 1100.0
EXPECT_SERVO_MAX = 1900.0


def verify_thruster_params(master, motors, label):
    """Hard gate: compare live MOT_x_DIRECTION / SERVOx_FUNCTION /
    SERVOx_REVERSED / SERVOx_TRIM/MIN/MAX for the given motors against the
    known-good backup. Returns True only if every value matches. No bypass
    flag — a mismatch means a stick command will not map to coherent thrust
    (verticals fight on z; a flipped horizontal turns forward thrust into a
    spin), which is unsafe to run regardless of the rest of the preflight."""
    print(f'Verifying {label} thruster direction/function params…')
    ok = True
    for m in motors:
        checks = (
            (f'MOT_{m}_DIRECTION', EXPECT_MOT_DIRECTION[m]),
            (f'SERVO{m}_FUNCTION', EXPECT_SERVO_FUNCTION[m]),
            (f'SERVO{m}_REVERSED', EXPECT_SERVO_REVERSED[m]),
            (f'SERVO{m}_TRIM', EXPECT_SERVO_TRIM),
            (f'SERVO{m}_MIN', EXPECT_SERVO_MIN),
            (f'SERVO{m}_MAX', EXPECT_SERVO_MAX),
        )
        for name, expect in checks:
            val = read_param(master, name)
            if val is None:
                print(f'  {name}: NO RESPONSE — cannot confirm. Aborting.')
                ok = False
                continue
            match = abs(val - expect) < 0.5
            print(f'  {name} = {val:+.0f} (expect {expect:+.0f})'
                  + ('' if match else '  <-- MISMATCH'))
            if not match:
                ok = False
    if ok:
        print(f'{label} thruster config: matches known-good backup.')
    else:
        print(f'ABORT: {label} thruster config does NOT match the '
              'known-good backup (pixhawk_params_4.5.7_backup_2026-07-08.param). '
              'Stick commands would not map to coherent thrust — fix the '
              'flagged param(s) in QGC/the backup file before diving. '
              'This check cannot be skipped.')
    return ok


def verify_vertical_thruster_directions(master):
    """Back-compat wrapper: hard-gate the 4 vertical thrusters (5-8)."""
    return verify_thruster_params(master, VERTICAL_MOTORS, 'vertical (5-8)')


def verify_horizontal_thruster_directions(master):
    """Hard-gate the 4 horizontal vectored thrusters (1-4). Required before
    any script that commands x/y: one flipped horizontal makes a forward
    command yaw the sub instead of driving it straight."""
    return verify_thruster_params(master, HORIZONTAL_MOTORS,
                                  'horizontal (1-4)')


class PwmMonitor:
    """Watch SERVO_OUTPUT_RAW — the actual PWM the FC drives each ESC with —
    for vertical-thruster abnormalities. This is the ground truth for "did the
    mixer turn my z command into a coherent submerge push": params can look
    right and the sub still not move if an output is dead, saturated, or
    fighting the others.

    Checks each update (all rate-limited to one print per 2 s per kind):
      * dead output: a vertical channel reporting 0 (FC not driving that pin)
      * saturation: a vertical pinned at SERVO_MIN/MAX (controller demanding
        more than the thruster can give — trim/ballast problem)
      * fight: with z commanded past the deadzone, the sign of each vertical's
        thrust — (pwm - trim) * MOT_x_DIRECTION — must agree across all four.
        A split means one thruster pushes against the other three: the exact
        failure the param preflight guards against, caught here at runtime.
      * no response: z commanded past the deadzone but every vertical still
        sits at trim — mixer is not listening (wrong mode / not armed /
        another writer on the port).

    With an x (surge) command it also checks the horizontals (1-4): each
    live output's PWM offset sign must match x * FWD_FACTOR *
    MOT_x_DIRECTION — on this frame/params that means all four on the same
    side of trim, all pushing the same direction. A mismatch or a horizontal
    stuck at trim during a surge means the sub will spin or crab instead of
    driving straight.
    """

    WARN_PERIOD = 2.0
    ACTIVE_PWM = 40         # |pwm - trim| beyond this counts as "driving"
    ACTIVE_X = 50           # |x| beyond this counts as "commanding surge"

    def __init__(self):
        self.pwm = None                 # latest 8-tuple, servo1..8
        self._last_warn = {}

    def _warn(self, key, text):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= self.WARN_PERIOD:
            self._last_warn[key] = now
            print(f'PWM WATCH: {text}')

    def update(self, pwm, z_cmd, x_cmd=0):
        """Feed one SERVO_OUTPUT_RAW frame (8-tuple) + the z/x just
        commanded. x_cmd=0 skips the horizontal check (depth-only runs)."""
        self.pwm = pwm
        if abs(x_cmd) > self.ACTIVE_X:
            self._check_horizontals(pwm, x_cmd)
        vert = {m: pwm[m - 1] for m in VERTICAL_MOTORS}
        for m, p in vert.items():
            if p == 0:
                self._warn(f'dead{m}', f'motor {m} output is 0 — FC is not '
                           f'driving that channel.')
            elif p <= EXPECT_SERVO_MIN + 2 or p >= EXPECT_SERVO_MAX - 2:
                self._warn(f'sat{m}', f'motor {m} saturated at {p} — '
                           f'controller is demanding more than the thruster '
                           f'can give.')
        commanding = abs(z_cmd - NEUTRAL_Z) > THROTTLE_DZ
        live = {m: p for m, p in vert.items() if p != 0}
        if not commanding or len(live) < len(vert):
            return
        signs = {m: (p - EXPECT_SERVO_TRIM) * EXPECT_MOT_DIRECTION[m]
                 for m, p in live.items()
                 if abs(p - EXPECT_SERVO_TRIM) > self.ACTIVE_PWM}
        if not signs:
            self._warn('noresp', f'z={z_cmd} commanded but all verticals at '
                       f'trim ({self.fmt()}) — mixer not responding '
                       f'(mode? armed? second writer on the port?)')
        elif min(signs.values()) < 0 < max(signs.values()):
            detail = ' '.join(f'm{m}:{vert[m]}({"+" if s > 0 else "-"})'
                              for m, s in sorted(signs.items()))
            self._warn('fight', f'vertical thrust signs disagree — {detail} '
                       f'— one thruster is fighting the others.')

    def _check_horizontals(self, pwm, x_cmd):
        """Surge in progress: every horizontal must push the way x says."""
        horiz = {m: pwm[m - 1] for m in HORIZONTAL_MOTORS}
        live = {m: p for m, p in horiz.items() if p != 0}
        for m in HORIZONTAL_MOTORS:
            if m not in live:
                self._warn(f'hdead{m}', f'motor {m} output is 0 during surge '
                           f'— FC is not driving that channel.')
        wrong, idle = [], []
        for m, p in live.items():
            expect = x_cmd * FWD_FACTOR[m] * EXPECT_MOT_DIRECTION[m]
            offset = p - EXPECT_SERVO_TRIM
            if abs(offset) <= self.ACTIVE_PWM:
                idle.append(m)
            elif offset * expect < 0:
                wrong.append((m, p))
        if wrong:
            detail = ' '.join(f'm{m}:{p}' for m, p in wrong)
            self._warn('hwrong', f'x={x_cmd} commanded but {detail} pushing '
                       f'the WRONG direction — sub will spin/crab, not drive '
                       f'straight.')
        elif len(idle) == len(live):
            self._warn('hnoresp', f'x={x_cmd} commanded but all horizontals '
                       f'at trim ({self.fmt()}) — mixer not responding.')
        elif idle:
            detail = ' '.join(f'm{m}' for m in sorted(idle))
            self._warn('hidle', f'x={x_cmd} commanded but {detail} sitting at '
                       f'trim — not contributing to the surge.')

    def fmt(self):
        """Compact all-8 PWM string for the telemetry line."""
        if self.pwm is None:
            return 'pwm=n/a'
        return 'pwm=' + ','.join(str(p) for p in self.pwm)


def vertical_z(effort, direction):
    """Map an effort fraction (0..1) to a manual_control z that actually
    exceeds the ALT_HOLD stick deadzone (THR_DZ). Effort then scales the
    commanded climb rate linearly: rate = effort * PILOT_SPEED_DN/UP.
    direction: -1 descend, +1 ascend."""
    offset = THROTTLE_DZ + effort * (500 - THROTTLE_DZ)
    return NEUTRAL_Z + direction * round(clamp(offset, 0, 500))


def send_frame(master, z, x=0, y=0, r=0):
    """One manual_control frame + GCS heartbeat (failsafe guard). z is vertical
    (0..1000, 500 neutral); x/y are forward/lateral station-keeping thrust; r is
    yaw (-1000..1000, +right) holding the latched heading."""
    master.mav.manual_control_send(
        master.target_system, x=int(x), y=int(y), z=int(z), r=int(r),
        buttons=0)
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def drain_depth(master, surface_hpa, rho, ptype):
    """Pull all buffered pressure + attitude + servo-output msgs. Returns
    (depth_m, yaw_rad, pwm), each None if no fresh message of that kind was
    buffered. Yaw is the Pixhawk EKF heading (ATTITUDE.yaw) — what ALT_HOLD is
    actually holding. pwm is the servo1..8 output tuple from SERVO_OUTPUT_RAW
    — the actual PWM sent to the ESCs."""
    depth = None
    yaw = None
    pwm = None
    while True:
        msg = master.recv_match(type=[ptype, 'ATTITUDE', 'SERVO_OUTPUT_RAW'],
                                blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == 'ATTITUDE':
            yaw = msg.yaw
        elif mtype == 'SERVO_OUTPUT_RAW':
            pwm = (msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                   msg.servo4_raw, msg.servo5_raw, msg.servo6_raw,
                   msg.servo7_raw, msg.servo8_raw)
        else:
            depth = (msg.press_abs - surface_hpa) * 100.0 / (rho * G)
    return depth, yaw, pwm


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--depth', type=float, default=3.0,
                    help='target depth in FEET below surface (default 3.0)')
    ap.add_argument('--hold-duration', type=float, default=20.0,
                    help='seconds to hold once target reached (default 20)')
    ap.add_argument('--run-seconds', type=float, default=2.0,
                    help='hard cap on the dive loop: surface + disarm this '
                         'many seconds after arming, even if the target/hold '
                         'was never reached (0 = no cap; default 2)')
    ap.add_argument('--kp', type=float, default=2.0,
                    help='gain: climb-rate fraction per metre of error')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='overall vertical strength scale 0-1: multiplies '
                         'every vertical effort (descend, correct, surface). '
                         'e.g. 0.5 = half speed (default 1.0)')
    ap.add_argument('--min-speed', type=float, default=0.15,
                    help='min vertical effort while moving (0-1)')
    ap.add_argument('--max-speed', type=float, default=0.6,
                    help='max vertical effort (0-1)')
    ap.add_argument('--expect-descent-rate', type=float, default=25.0,
                    help='expected PILOT_SPEED_DN (cm/s) on the flight '
                         'controller; read-only sanity check, warns on '
                         'mismatch instead of writing (0 = skip check)')
    ap.add_argument('--deadband', type=float, default=0.07,
                    help='half-width (m) of neutral hold band')
    ap.add_argument('--settle-tol', type=float, default=0.1,
                    help='error (m) under which target counts as reached')
    ap.add_argument('--max-depth', type=float, default=0.0,
                    help='abort+surface past this depth (m). 0 → 2x target')
    ap.add_argument('--water-density', type=float, default=1000.0,
                    help='kg/m^3 (fresh ~1000, salt ~1025)')
    ap.add_argument('--no-station-keep', action='store_true',
                    help='disable ZED 2i horizontal position hold (drift free)')
    ap.add_argument('--sk-kp', type=float, default=0.8,
                    help='station-keep P gain (out fraction per metre error)')
    ap.add_argument('--sk-ki', type=float, default=0.15,
                    help='station-keep I gain (nulls current/tether/asym drift)')
    ap.add_argument('--sk-i-limit', type=float, default=1.0,
                    help='station-keep integral clamp (anti-windup)')
    ap.add_argument('--sk-max', type=int, default=350,
                    help='max station-keep thrust, MANUAL_CONTROL units (<=1000)')
    ap.add_argument('--sk-yaw-kp', type=float, default=6.0,
                    help='yaw-hold P gain, out fraction per radian error '
                         '(~0.10 fraction/deg; 0 disables yaw hold)')
    ap.add_argument('--sk-yaw-max', type=int, default=250,
                    help='max yaw-hold thrust, MANUAL_CONTROL units (<=1000)')
    ap.add_argument('--zed-fps', type=int, default=30,
                    help='ZED camera FPS for station-keeping')
    ap.add_argument('--dry-run', action='store_true',
                    help='detect sensor + latch surface, then exit (no arm)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    log_path = tee_output_to_log()
    print(f'Logging this run to {log_path}')
    argv = ' '.join(sys.argv[1:]) or '(default args)'
    print(f'Run started {datetime.now():%Y-%m-%d %H:%M:%S} — args: {argv}')

    if args.depth <= 0:
        ap.error('--depth must be > 0')
    if not 0.0 < args.min_speed <= args.max_speed <= 1.0:
        ap.error('need 0 < --min-speed <= --max-speed <= 1')
    if not 0.0 < args.speed <= 1.0:
        ap.error('--speed must be in (0, 1]')

    target_m = args.depth * FEET_TO_M
    max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * target_m
    rho = args.water_density

    bar02_limit_m = ((BAR02_FULL_SCALE_PA - 101325.0) / (rho * G)
                     - BAR02_MARGIN_M)
    if target_m > bar02_limit_m:
        ap.error(f'--depth {args.depth:.1f} ft = {target_m:.2f} m exceeds '
                 f'Bar02 usable range (~{bar02_limit_m:.1f} m)')
    if max_depth_m > bar02_limit_m:
        print(f'NOTE: clamping max depth {max_depth_m:.2f} m → '
              f'{bar02_limit_m:.2f} m (Bar02 saturation limit)')
        max_depth_m = bar02_limit_m

    master = connect(args.port, args.baud)

    if not verify_vertical_thruster_directions(master):
        master.close()
        return 1

    request_streams(master)

    print('Detecting depth/pressure source…')
    ptype, first = detect_pressure_source(master)
    if ptype is None:
        print('No SCALED_PRESSURE/2/3 arrived. Since test_pixhawk.py saw '
              'SCALED_PRESSURE2, re-check wiring/params only if it now fails '
              'too: Bar02 on external I2C, BARO_PROBE_EXT=768, '
              'BARO_EXT_BUS=1, reboot after. Aborting.')
        master.close()
        return 1
    print(f'Using depth source: {ptype} '
          f'(first reading {first.press_abs:.1f} hPa, '
          f'{first.temperature / 100.0:.1f} °C)')

    print('Latching surface pressure (keep sub at surface, still)…')
    surface_hpa = latch_surface(master, ptype)
    if surface_hpa is None:
        print(f'{ptype} stopped streaming during latch — aborting.')
        master.close()
        return 1
    print(f'Surface baseline = {surface_hpa:.1f} hPa')

    if not surface_sane(surface_hpa):
        ratio = surface_hpa / 1013.25
        print(f'ABORT: surface pressure {surface_hpa:.0f} hPa is not '
              f'atmospheric ({ratio:.1f}x expected).')
        if 15.0 < ratio < 25.0:
            print('  ~19.6x high matches the known ArduSub 4.5.x bug that '
                  'reads the Bar02 with 30BA math. ALT_HOLD depth is garbage '
                  'on this firmware — upgrade to ArduSub 4.7.0-beta1+ (or '
                  'swap in a Bar30) before depth testing.')
        else:
            print('  Sensor mis-scaled or faulty — do not trust ALT_HOLD '
                  'until this reads ~1013 hPa in air at the surface.')
        master.close()
        return 1
    print('Surface pressure plausible — sensor scaling looks correct.')

    if args.dry_run:
        print('Dry run complete: sensor detected, baseline sane. '
              'Not arming.')
        master.close()
        return 0

    cap_str = (f'Run capped at {args.run_seconds:.1f}s (--run-seconds). '
               if args.run_seconds > 0 else '')
    print(f'\nWILL SUBMERGE to {target_m:.2f} m ({args.depth:.1f} ft) via '
          f'ALT_HOLD, hold {args.hold_duration:.0f}s, then surface. '
          f'{cap_str}'
          f'Vertical strength scale --speed={args.speed:.2f}. '
          f'THRUSTERS WILL SPIN.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            master.close()
            return 1

    if not set_alt_hold(master):
        print('Aborting — not arming without a verified ALT_HOLD.')
        master.close()
        return 1
    if args.expect_descent_rate > 0:
        actual = read_param(master, 'PILOT_SPEED_DN')
        if actual is None:
            print('WARNING: could not read PILOT_SPEED_DN — proceeding '
                  'without confirming descent-rate cap.')
        elif abs(actual - args.expect_descent_rate) > 0.5:
            print(f'WARNING: PILOT_SPEED_DN={actual:.0f} cm/s on the flight '
                  f'controller, expected {args.expect_descent_rate:.0f}. '
                  f'This script no longer writes params at runtime — fix it '
                  f'in QGC / the .param backup, not here.')
        else:
            print(f'PILOT_SPEED_DN = {actual:.0f} cm/s (as expected).')
    time.sleep(0.5)
    if not arm(master, True):
        print('Arm failed — aborting, not driving.')
        master.close()
        return 1

    sk = None
    if not args.no_station_keep:
        sk = StationKeeper(args.sk_kp, args.sk_ki, args.sk_i_limit,
                           args.sk_max, args.sk_yaw_kp, args.sk_yaw_max,
                           fps=args.zed_fps)
        if not sk.available:
            print('Station-keep: pyzed not importable — horizontal hold OFF '
                  '(sub will drift). Install ZED SDK or pass --no-station-keep '
                  'to silence.')
            sk = None
        elif sk.start():
            if sk.latch():
                print(f'Station-keep: ZED fix OK, reference latched. '
                      f'PI kp={args.sk_kp} ki={args.sk_ki} max={args.sk_max}. '
                      f'Yaw hold kp={args.sk_yaw_kp} max={args.sk_yaw_max}.')
            else:
                print('Station-keep: ZED up but no fix yet — will latch once '
                      'tracking locks.')
        else:
            print('Station-keep: ZED did not lock in time — horizontal hold '
                  'idle until it does (sub may drift meanwhile).')

    period = 1.0 / RATE_HZ
    reached_at = None
    aborted = False
    depth = 0.0
    loops = 0
    # Yaw diagnosis: log Pixhawk EKF yaw vs ZED visual yaw as deltas from
    # their values at arm. Sub physically turning while pix stays ~0 = EKF
    # heading is being dragged (compass interference) and ALT_HOLD follows it.
    pix_yaw = zed_yaw = None
    pix_yaw0 = zed_yaw0 = None
    move_dir = 0             # -1 descend, 0 hold, +1 ascend — direction of
                              # the last commanded vertical effort
    ramp_t0 = None            # monotonic time the current direction began
    pwm_mon = PwmMonitor()
    last_z = NEUTRAL_Z        # z sent last tick — what the drained PWM answers
    run_t0 = time.monotonic()  # --run-seconds cap counts from here (post-arm)

    try:
        while True:
            loops += 1
            d, py, pwm = drain_depth(master, surface_hpa, rho, ptype)
            if d is not None:
                depth = d
            if pwm is not None:
                pwm_mon.update(pwm, last_z)
            if py is not None:
                pix_yaw = py
                if pix_yaw0 is None:
                    pix_yaw0 = py
            if sk is not None:
                zy = sk.yaw()
                if zy is not None:
                    zed_yaw = zy
                    if zed_yaw0 is None:
                        zed_yaw0 = zy

            # Horizontal station-keeping (0,0 if disabled / no ZED fix). Latch
            # a reference the moment tracking first locks if we couldn't at arm.
            xc, yc, rc = 0, 0, 0
            if sk is not None:
                if sk.ref is None:
                    sk.latch()
                xc, yc = sk.compute()
                rc = sk.compute_yaw()

            # Hard safety aborts: runaway descend, or reading gone implausible
            # (saturation / sensor fault mid-run).
            if not aborted and (depth > max_depth_m
                                or depth < -1.0
                                or depth > bar02_limit_m):
                aborted = True
                print(f'ABORT: depth {depth:.2f} m outside safe envelope '
                      f'(max {max_depth_m:.2f} m) — surfacing.')
            if aborted:
                if depth > args.deadband:
                    last_z = vertical_z(args.speed * args.min_speed, +1)
                    send_frame(master, last_z, xc, yc, rc)
                    time.sleep(period)
                    continue
                break

            error = target_m - depth        # +ve → need to go deeper
            mag = abs(error)
            raw_effort = args.speed * max(args.min_speed,
                                          min(args.max_speed, args.kp * mag))

            if mag <= args.deadband:
                z = NEUTRAL_Z               # ALT_HOLD locks current depth
                move_dir = 0
                ramp_t0 = None
                if reached_at is None and mag <= args.settle_tol:
                    reached_at = time.monotonic()
                    print(f'✓ Reached {depth:.2f} m — holding '
                          f'{args.hold_duration:.0f}s via ALT_HOLD.')
            else:
                direction = -1 if error > 0 else +1     # descend / ascend
                if direction != move_dir:
                    move_dir = direction
                    ramp_t0 = time.monotonic()
                ramp_frac = clamp(
                    (time.monotonic() - ramp_t0) / RAMP_SECONDS, 0.0, 1.0)
                floor = args.speed * args.min_speed
                effort = floor + (raw_effort - floor) * ramp_frac
                z = vertical_z(effort, direction)

            send_frame(master, z, xc, yc, rc)
            last_z = z

            if loops % RATE_HZ == 0:        # ~1 Hz telemetry
                state = ('HOLD' if mag <= args.deadband
                         else 'DESCEND' if error > 0 else 'CORRECT-UP')
                sk_str = (f' sk x={xc:+d} y={yc:+d} r={rc:+d}'
                          if sk is not None else '')

                def _dyaw(cur, ref):
                    if cur is None or ref is None:
                        return '  n/a'
                    d = math.degrees(math.atan2(math.sin(cur - ref),
                                                math.cos(cur - ref)))
                    return f'{d:+5.0f}°'
                yaw_str = (f' yaw pix={_dyaw(pix_yaw, pix_yaw0)}'
                           f' zed={_dyaw(zed_yaw, zed_yaw0)}')
                print(f'[{state}] depth={depth:.2f} m '
                      f'target={target_m:.2f} m err={error:+.2f} m z={z} '
                      f'{pwm_mon.fmt()}{sk_str}{yaw_str}')

            if (args.run_seconds > 0
                    and time.monotonic() - run_t0 >= args.run_seconds):
                print(f'Run-time cap {args.run_seconds:.1f}s reached — '
                      f'surfacing.')
                break

            if (reached_at is not None
                    and time.monotonic() - reached_at >= args.hold_duration):
                print('Hold complete — surfacing.')
                break

            time.sleep(period)

        print('Surfacing…')
        deadline = time.time() + 30.0
        while time.time() < deadline:
            loops += 1
            d, _, pwm = drain_depth(master, surface_hpa, rho, ptype)
            if d is not None:
                depth = d
            if pwm is not None:
                pwm_mon.update(pwm, last_z)
            xc, yc = sk.compute() if sk is not None else (0, 0)
            rc = sk.compute_yaw() if sk is not None else 0
            if depth <= args.deadband:
                break
            last_z = vertical_z(args.speed * args.max_speed, +1)
            send_frame(master, last_z, xc, yc, rc)
            if loops % RATE_HZ == 0:
                print(f'[SURFACE] depth={depth:.2f} m z={last_z} '
                      f'{pwm_mon.fmt()}')
            time.sleep(period)
    except KeyboardInterrupt:
        print('\nInterrupted — stopping.')
    finally:
        for _ in range(5):                  # flush neutral
            send_frame(master, NEUTRAL_Z)
            time.sleep(0.05)
        arm(master, False)
        if sk is not None:
            sk.stop()
        master.close()
    print('Done. Neutral + disarmed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
