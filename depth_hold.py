#!/usr/bin/env python3
"""Submerge 1 ft, hold 20 s, resurface — via the Bar02 depth-hold engine.

Thin wrapper around depth_hold_bar02_test.py (tested, safety-checked ALT_HOLD
depth controller) with --depth 1 --hold-duration 20 forced. Any other flag
that script accepts (--port, --speed, --no-station-keep, --dry-run, --yes,
...) can be passed through here and is forwarded as-is.

    python3 depth_hold.py
    python3 depth_hold.py --dry-run
    python3 depth_hold.py --no-station-keep --yes
    python3 depth_hold.py --skip-imu-check

Do not duplicate the controller logic here — see depth_hold_bar02_test.py's
docstring for the full safety notes (kill switch in reach, props clear,
stop thruster_node first, one owner of the Pixhawk serial port).

Before handing off to the controller, this wrapper does its own short-lived
MAVLink connection to rule the IMU in or out as a cause of any roll/tilt seen
during descent: sensor health bits, EKF attitude/velocity flags, vibration
clipping counts, and (when a second IMU is present) primary-vs-secondary
accel agreement. A vehicle that rolls on descent with a clean IMU check
points at thrust asymmetry (motor trim/direction), not the flight
controller's attitude estimate. The connection is closed before the
controller subprocess starts — only one process may own the Pixhawk serial
port at a time.
"""

import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SCRIPT = Path(__file__).resolve().parent / 'depth_hold_bar02_test.py'
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200

# VIBRATION.vibration_{x,y,z} are RMS accel-derived, m/s^2; ArduPilot's own
# guidance treats values under this as "not vibration-limited". Clipping
# counts (accel samples that hit the sensor's clip threshold) must be zero —
# any clipping means the accel signal feeding the EKF is corrupted, which is
# a real IMU-side explanation for a bogus attitude estimate.
VIBE_WARN = 30.0
# Primary vs secondary accel disagreement (mg) beyond typical mounting-offset
# noise suggests one IMU is miscalibrated/faulty rather than both agreeing
# the vehicle is rolling.
ACCEL_AGREE_WARN_MG = 250


def _sensor_health(sys_status, bit):
    present = bool(sys_status.onboard_control_sensors_present & bit)
    healthy = bool(sys_status.onboard_control_sensors_health & bit)
    return present, healthy


def imu_preflight_check(port, baud, timeout=6.0):
    """Connect briefly, report IMU/EKF/vibration health, return True if
    nothing suggests the IMU is at fault. Always prints what it saw so a
    borderline case is visible even when it doesn't hard-fail."""
    print(f'IMU pre-flight check: connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    if master.wait_heartbeat(timeout=10) is None:
        print('  No heartbeat — cannot check IMU, aborting.')
        master.close()
        return False
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    want = {'SYS_STATUS', 'EKF_STATUS_REPORT', 'VIBRATION', 'RAW_IMU',
            'SCALED_IMU2'}
    seen = {}
    deadline = time.time() + timeout
    while time.time() < deadline and not want.issubset(seen):
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is not None and msg.get_type() in want:
            seen[msg.get_type()] = msg
    master.close()

    ok = True

    sys_status = seen.get('SYS_STATUS')
    if sys_status is None:
        print('  No SYS_STATUS — cannot confirm sensor health, aborting.')
        return False
    for label, bit in (('gyro', 1 << 2), ('accel', 1 << 3)):
        present, healthy = _sensor_health(sys_status, bit)
        print(f'  primary {label:6s} present={present} healthy={healthy}')
        if not (present and healthy):
            print(f'  -> primary {label} unhealthy: IMU IS a plausible '
                  f'cause. Fix this before blaming thrusters.')
            ok = False
    accel2_present, accel2_healthy = _sensor_health(sys_status, 1 << 20)

    ekf = seen.get('EKF_STATUS_REPORT')
    if ekf is None:
        print('  No EKF_STATUS_REPORT — cannot confirm attitude estimate.')
        ok = False
    else:
        attitude_ok = bool(ekf.flags & 0x01)      # EKF_ATTITUDE bit
        print(f'  EKF attitude flag ok={attitude_ok} '
              f'velocity_variance={ekf.velocity_variance:.3f} '
              f'compass_variance={ekf.compass_variance:.3f}')
        if not attitude_ok:
            print('  -> EKF does not trust its own attitude estimate: IMU '
                  'IS a plausible cause.')
            ok = False

    vib = seen.get('VIBRATION')
    if vib is None:
        print('  No VIBRATION message — cannot confirm accel signal quality.')
        ok = False
    else:
        clip = (vib.clipping_0, vib.clipping_1, vib.clipping_2)
        print(f'  vibe x/y/z={vib.vibration_x:.2f}/{vib.vibration_y:.2f}/'
              f'{vib.vibration_z:.2f} m/s^2  clipping={clip}')
        if any(c > 0 for c in clip):
            print('  -> accel clipping on at least one IMU: the signal the '
                  'EKF is fusing is corrupted. IMU IS a plausible cause.')
            ok = False
        if max(vib.vibration_x, vib.vibration_y, vib.vibration_z) > VIBE_WARN:
            print(f'  -> vibration above {VIBE_WARN} m/s^2 guidance level — '
                  f'borderline, worth rechecking mounting.')

    raw = seen.get('RAW_IMU')
    scaled2 = seen.get('SCALED_IMU2')
    if accel2_present and accel2_healthy and raw is not None and scaled2 is not None:
        dx = abs(raw.xacc - scaled2.xacc)
        dy = abs(raw.yacc - scaled2.yacc)
        dz = abs(raw.zacc - scaled2.zacc)
        print(f'  primary vs secondary accel diff (mg): '
              f'x={dx} y={dy} z={dz}')
        if max(dx, dy, dz) > ACCEL_AGREE_WARN_MG:
            print(f'  -> primary/secondary accel disagree by >'
                  f'{ACCEL_AGREE_WARN_MG} mg: one IMU may be faulty. IMU IS '
                  f'a plausible cause.')
            ok = False
    else:
        print('  Only one healthy accel online — no cross-check available '
              '(not itself a fault).')

    if ok:
        print('IMU check: nominal. Sensors healthy, EKF trusts attitude, no '
              'clipping. If the vehicle still rolls on descent, that is NOT '
              'the IMU — look at thrust balance (motor trim/direction).')
    else:
        print('IMU check: FAILED. Do not chase thruster trim until the '
              'flagged item(s) above are fixed — they are a real, '
              'independent explanation for bad attitude behavior.')
    return ok


def main():
    argv = sys.argv[1:]
    skip_check = '--skip-imu-check' in argv
    argv = [a for a in argv if a != '--skip-imu-check']

    port = DEFAULT_PORT
    if '--port' in argv:
        port = argv[argv.index('--port') + 1]
    baud = DEFAULT_BAUD
    if '--baud' in argv:
        baud = int(argv[argv.index('--baud') + 1])

    if not skip_check:
        if not imu_preflight_check(port, baud):
            print('Aborting — pass --skip-imu-check to dive anyway '
                  '(not recommended).')
            sys.stdout.flush()
            return 1
    else:
        print('IMU pre-flight check skipped (--skip-imu-check).')
    # subprocess writes straight to the inherited stdout fd; without this,
    # our buffered prints above land AFTER the child's output.
    sys.stdout.flush()

    cmd = [sys.executable, str(SCRIPT), '--depth', '1', '--hold-duration', '20']
    cmd += argv
    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
