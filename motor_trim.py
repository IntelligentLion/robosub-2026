#!/usr/bin/env python3
"""Per-motor thrust balancing for a weak thruster (ArduSub, vectored-6DOF).

Our stack commands the Pixhawk with MAVLink ``manual_control`` — ArduSub does
all the motor mixing — so a single weak thruster CANNOT be compensated in the
Python control code. The knob that exists is per-output PWM range: each motor's
output is mapped onto [SERVOn_MIN, SERVOn_MAX] around 1500 µs neutral.

The ESCs are already driven over their full usable range (1100–1900 µs), so a
weak motor can't be boosted. Instead this tool DERATES every OTHER motor by
narrowing its PWM span, so all eight motors produce comparable thrust and the
sub stops crabbing/yawing on straight commands:

    weak motor   : keeps  1500 ± 400 µs  (1100–1900, full authority)
    other motors : set to 1500 ± 400·factor µs   (e.g. factor 0.85 → 1160–1840)

Trade-off: total thrust drops by ~(1-factor) on the derated motors. Fix the
hardware (prop, debris, bearing, ESC, connector) when you can — this is a
software band-aid to keep driving straight until then.

Identify the weak motor's NUMBER first (spins each in turn):
    python3 motor_test.py --sweep

Then, e.g. motor 6 is weak, derate the rest to 85%:
    python3 motor_trim.py --weak 6 --factor 0.85

Inspect / undo:
    python3 motor_trim.py --show
    python3 motor_trim.py --reset

Params persist across reboots. Stop thruster_node first (single owner of the
serial port). Safe with the sub dry — this only writes parameters.
"""

import argparse
import sys
import time

from pymavlink import mavutil

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200

NEUTRAL_US = 1500
HALF_SPAN_US = 400            # full authority: 1500 ± 400 → 1100–1900
SERVO_FUNCTION_MOTOR1 = 33    # SERVOn_FUNCTION 33..40 = Motor1..Motor8
NUM_MOTORS = 8
NUM_OUTPUTS = 16


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    if master.wait_heartbeat(timeout=10) is None:
        print('No heartbeat in 10 s — is the Pixhawk on this port?')
        sys.exit(1)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def get_param(master, name, timeout=3.0):
    """Read one parameter value (float) or None on timeout."""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            return msg.param_value
    return None


def set_param(master, name, value, retries=3):
    """Write one parameter and verify by readback. Returns True on success."""
    for _ in range(retries):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('ascii'), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        got = get_param(master, name)
        if got is not None and abs(got - value) < 0.5:
            return True
    return False


def motor_output_map(master):
    """Map motor number (1-8) -> servo output number (1-16) via SERVOn_FUNCTION."""
    mapping = {}
    for out in range(1, NUM_OUTPUTS + 1):
        fn = get_param(master, f'SERVO{out}_FUNCTION')
        if fn is None:
            continue
        fn = int(round(fn))
        if SERVO_FUNCTION_MOTOR1 <= fn < SERVO_FUNCTION_MOTOR1 + NUM_MOTORS:
            mapping[fn - SERVO_FUNCTION_MOTOR1 + 1] = out
    return mapping


def show(master, mapping):
    print(f'{"motor":>5} {"output":>6} {"MIN":>6} {"MAX":>6} {"span":>7}')
    for motor in sorted(mapping):
        out = mapping[motor]
        lo = get_param(master, f'SERVO{out}_MIN')
        hi = get_param(master, f'SERVO{out}_MAX')
        if lo is None or hi is None:
            print(f'{motor:>5} {out:>6}   (param read failed)')
            continue
        span = (hi - lo) / (2.0 * HALF_SPAN_US)
        print(f'{motor:>5} {out:>6} {int(lo):>6} {int(hi):>6} {span:>6.0%}')


def apply_trim(master, mapping, weak, factor):
    half = int(round(HALF_SPAN_US * factor))
    lo, hi = NEUTRAL_US - half, NEUTRAL_US + half
    ok = True
    for motor, out in sorted(mapping.items()):
        if motor == weak:
            tgt_lo, tgt_hi = NEUTRAL_US - HALF_SPAN_US, NEUTRAL_US + HALF_SPAN_US
        else:
            tgt_lo, tgt_hi = lo, hi
        for name, val in ((f'SERVO{out}_MIN', tgt_lo), (f'SERVO{out}_MAX', tgt_hi)):
            if set_param(master, name, val):
                print(f'  motor {motor} (output {out}): {name} = {val}')
            else:
                print(f'  motor {motor} (output {out}): FAILED to set {name}')
                ok = False
    return ok


def reset_trim(master, mapping):
    ok = True
    for motor, out in sorted(mapping.items()):
        for name, val in ((f'SERVO{out}_MIN', NEUTRAL_US - HALF_SPAN_US),
                          (f'SERVO{out}_MAX', NEUTRAL_US + HALF_SPAN_US)):
            if set_param(master, name, val):
                print(f'  motor {motor} (output {out}): {name} = {val}')
            else:
                print(f'  motor {motor} (output {out}): FAILED to set {name}')
                ok = False
    return ok


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--weak', type=int, choices=range(1, NUM_MOTORS + 1),
                    metavar='1-8', help='weak motor number (from motor_test.py)')
    ap.add_argument('--factor', type=float, default=0.85,
                    help='derate factor for the OTHER motors (default 0.85)')
    ap.add_argument('--show', action='store_true',
                    help='print current per-motor PWM ranges and exit')
    ap.add_argument('--reset', action='store_true',
                    help='restore all motors to full 1100-1900 range')
    args = ap.parse_args()

    if not (args.show or args.reset or args.weak is not None):
        ap.error('one of --weak N, --show, or --reset is required')
    if args.weak is not None and not (0.5 <= args.factor <= 1.0):
        ap.error('--factor must be in [0.5, 1.0]')

    master = connect(args.port, args.baud)
    mapping = motor_output_map(master)
    if len(mapping) < NUM_MOTORS:
        print(f'WARNING: found only {len(mapping)}/{NUM_MOTORS} motor outputs '
              f'({mapping}) — check SERVOn_FUNCTION params.')
    if not mapping:
        sys.exit(1)

    if args.show:
        show(master, mapping)
        return

    if args.reset:
        print('Restoring full range 1100-1900 on all motors…')
        ok = reset_trim(master, mapping)
    else:
        print(f'Weak motor {args.weak}: keeping full range. '
              f'Derating others to {args.factor:.0%} '
              f'({NEUTRAL_US - int(HALF_SPAN_US * args.factor)}-'
              f'{NEUTRAL_US + int(HALF_SPAN_US * args.factor)}).')
        ok = apply_trim(master, mapping, args.weak, args.factor)

    print()
    show(master, mapping)
    if not ok:
        print('Some params failed — rerun or check MAVLink link.')
        sys.exit(1)
    print('Done. Params persist across reboots. Verify with motor_test.py --sweep.')


if __name__ == '__main__':
    main()
