#!/usr/bin/env python3
"""Per-thruster motor test for Pixhawk + ArduSub via MAV_CMD_DO_MOTOR_TEST.

Isolates a single thruster fault that MANUAL_CONTROL/heave testing can't: the
depth-hold scripts only send a `z` heave stick, and ArduSub mixes that to every
vertical thruster in firmware, so a dead output or an ESC that only spins one
way is invisible from the Python side. This spins ONE motor at a time, forward
AND reverse, so "bottom-left dead only on descent" narrows to output vs
direction vs ESC calibration.

Throttle is an ArduSub PERCENT where 50 = stop, >50 = forward, <50 = reverse
(sub thrusters are bidirectional, trimmed at the PWM midpoint). So a thruster
that spins at 65% but NOT at 35% is an ESC that never got a bidirectional
throttle calibration (its reverse half sits in the deadband) — the classic
"won't move going down" symptom.

Motor numbers are ArduSub's 1-based test order (motor 1..8), NOT SERVO output
numbers; ArduSub maps test-motor N → the SERVOn assigned MotorN. Use the
ArduSub/QGC frame diagram, or --all to sweep every motor and watch which prop
moves, to learn which number is the bottom-left thruster on this frame.

Usage:
    python3 motor_test.py --motor 3 --both          # motor 3 fwd then rev
    python3 motor_test.py --motor 3 --throttle 35   # motor 3 reverse only
    python3 motor_test.py --all --both              # sweep all, fwd + rev
    --motor      1-based ArduSub motor number to test
    --throttle   PERCENT: 50 stop, >50 fwd, <50 rev (default 60)
    --duration   seconds to spin each step (default 2)
    --both       run the given offset forward AND its reverse mirror
    --all        sweep --count motors in sequence (ignores --motor)
    --count      motor count for --all (default 8)
    --port       flight-controller serial (default /dev/ttyACM0)

ArduSub runs MAV_CMD_DO_MOTOR_TEST only while ARMED (opposite of Copter). The
script arms before the sweep and disarms after; a disarmed vehicle returns
MAV_RESULT_FAILED on every motor. THRUSTERS WILL SPIN. Props clear, out of
water is fine and safest.
"""

import argparse
import sys
import time

from pymavlink import mavutil

DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200
THROTTLE_TYPE_PERCENT = 0        # MOTOR_TEST_THROTTLE_PERCENT
MOTOR_TEST_ORDER_BOARD = 2       # p6: match motor numbering to SERVOn order


def connect(port, baud):
    print(f'Connecting {port} @ {baud} …')
    master = mavutil.mavlink_connection(port, baud=baud)
    if master.wait_heartbeat(timeout=10) is None:
        print('No heartbeat in 10 s — is the Pixhawk on this port?')
        sys.exit(1)
    print(f'Heartbeat OK (sysid={master.target_system} '
          f'compid={master.target_component})')
    return master


def _drain(master):
    """Discard buffered messages so a stale COMMAND_ACK from a previous step
    can't be misread as this command's result."""
    while True:
        try:
            if master.recv_match(blocking=False) is None:
                return
        except TypeError:
            # old pymavlink crashes in post_message() on some instanced
            # messages (TypeError: 'NoneType' ... _instances) — skip it
            continue


def motor_test(master, motor, throttle_pct, duration):
    """Spin one ArduSub motor (1-based) at PERCENT throttle for `duration` s.
    Returns the COMMAND_ACK result (0 == accepted) or None if no ack.

    Matches the ack to MAV_CMD_DO_MOTOR_TEST: ArduSub interleaves heartbeats and
    other acks, and sends this ack around when the test starts, so a plain
    single-shot recv_match races with unrelated traffic (the classic
    NO-ACK/REJECTED-alternating symptom)."""
    _drain(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        motor,                  # p1: motor instance (1-based)
        THROTTLE_TYPE_PERCENT,  # p2: throttle type
        throttle_pct,           # p3: throttle value
        duration,               # p4: timeout / spin time (s)
        0,                      # p5: motor count (0 = just this one)
        MOTOR_TEST_ORDER_BOARD, # p6: test order
        0)
    deadline = time.time() + 4
    while time.time() < deadline:
        try:
            ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        except TypeError:
            continue
        if ack is None:
            continue
        if ack.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            return ack.result
        # ack for some other command — keep waiting for ours
    return None


RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS',
}


def arm(master, want_armed):
    """Arm (want_armed=True) or disarm the vehicle. ArduSub, unlike Copter,
    only runs MAV_CMD_DO_MOTOR_TEST while ARMED, so we arm before the sweep."""
    _drain(master)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if want_armed else 0, 0, 0, 0, 0, 0, 0)
    try:
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    except TypeError:
        ack = None
    verb = 'Arm' if want_armed else 'Disarm'
    if ack is None:
        print(f'{verb}: NO ACK')
    else:
        print(f'{verb} ACK: {RESULT_NAMES.get(ack.result, ack.result)}')
    return ack is not None and ack.result == 0


def run_step(master, motor, throttle_pct, duration):
    arrow = 'FWD' if throttle_pct > 50 else 'REV' if throttle_pct < 50 else 'STOP'
    print(f'  motor {motor}  {throttle_pct:3d}% ({arrow})  {duration:.0f}s … ',
          end='', flush=True)
    res = motor_test(master, motor, throttle_pct, duration)
    if res is None:
        print('NO ACK')
    elif res == 0:
        print('accepted — watch the prop')
    else:
        print(f'REJECTED result={res} (armed? safety switch? bad motor num?)')
    # Spin runs on the FC for `duration`; wait it out plus a settle gap.
    time.sleep(duration + 0.5)
    return res


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--motor', type=int, default=1,
                    help='1-based ArduSub motor number (ignored with --all)')
    ap.add_argument('--throttle', type=float, default=60.0,
                    help='PERCENT: 50 stop, >50 fwd, <50 rev (default 60)')
    ap.add_argument('--duration', type=float, default=2.0,
                    help='seconds to spin each step (default 2)')
    ap.add_argument('--both', action='store_true',
                    help='run the throttle AND its reverse mirror (100-t)')
    ap.add_argument('--all', action='store_true',
                    help='sweep --count motors instead of one')
    ap.add_argument('--count', type=int, default=8,
                    help='motor count for --all (default 8)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if not 0 <= args.throttle <= 100:
        ap.error('--throttle is a PERCENT 0..100')

    master = connect(args.port, args.baud)

    # Reverse mirror about the 50% stop point: 60% fwd -> 40% rev.
    steps = [args.throttle]
    if args.both:
        steps.append(100.0 - args.throttle)
    motors = list(range(1, args.count + 1)) if args.all else [args.motor]

    print(f'\nWILL SPIN motor(s) {motors} at {steps}% for '
          f'{args.duration:.0f}s each. Vehicle will be ARMED. '
          f'THRUSTERS WILL SPIN.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            master.close()
            return 1

    try:
        # ArduSub only runs motor test while ARMED (differs from Copter).
        if not arm(master, True):
            print('Arm failed — motor test needs an armed vehicle. '
                  'Check prearm (safety switch, sensors) in QGC.')
            return 1
        time.sleep(1)  # let arming settle before commanding a test
        for m in motors:
            for t in steps:
                run_step(master, m, int(round(t)), args.duration)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        # Best-effort stop, then disarm so props can't spin after exit.
        motor_test(master, motors[0], 50, 0)
        arm(master, False)
        master.close()
    print('Done.')
    print('READ: a motor that spins at >50% but not its <50% mirror = ESC '
          'never got a bidirectional throttle calibration (reverse half in '
          'deadband). That is the usual "dead going down" cause — recalibrate '
          "that ESC's throttle range. No spin either way = dead output / wiring "
          '/ ESC. Wrong motor moves = SERVO mapping.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
