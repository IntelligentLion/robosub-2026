#!/usr/bin/env python3
"""Isolate a MOT_x_DIRECTION mismatch between two vertical thrusters.

Context: depth_hold_bar02_test.py only ever sends a `z` heave stick — roll
correction under ALT_HOLD is entirely ArduSub's attitude controller, driven
by differential thrust between the vertical thrusters. If two thrusters that
should behave identically for the same commanded direction don't (one pushes
water the "wrong" way), the roll controller can't null a bias and the sub
sits tilted instead of leveling — this looked like a right-side-up roll
traced to motors 5 and 7 (right-side verticals) both appearing to push UP
when the params disagree: MOT_5_DIRECTION=-1 vs MOT_7_DIRECTION=1/8=1.

Spins motor A then motor B, FORWARD ONLY, in short alternating bursts so you
can watch/feel both without the confusion of a fwd+rev sweep. Uses the same
MAV_CMD_DO_MOTOR_TEST dead-man streaming as motor_test.py (ArduSub stops the
test 500ms after the last frame; a lapsed session costs a 10s cooldown).

Usage:
    python3 check_vertical_direction.py --a 5 --b 7
    python3 check_vertical_direction.py --a 5 --b 7 --rounds 3 --throttle 65
    # once you know which one is wrong (say motor 5):
    python3 check_vertical_direction.py --flip 5

THRUSTERS WILL SPIN. Props clear, kill switch in reach. Vehicle is armed for
the test and disarmed after (or on Ctrl-C).
"""

import argparse
import sys
import time

from pymavlink import mavutil

import motor_test as mt

DEFAULT_PORT = mt.DEFAULT_PORT
DEFAULT_BAUD = mt.DEFAULT_BAUD


def flip_direction(master, motor):
    """Read MOT_<motor>_DIRECTION, write its negation, verify by readback."""
    name = f'MOT_{motor}_DIRECTION'
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode('ascii'), -1)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg is None or msg.param_id != name:
        print(f'{name}: could not read current value — aborting flip.')
        return False
    cur = msg.param_value
    new = -1.0 if cur > 0 else 1.0
    print(f'{name}: {cur:+.0f} -> {new:+.0f}')
    for _ in range(3):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('ascii'), new, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg is not None and msg.param_id == name and abs(msg.param_value - new) < 0.5:
            print(f'{name} = {msg.param_value:+.0f} (confirmed)')
            return True
    print(f'{name}: SET FAILED — check link.')
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--a', type=int, default=5, help='first motor to compare (1-based)')
    ap.add_argument('--b', type=int, default=7, help='second motor to compare (1-based)')
    ap.add_argument('--throttle', type=float, default=60.0,
                    help='PERCENT forward, >50 (default 60)')
    ap.add_argument('--duration', type=float, default=1.5,
                    help='seconds per burst (default 1.5)')
    ap.add_argument('--rounds', type=int, default=2,
                    help='how many A/B alternations (default 2)')
    ap.add_argument('--flip', type=int, default=None, metavar='MOTOR',
                    help='flip MOT_<MOTOR>_DIRECTION and exit (no spinning)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    if not 50 < args.throttle <= 100:
        ap.error('--throttle must be forward, i.e. > 50 and <= 100')

    master = mt.connect(args.port, args.baud)

    if args.flip is not None:
        ok = flip_direction(master, args.flip)
        master.close()
        return 0 if ok else 1

    print(f'\nWILL alternate motor {args.a} <-> motor {args.b} FORWARD at '
          f'{args.throttle:.0f}% for {args.rounds} round(s), {args.duration:.1f}s '
          f'each. Vehicle will be ARMED. THRUSTERS WILL SPIN.')
    print('Watch/feel both — same physical push direction (both "up" or both '
          '"down") is correct. If one pushes the opposite way, that motor\'s '
          'MOT_x_DIRECTION is flipped relative to its wiring.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            master.close()
            return 1

    try:
        if not mt.arm(master, True):
            print('Arm failed — check prearm (safety switch, sensors) in QGC.')
            return 1
        time.sleep(1)
        for i in range(args.rounds):
            print(f'-- round {i + 1}/{args.rounds} --')
            mt.run_step(master, args.a, int(round(args.throttle)), args.duration)
            mt.run_step(master, args.b, int(round(args.throttle)), args.duration)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        mt.motor_test(master, args.a, 50, 0)
        mt.arm(master, False)
        master.close()

    print('\nDone. If one motor pushed the wrong way, rerun with '
          f'--flip <motor number> to reverse its MOT_x_DIRECTION, then '
          're-check with this same A/B compare before trusting depth_hold_bar02_test.py again.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
