#!/usr/bin/env python3
"""Spin the 4 horizontal vectored thrusters (motors 1-4) ONE AT A TIME to
check which way each pushes water.

Why this exists: a single flipped horizontal turns a pure forward (x)
command into a yaw torque — the sub spins instead of driving straight
(see HORIZONTAL_MOTORS in depth_hold_bar02_test.py). The param preflight
catches a MOT_x_DIRECTION/SERVOx drift vs the backup, but it cannot catch
a wiring/ESC swap that flips the physical spin while the params still look
right. This script is the physical half of that check.

What "correct" looks like: with the known-good params
(MOT_1..4_DIRECTION = -1,-1,+1,+1 and forward factors -1,-1,+1,+1 on
FRAME_CONFIG=2 VECTORED_6DOF), a pure forward command drives ALL FOUR
horizontal outputs the same side of trim — so under this test every motor
should push water TOWARD THE STERN (sub nudges forward). Any motor whose
jet blows the opposite way vs the other three is the flipped one.

IMPORTANT — what DO_MOTOR_TEST can and cannot tell you (see
vertical-thruster-motor-test-red-herring in project memory): the test
writes PWM straight to the output, BYPASSING the mixer and
MOT_x_DIRECTION. So this script answers "does the physical
wiring/prop/ESC of each motor match what the params assume", by
compensating each motor's commanded PWM with its expected direction and
forward factor. It is NOT a substitute for the param preflight — which is
why the preflight runs first and this script refuses to spin if params
don't match the backup (an observation against drifted params is
uninterpretable). Do NOT "fix" a wrong-way motor by ad-hoc param writes
from this script; fix the wiring, or change the param deliberately in QGC
AND the backup file. This script never writes a param.

Usage:
    python3 check_horizontal_direction.py                 # motors 1-4, 1 round
    python3 check_horizontal_direction.py --rounds 2
    python3 check_horizontal_direction.py --motors 1 3    # subset
    python3 check_horizontal_direction.py --together      # ALL 4 at once:
        MANUAL mode + a real MANUAL_CONTROL forward x-stick through the
        mixer (DO_MOTOR_TEST is one-motor-at-a-time by design). This is the
        true oracle for "does a forward command drive straight"; prints the
        live SERVO_OUTPUT_RAW PWM per horizontal so a wrong-way output is
        visible even dry.
    python3 check_horizontal_direction.py --skip-preflight-abort  # (still
        runs the param check and prints mismatches, but spins anyway — only
        for diagnosing WHICH param is wrong; never dive off this)

THRUSTERS WILL SPIN. Props clear, kill switch in reach. Vehicle is armed
for the test and disarmed after (or on Ctrl-C). Out of water is fine and
safest — watch the prop spin direction / feel the airflow.
"""

import argparse
import sys
import time

import motor_test as mt
# Importing dh also applies the field_common pymavlink monkeypatch and gives
# us the known-good param table + the non-skippable-style preflight.
import depth_hold_bar02_test as dh
import submerge_forward_10ft as sf   # set_manual (verified mode switch)

DEFAULT_PORT = mt.DEFAULT_PORT
DEFAULT_BAUD = mt.DEFAULT_BAUD

RATE_HZ = 10                         # MANUAL_CONTROL stream rate (--together)


def surge_throttle_pct(motor, strength_pct):
    """Throttle percent (50 = stop) that reproduces what a pure FORWARD
    surge command would ask of this motor, per the ArduSub 4.5 VECTORED_6DOF
    mixer: PWM offset = x * FWD_FACTOR[m] * MOT_m_DIRECTION. With the backup
    directions the product is +1 for all four horizontals, so every motor is
    commanded the same side of trim — but computing it keeps the script
    honest if the expected table ever changes."""
    sign = dh.FWD_FACTOR[motor] * dh.EXPECT_MOT_DIRECTION[motor]
    return 50.0 + sign * strength_pct / 2.0


def run_together(master, strength_pct, duration):
    """All 4 horizontals at once: stream a forward MANUAL_CONTROL x-stick in
    MANUAL mode (mixer in the loop — the real thing a mission commands, unlike
    DO_MOTOR_TEST which drives one output directly). Prints live
    SERVO_OUTPUT_RAW for motors 1-4 about once a second: with correct config
    every horizontal sits the same side of 1500 and pushes water toward the
    stern. Caller has already armed; this returns with sticks at neutral."""
    # strength is percent of full power: 100% -> x=1000 (stick endpoint).
    x_cmd = int(round(strength_pct / 100.0 * 1000))
    print(f'\nALL horizontals together: MANUAL_CONTROL x={x_cmd} '
          f'for {duration:.0f}s. Expect all 4 jets toward the STERN and all '
          f'4 PWMs the same side of 1500.')
    end = time.time() + duration
    next_print = 0.0
    while time.time() < end:
        dh.send_frame(master, z=dh.NEUTRAL_Z, x=x_cmd)
        # Passive drain only — single writer/reader on this port.
        msg = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
        if msg is not None and time.time() >= next_print:
            next_print = time.time() + 1.0
            pwm = [getattr(msg, f'servo{m}_raw') for m in dh.HORIZONTAL_MOTORS]
            marks = []
            for m, p in zip(dh.HORIZONTAL_MOTORS, pwm):
                off = p - dh.EXPECT_SERVO_TRIM
                tag = '+' if off > 0 else '-' if off < 0 else '0'
                marks.append(f'm{m}:{p}({tag})')
            print('  ' + '  '.join(marks))
        time.sleep(1.0 / RATE_HZ)
    # Neutral for a moment so thrust actually stops before disarm.
    for _ in range(RATE_HZ):
        dh.send_frame(master, z=dh.NEUTRAL_Z, x=0)
        time.sleep(1.0 / RATE_HZ)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--motors', type=int, nargs='+',
                    default=list(dh.HORIZONTAL_MOTORS),
                    help='which motors to test (default: 1 2 3 4)')
    ap.add_argument('--strength', type=float, default=20.0,
                    help='thrust strength, percent of full power 0-100 '
                         '(default 20; 100 = stick/PWM endpoint — use low '
                         'values dry, ESCs and seals hate full power in air)')
    ap.add_argument('--duration', type=float, default=2.0,
                    help='seconds per burst (default 2)')
    ap.add_argument('--rounds', type=int, default=1,
                    help='full 1..4 sweeps (default 1; use 2+ to re-watch)')
    ap.add_argument('--together', action='store_true',
                    help='spin ALL horizontals at once via a real forward '
                         'MANUAL_CONTROL stick in MANUAL mode (mixer in the '
                         'loop) instead of one-by-one DO_MOTOR_TEST')
    ap.add_argument('--skip-preflight-abort', action='store_true',
                    help='spin even if params mismatch the backup (diagnosis '
                         'only — results are NOT trustworthy for diving)')
    ap.add_argument('--yes', action='store_true', help='skip confirm prompt')
    args = ap.parse_args()

    bad = [m for m in args.motors if m not in dh.HORIZONTAL_MOTORS]
    if bad:
        ap.error(f'motors {bad} are not horizontal thrusters '
                 f'(horizontals are {dh.HORIZONTAL_MOTORS})')
    if not 0 < args.strength <= 100:
        ap.error('--strength is a percent of full power, in (0, 100]')

    master = mt.connect(args.port, args.baud)

    # Param gate first: physical observations only mean something against
    # the known-good param baseline.
    if not dh.verify_horizontal_thruster_directions(master):
        if args.skip_preflight_abort:
            print('\nParam preflight FAILED but --skip-preflight-abort set — '
                  'spinning anyway. Interpret with care; do not dive.')
        else:
            print('\nParam preflight failed — fix params (or rerun with '
                  '--skip-preflight-abort to spin anyway for diagnosis).')
            master.close()
            return 1

    if args.together:
        print(f'\nWILL SPIN ALL horizontals (1-4) TOGETHER: forward '
              f'MANUAL_CONTROL stick ({args.strength:.0f}% power) in '
              f'MANUAL mode, {args.duration:.0f}s x {args.rounds} round(s). '
              f'Vehicle will be ARMED. THRUSTERS WILL SPIN — in water the sub '
              f'WILL drive forward; hold it or keep a tether taut.')
    else:
        print(f'\nWILL SPIN motors {args.motors} one at a time, '
              f'{args.duration:.0f}s each, {args.rounds} round(s), at the PWM '
              f'a pure FORWARD command would give each ({args.strength:.0f}% '
              'power). Vehicle will be ARMED. THRUSTERS WILL SPIN.')
    print('EXPECT: every motor pushes water toward the STERN (airflow to the '
          'rear when dry). A motor blowing the OTHER way vs the rest is '
          'flipped — wiring/ESC vs params.')
    if not args.yes:
        if input('Props clear? type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            master.close()
            return 1

    try:
        if args.together:
            # Mixer path needs a flight mode; MANUAL passes the stick
            # straight through (no EKF/depth sensor in the loop).
            if not sf.set_manual(master):
                return 1
            if not dh.arm(master, True):
                print('Arm failed — check prearm (safety switch) in QGC.')
                return 1
            time.sleep(1)
            for r in range(args.rounds):
                print(f'-- round {r + 1}/{args.rounds} --')
                run_together(master, args.strength, args.duration)
        else:
            if not mt.arm(master, True):
                print('Arm failed — check prearm (safety switch, sensors) '
                      'in QGC.')
                return 1
            time.sleep(1)
            for r in range(args.rounds):
                print(f'-- round {r + 1}/{args.rounds} --')
                for m in args.motors:
                    t = surge_throttle_pct(m, args.strength)
                    print(f'motor {m}: expect water jet toward STERN')
                    mt.run_step(master, m, int(round(t)), args.duration)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        if args.together:
            # Best-effort neutral stick, then disarm.
            dh.send_frame(master, z=dh.NEUTRAL_Z, x=0)
            dh.arm(master, False)
        else:
            mt.motor_test(master, args.motors[0], 50, 0)
            mt.arm(master, False)
        master.close()

    print('\nDone. READ:')
    print('  * all 4 jets toward the stern  -> horizontals coherent, forward '
          'command will drive straight (trim aside).')
    print('  * one jet the other way        -> that motor is flipped '
          'physically. Fix the wiring/ESC, or deliberately change its '
          'MOT_x_DIRECTION in QGC AND the .param backup — then rerun this '
          'AND the param preflight.')
    print('  * no spin one way only         -> ESC missing bidirectional '
          'throttle calibration (see motor_test.py --both).')
    print('  * wrong motor spins            -> SERVOx_FUNCTION mapping.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
