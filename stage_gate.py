#!/usr/bin/env python3
"""Stage (movements only): SUBMERGE → pause → move through the GATE.

No vision — the descent is purely timed. Use this to rehearse the gate run's
motion profile and tune speeds/ramps before adding detection
(see stage_gate_detect.py).

  python3 stage_gate.py --submerge-speed 0.4 --submerge-duration 4 \
                        --speed 0.4 --duration 5

Tuning:
  --submerge-speed / --submerge-duration   the timed descent
  --speed / --duration                     the forward gate transit
  --ramp-up / --ramp-down / --pause        shared (see field_common.py)
"""

import argparse

from field_common import add_move_args, session


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.4, duration=5.0)          # forward gate transit
    ap.add_argument('--submerge-speed', type=float, default=0.4)
    ap.add_argument('--submerge-duration', type=float, default=4.0,
                    help='seconds to descend (timed, no depth sensor)')
    args = ap.parse_args()

    with session(confirm_msg='STAGE: submerge → through gate. Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, _):
        driver.ramp_move('submerge', args.submerge_speed,
                         args.ramp_up, args.submerge_duration, args.ramp_down)
        driver.pause(args.pause)
        driver.ramp_move('surge_forward', args.speed,
                         args.ramp_up, args.duration, args.ramp_down)
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
