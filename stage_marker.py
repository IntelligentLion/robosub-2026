#!/usr/bin/env python3
"""Stage (movements only): SUBMERGE → pause → move AROUND the marker.

No vision — timed descent, then the open-loop around-marker maneuver
(strafe right → forward → turn left → forward → turn left → forward).
Use stage_marker_detect.py for the detection-gated version.

  python3 stage_marker.py --submerge-duration 4 --speed 0.35 \
                          --leg-duration 3 --turn-duration 6

Tuning:
  --submerge-speed / --submerge-duration   the timed descent
  --speed                                  effort for every maneuver leg
  --leg-duration                           seconds per straight/strafe leg
  --turn-duration                          seconds per ~90° turn
  --ramp-up / --ramp-down / --pause        shared (see field_common.py)
"""

import argparse

from field_common import add_move_args, around_marker, session


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.35)
    ap.add_argument('--submerge-speed', type=float, default=0.4)
    ap.add_argument('--submerge-duration', type=float, default=4.0,
                    help='seconds to descend (timed, no depth sensor)')
    ap.add_argument('--leg-duration', type=float, default=3.0,
                    help='seconds per straight/strafe leg')
    ap.add_argument('--turn-duration', type=float, default=6.0,
                    help='seconds per ~90 deg turn')
    args = ap.parse_args()

    with session(confirm_msg='STAGE: submerge → around marker. Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, _):
        driver.ramp_move('submerge', args.submerge_speed,
                         args.ramp_up, args.submerge_duration, args.ramp_down)
        driver.pause(args.pause)
        around_marker(driver, args.speed, args.ramp_up, args.ramp_down,
                      args.leg_duration, args.turn_duration)
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
