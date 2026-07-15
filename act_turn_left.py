#!/usr/bin/env python3
"""Isolated action: TURN LEFT (yaw CCW, ramped) then pause holding depth.

  python3 act_turn_left.py --speed 0.3 --ramp-up 1.0 --duration 6 --pause 2

`--duration` sets how long the turn holds at speed — tune it to land a ~90°
sweep for your sub. See field_common.py for shared safety notes / flags.
"""

import argparse

from field_common import add_move_args, session


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, duration=6.0)
    args = ap.parse_args()

    with session(confirm_msg='WILL TURN LEFT (yaw CCW). Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, _):
        driver.ramp_move('rotate_ccw', args.speed,
                         args.ramp_up, args.duration, args.ramp_down)
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
