#!/usr/bin/env python3
"""Isolated action: move FORWARD (ramped) then pause holding depth.

  python3 act_forward.py --speed 0.4 --ramp-up 1.5 --duration 3 --pause 2

See field_common.py for the shared safety notes and tuning flags.
"""

import argparse

from field_common import add_move_args, session


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap)
    args = ap.parse_args()

    with session(confirm_msg='WILL DRIVE FORWARD. Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, _):
        driver.ramp_move('surge_forward', args.speed,
                         args.ramp_up, args.duration, args.ramp_down)
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
