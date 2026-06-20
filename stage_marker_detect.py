#!/usr/bin/env python3
"""Stage (movements + detection): SUBMERGE until the MARKER is detected (or the
timeout expires) → pause → move around the marker.

Spawns the TensorRT detector in-process and descends until the marker label is
seen on vision/detections, or --marker-timeout seconds pass, then runs the
open-loop around-marker maneuver.

  python3 stage_marker_detect.py --submerge-speed 0.4 --marker-timeout 25 \
                                 --speed 0.35 --leg-duration 3 --turn-duration 6

Tuning:
  --submerge-speed                 descent effort
  --marker-timeout                 give up on detection, proceed anyway
  --conf                           min detection confidence
  --speed                          effort for every maneuver leg
  --leg-duration / --turn-duration maneuver leg / turn seconds
  --ramp-up / --ramp-down / --pause   shared (see field_common.py)
"""

import argparse

from field_common import (DetectionMonitor, add_move_args, around_marker,
                          descend_until, find_node, session,
                          spawn_vision_factory)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.35)
    ap.add_argument('--label', default='marker')
    ap.add_argument('--submerge-speed', type=float, default=0.4)
    ap.add_argument('--marker-timeout', type=float, default=25.0,
                    help='descend until marker detected OR this many seconds')
    ap.add_argument('--conf', type=float, default=0.5, help='min detection conf')
    ap.add_argument('--leg-duration', type=float, default=3.0,
                    help='seconds per straight/strafe leg')
    ap.add_argument('--turn-duration', type=float, default=6.0,
                    help='seconds per ~90 deg turn')
    args = ap.parse_args()

    with session(spawn_vision_factory(),
                 confirm_msg='STAGE: submerge-until-marker → around marker. '
                             'Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, extra):
        det = find_node(extra, DetectionMonitor)
        descend_until(driver, det, args.label, args.submerge_speed,
                      args.ramp_up, args.ramp_down, args.marker_timeout, args.conf)
        driver.pause(args.pause)
        around_marker(driver, args.speed, args.ramp_up, args.ramp_down,
                      args.leg_duration, args.turn_duration)
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
