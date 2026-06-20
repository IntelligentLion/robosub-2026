#!/usr/bin/env python3
"""Stage (movements + detection): SUBMERGE until the GATE is detected (or the
timeout expires) → pause → move forward through the gate.

Spawns the TensorRT detector in-process and descends until the gate label is
seen on vision/detections, or --gate-timeout seconds pass, then drives the
timed forward transit.

  python3 stage_gate_detect.py --submerge-speed 0.4 --gate-timeout 25 \
                               --speed 0.4 --duration 5

Tuning:
  --submerge-speed                 descent effort
  --gate-timeout                   give up on detection, proceed anyway
  --conf                           min detection confidence
  --speed / --duration             forward gate transit
  --ramp-up / --ramp-down / --pause   shared (see field_common.py)
"""

import argparse

from field_common import (add_move_args, descend_until, session,
                          spawn_vision_factory)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.4, duration=5.0)          # forward gate transit
    ap.add_argument('--label', default='gate')
    ap.add_argument('--submerge-speed', type=float, default=0.4)
    ap.add_argument('--gate-timeout', type=float, default=25.0,
                    help='descend until gate detected OR this many seconds')
    ap.add_argument('--conf', type=float, default=0.5, help='min detection conf')
    args = ap.parse_args()

    from field_common import DetectionMonitor, find_node

    with session(spawn_vision_factory(),
                 confirm_msg='STAGE: submerge-until-gate → through gate. '
                             'Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, extra):
        det = find_node(extra, DetectionMonitor)
        descend_until(driver, det, args.label, args.submerge_speed,
                      args.ramp_up, args.ramp_down, args.gate_timeout, args.conf)
        driver.pause(args.pause)
        driver.ramp_move('surge_forward', args.speed,
                         args.ramp_up, args.duration, args.ramp_down)
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
