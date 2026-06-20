#!/usr/bin/env python3
"""Isolated action: CENTER on the gate (closed-loop yaw), then pause.

Spawns the TensorRT detector in-process, reads the gate's normalised image
centre-x from vision/detections, and yaws until the gate is centred (within
--tol) or --timeout expires. Yaw effort is proportional to the centring error,
clamped to [--min-speed, --speed].

  python3 act_center_gate.py --speed 0.3 --gain 0.6 --tol 0.08

See field_common.py for shared safety notes. Needs the ZED/camera the detector
reads from to be connected.
"""

import argparse
import time

from field_common import (RATE_HZ, DetectionMonitor, add_move_args,
                           find_node, session, spawn_vision_factory)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.3, duration=0.0, pause=2.0)
    ap.add_argument('--label', default='gate', help='detection label to centre on')
    ap.add_argument('--gain', type=float, default=0.6,
                    help='yaw effort per unit centre-x error (default 0.6)')
    ap.add_argument('--min-speed', type=float, default=0.1,
                    help='min yaw effort while correcting (default 0.1)')
    ap.add_argument('--tol', type=float, default=0.08,
                    help='|centre-x - 0.5| under which centred (default 0.08)')
    ap.add_argument('--conf', type=float, default=0.5, help='min detection conf')
    ap.add_argument('--timeout', type=float, default=20.0,
                    help='give up centring after this many seconds')
    args = ap.parse_args()

    with session(spawn_vision_factory(),
                 confirm_msg='WILL YAW to centre on the gate. Thrusters WILL spin.',
                 skip_confirm=args.yes) as (driver, extra):
        det = find_node(extra, DetectionMonitor)
        period = 1.0 / RATE_HZ
        deadline = time.time() + args.timeout
        centred = False
        while time.time() < deadline:
            d = det.best(args.label, args.conf)
            if d is None:
                driver.send('depth_hold', 0.0)        # hold, wait for a detection
                time.sleep(period)
                continue
            error = d.position.x - 0.5                # +ve → target is to the right
            if abs(error) <= args.tol:
                driver.send('depth_hold', 0.0)
                centred = True
                driver.get_logger().info(
                    f'✓ Centred ({args.label} cx={d.position.x:.2f}).')
                break
            effort = max(args.min_speed, min(args.speed, args.gain * abs(error)))
            driver.send('rotate_cw' if error > 0 else 'rotate_ccw', effort)
            time.sleep(period)
        if not centred:
            driver.get_logger().warn('Centring timed out.')
        driver.pause(args.pause)


if __name__ == '__main__':
    main()
