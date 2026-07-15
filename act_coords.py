#!/usr/bin/env python3
"""Check x,y,z coordinates — read and print the ZED positional-tracking pose.

Spawns the ZED vslam node in-process and prints the WORLD-frame position from
vslam/odometry at ~2 Hz. Does NOT arm or drive any thruster — pure readout, so
it is safe to run on the bench to confirm the camera tracks before a depth or
movement test.

  python3 act_coords.py              # print until Ctrl+C
  python3 act_coords.py --once       # print one fix and exit
  python3 act_coords.py --external   # subscribe to a vslam node you launch

ZED Y_UP world frame → Y is vertical (depth), X/Z are horizontal.
"""

import argparse
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from field_common import CoordMonitor


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--once', action='store_true',
                    help='print the first fix then exit')
    ap.add_argument('--external', action='store_true',
                    help='do NOT spawn the vslam node; subscribe to an existing one')
    ap.add_argument('--rate', type=float, default=2.0, help='print rate Hz')
    args = ap.parse_args()

    rclpy.init()
    coords = CoordMonitor()
    nodes = [coords]
    if not args.external:
        from localization.vslam_node import VSLAMZedNode
        nodes.append(VSLAMZedNode())
    else:
        print('[external] subscribing to vslam/odometry — start it with:\n'
              '    ros2 run localization vslam_node\n')

    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('Waiting for vslam/odometry…  (Ctrl+C to stop)')
    try:
        period = 1.0 / args.rate
        while rclpy.ok():
            if coords.have_fix():
                print(f'x={coords.x:+.3f}  y={coords.y:+.3f} (vertical)  '
                      f'z={coords.z:+.3f}  m')
                if args.once:
                    break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
