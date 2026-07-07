#!/usr/bin/env python3
"""Send a high-level centering goal to autonomous_controller.

This is the pool-test entry point for the vision centering framework
(see docs/CENTERING.md). It does NOT drive thrusters itself — it publishes
``auv_msgs/NavigationCommand(mode=track_object, target_label, speed,
approach_dist)`` at 1 Hz so the running ``autonomous_controller`` stays in
centering mode. Ctrl+C sends ``idle`` and exits.

Requires (already running):
  - thruster_controller   (owns the serial port — WILL drive the real sub)
  - vision_node          (publishes vision/detections)
  - autonomous_controller

  python3 test_centering.py --label gate --speed 0.3 --approach 1.5
  python3 test_centering.py --label large_opening --approach 1.2   # torpedo stub
"""

import argparse
import rclpy
from rclpy.node import Node
from auv_msgs.msg import NavigationCommand


class NavGoalSender(Node):
    def __init__(self, label, speed, approach):
        super().__init__('centering_goal_sender')
        self._label = label
        self._speed = speed
        self._approach = approach
        self._pub = self.create_publisher(NavigationCommand, 'navigation_command', 10)
        self._timer = self.create_timer(1.0, self._publish)
        self.get_logger().info(
            f'Sending track_object label="{label}" speed={speed:.2f} '
            f'approach={approach:.2f}m at 1 Hz. Ctrl+C to stop.')

    def _publish(self):
        msg = NavigationCommand()
        msg.mode = 'track_object'
        msg.target_label = self._label
        try:
            msg.speed = float(self._speed)
            msg.approach_dist = float(self._approach)
        except (TypeError, ValueError):
            msg.speed = 0.3
            msg.approach_dist = 1.5
        self._pub.publish(msg)

    def send_idle(self):
        msg = NavigationCommand()
        msg.mode = 'idle'
        self._pub.publish(msg)
        self.get_logger().info('Sent idle.')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--label', default='gate',
                    help='detection label to center on (default: gate)')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='max effort 0-1 (default 0.3)')
    ap.add_argument('--approach', type=float, default=1.5,
                    help='standoff distance in metres (default 1.5)')
    args = ap.parse_args()

    rclpy.init()
    node = NavGoalSender(args.label, args.speed, args.approach)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_idle()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
