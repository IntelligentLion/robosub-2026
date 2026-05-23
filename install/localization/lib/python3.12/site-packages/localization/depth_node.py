#!/usr/bin/env python3
"""Depth sensing node.

Subscribes to vision/detections (object distances via position.z) and
depth/sub_depth (sub's current depth published by the vision detector).
Publishes depth/info (DepthInfo) at 10 Hz so the mission planner knows
how far to move and where to stop.
"""

import sys
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from auv_msgs.msg import ObjectDetectionArray, DepthInfo


class DepthNode(Node):
    def __init__(self, stop_distance_m: float = 1.5):
        super().__init__('depth_node')
        self._stop_distance_m = stop_distance_m
        self._sub_depth_m = -1.0

        self.create_subscription(
            ObjectDetectionArray,
            'vision/detections',
            self._detections_cb,
            10,
        )
        self.create_subscription(
            Float32,
            'depth/sub_depth',
            self._sub_depth_cb,
            10,
        )

        self._depth_pub = self.create_publisher(DepthInfo, 'depth/info', 10)
        self.create_timer(0.1, self._publish)  # 10 Hz

        self.get_logger().info(
            f'DepthNode started – stop distance: {self._stop_distance_m:.2f} m'
        )

    def _sub_depth_cb(self, msg: Float32):
        self._sub_depth_m = msg.data

    def _detections_cb(self, msg: ObjectDetectionArray):
        for det in msg.detections:
            d = det.position.z
            if d > 0:
                approaching = d <= self._stop_distance_m
                self.get_logger().debug(
                    f'{det.label}: {d:.2f} m '
                    f'(stop@{self._stop_distance_m:.2f} m '
                    f'{"STOP" if approaching else "approach"})'
                )

    def _publish(self):
        msg = DepthInfo()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sub_depth_m = self._sub_depth_m
        msg.stop_distance_m = self._stop_distance_m
        self._depth_pub.publish(msg)


def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--stop_distance', type=float, default=1.5,
        help='Distance in metres at which to stop approaching an object (default: 1.5)',
    )
    args, _ = parser.parse_known_args(sys.argv[1:])

    node = DepthNode(stop_distance_m=args.stop_distance)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
