#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from auv_msgs.msg import ObjectDetectionArray, DepthInfo

class IntegrationTestNode(Node):
    """
    A simple test node to verify the integration between the vision
    and localization pipelines. It listens to the critical topics
    where VSLAM-grounded detection and depth information are published.
    """
    def __init__(self):
        super().__init__('integration_test_node')
        
        self.get_logger().info("Integration Test Node Started.")
        self.get_logger().info("Listening to Vision and Localization topics...")

        # Listen to grounded vision detections
        self.create_subscription(
            ObjectDetectionArray,
            'vision/detections',
            self.detections_callback,
            10
        )

        # Listen to Raw VSLAM positional tracks 
        self.create_subscription(
            Float32,
            'depth/sub_depth',
            self.sub_depth_callback,
            10
        )

        # Listen to the correlated movement/localization info
        self.create_subscription(
            DepthInfo,
            'depth/info',
            self.depth_info_callback,
            10
        )

    def detections_callback(self, msg):
        self.get_logger().info(f"[VISION] Received {len(msg.detections)} grounded detections.")
        for d in msg.detections:
            self.get_logger().info(f"   -> {d.label} | Conf: {d.confidence:.2f} | Z-Depth: {d.position.z:.2f}m")

    def sub_depth_callback(self, msg):
        self.get_logger().info(f"[VSLAM] Sub Depth Update: {msg.data:.3f}m")

    def depth_info_callback(self, msg):
        self.get_logger().info(f"[LOCALIZATION] Depth Info | Sub Depth: {msg.sub_depth_m:.3f}m | Stop Dist: {msg.stop_distance_m:.3f}m")

def main(args=None):
    rclpy.init(args=args)
    node = IntegrationTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
