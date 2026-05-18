import rclpy
from rclpy.node import Node
from auv_msgs.msg import BehaviorStatus


class BehaviorStatusSubscriber(Node):
    def __init__(self):
        super().__init__('vision_behavior_status_listener')
        self.subscription = self.create_subscription(
            BehaviorStatus,
            'behavior_status',
            self.status_callback,
            10,
        )

    def status_callback(self, msg: BehaviorStatus):
        self.get_logger().info(
            f"Behavior update: action={msg.action_name}, status={msg.status}, reason={msg.reason}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorStatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
