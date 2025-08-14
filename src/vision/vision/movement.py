from pymavlink import mavutil
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

""" 
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("mavlink connected")

mode_id = master.mode_mapping()['MANUAL']
master.set_mode(mode_id)
print("manual")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("armed")
time.sleep(2)
""" 

class MovementSubscriber(Node): 
    def __init__(self):
        super().__init__('movement_subscriber')
        self.latest_info = None
        self.subscriber = self.create_subscription(String, 'movement_info', self.sub_callback, 10)

    def sub_callback(self, msg):
        self.latest_info = msg.data
    
    def get_latest_info(self):
        # Method to access the stored information
        if self.latest_info is not None:
            return self.latest_info
        else: 
            print("No data received yet.")

""" 
class MovementPublisher(Node): 
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher = self.create_publisher(String, 'movement_status', 10)
    
    def publish_detection(self, str):
        msg = String()
        msg.data = str
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {(msg.data)}')

def process_info(subscriber_node, publisher_node):
    info = subscriber_node.get_latest_info()
    if info == "Submerge": 
        print("Submerged yay!")
        publisher_node.publish_detection("Submerge successful")
    if info is not None: 
        print(f"Processing info outside node: {info}")
    else:
        print("No info received yet.")

    """


if __name__ == "__main__":
    rclpy.init()
    node1 = MovementSubscriber()
   #node2 = MovementPublisher()
    import threading 
    spin_thread1 = threading.Thread(target=rclpy.spin, args=(node1,), daemon=True)
    #spin_thread2 = threading.Thread(target=rclpy.spin, args=(node2,), daemon=True)
    spin_thread1.start()
   # spin_thread2.start()

    #import time 
   # while rclpy.ok(): 
        #process_info(node1, node2)
       # time.sleep(1)



