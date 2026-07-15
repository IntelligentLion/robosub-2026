
import argparse
import time

import rclpy
from rclpy.node import Node

from auv_msgs.msg import ObjectDetectionArray

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import Timeout

from control.api import Auv, SubmergeError

RATE_HZ = 10.0
FEET_TO_M = 0.3048

def main():
    # Auv() owns its own rclpy node + init. Context mgr guarantees stop()+cleanup.
    with Auv() as auv:
        try:
            auv.submerge_to_depth(target_depth=0.5)
            auv.move_backward(1, 10)
            auv.turn(0.3, 10, 180)
            auv.move_forward(1, 10)
            auv.move_left(1, 5)
            auv.move_forward(1, 10)
            auv.submerge_to_depth(target_depth=0)
            time.sleep(5)
            auv.submerge_to_depth(0.5)
            auv.turn(0.3, 10, 180)
            auv.move_forward(1, 30)
        except SubmergeError as e:
            print(f"dive aborted: {e}")                # dead Bar02, failed preflight, timeout
        # __exit__ → stop() + destroy_node automatically, even on exception

if __name__ == '__main__':
    main()
