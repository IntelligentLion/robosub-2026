from control.api import Auv, SubmergeError

def main():
    # Auv() owns its own rclpy node + init. Context mgr guarantees stop()+cleanup.
    with Auv() as auv:
        try:
            auv.submerge_to_depth(target_depth=0.5)   # blocks until 'hold'
            auv.move_forward(speed=1, duration=5)   # heading held through the veer
            auv.turn(yaw_rate=1, degrees=90)        # re-captures new heading
            auv.move_left(speed=1, duration = 5)
            auv.move_right(speed=1, duration=5)
            auv.move_left(speed=1, duration=5)
            auv.turn(yaw_rate=-1, degrees=90)       # back the other way
            #auv.move_forward(speed=0.4, duration=5)
            auv.stop()
        except SubmergeError as e:
            print(f"dive aborted: {e}")                # dead Bar02, failed preflight, timeout
        # __exit__ → stop() + destroy_node automatically, even on exception

if __name__ == '__main__':
    main()
