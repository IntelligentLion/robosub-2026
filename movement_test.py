from control.api import Auv, SubmergeError
import time

def main():
    # Auv() owns its own rclpy node + init. Context mgr guarantees stop()+cleanup.
    with Auv() as auv:
        try:
            auv.submerge_to_depth(target_depth=0.5)
            print("done with submerge")   # blocks until 'hold'
             # heading held through the veer
            for i in range(0, 30):
                auv.move_forward(speed=1, duration=0.1)
                auv.stop()
            #time.sleep(1)
            #auv.move_left(speed=1, duration = 5)
            #auv.move_left(speed=1, duration=5)
            #auv.turn(yaw_rate=0.2, degrees=0)
            #print("turn left 90 degrees done")
            #time.sleep(1)
            #auv.surface()
            print("resurfaced!")
            #auv.move_forward(speed=0.4, duration=5)
        
            auv.stop()
            auv.close()
            print("ros node destroyed")
        except SubmergeError as e:
            print(f"dive aborted: {e}")                # dead Bar02, failed preflight, timeout
        # __exit__ → stop() + destroy_node automatically, even on exception

if __name__ == '__main__':
    main()
