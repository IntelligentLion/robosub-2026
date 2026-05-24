from pymavlink import mavutil
import time
from math import fmod
import cv2

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Vehicle disarmed")
    time.sleep(2)

def connect():
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system}, component {master.target_component}")
    return master


def main():
    master = connect()

    disarm(master)

if __name__ == "__main__":
    main()