from pymavlink import mavutil
import time
from math import fmod
import cv2

# ========== USER CONFIG ==========
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

SUBMERGE_THRUST = 200    # Z-axis thrust (500 = neutral)
FORWARD_THRUST = 900     # X-axis thrust (500 = neutral)

SUBMERGE_TIME = 4        # Seconds to descend before starting heading correction
FORWARD_TIME = 10         # Seconds to move forward

Kp = 0.5                # Proportional gain for yaw correction
exit_signal = False
key = cv2.waitKey(1)
# =================================


def connect():
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system}, component {master.target_component}")
    return master


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Mode {mode_name} not in available modes: {list(mode_map.keys())}")
    mode_id = mode_map[mode_name]
    master.set_mode(mode_id)
    print(f">>> Mode set to {mode_name}")


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Vehicle armed")
    time.sleep(2)


def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Vehicle disarmed")
    time.sleep(2)


def send_manual_control(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x, y, z, r,
        buttons=0
    )


def get_heading(master):
    while True:
        msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        if msg and hasattr(msg, 'heading'):
            return float(msg.heading)


def heading_error(current, target):
    """Returns shortest angular error (-180 to 180 degrees)"""
    return fmod((current - target + 540), 360) - 180


# ========================== MAIN ==============================

def main(): 
    while (not(key in [27, ord('q'), ord('Q')])): 
        master = connect()

        # Step 1: Set to MANUAL and arm
        set_mode(master, "MANUAL")
        arm(master)

        # Step 2: Submerge
        print("Submerging...")
        start_time = time.time()
        while time.time() - start_time < SUBMERGE_TIME:
            #send_manual_control(master, 0, 0, SUBMERGE_THRUST, 0)
            time.sleep(0.1)
        print("Target depth reached.")

        # Step 3: Capture heading to maintain
        desired_heading = get_heading(master)
        print(f"Desired heading: {desired_heading:.2f}°")

        # Optional: Switch to DEPTH_HOLD mode (uncomment if desired)
        # set_mode(master, "ALT_HOLD")  # or "STABILIZE"/"ALT_HOLD" depending on firmware
        # time.sleep(2)

        # Step 4: Move forward with heading correction
        print("Moving forward while correcting heading...")
        start_time = time.time()
        while time.time() - start_time < FORWARD_TIME:
            current_heading = get_heading(master)
            error = heading_error(current_heading, desired_heading)

            yaw_correction = int(Kp * error)
            yaw_correction = max(min(yaw_correction, 1000), -1000)  # Clamp

            print(f"Heading: {current_heading:.1f}°, Error: {error:.1f}°, Yaw correction: {yaw_correction}")

            send_manual_control(master, FORWARD_THRUST, 0, SUBMERGE_THRUST, yaw_correction)

            time.sleep(0.1)

        # Step 5: Stop movement
        send_manual_control(master, 500, 0, 500, 0)
        print("Movement stopped.")

        # Step 6: Disarm
        disarm(master)
        print("Mission complete.")

def main2(): 
    exit_signal = True
    while (exit_signal):
        if (key in [27, ord('q'), ord('Q')]):
            exit_signal = False

if __name__ == "__main__":
    main()