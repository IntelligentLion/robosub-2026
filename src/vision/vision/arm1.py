from pymavlink import mavutil
import time


SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
SUBMERGE_THRUST = 225  # 500 neutral, lower pushes down
FORWARD_THRUST = 900   # 500 neutral, higher moves forward
SUBMERGE_TIME = 4      # seconds to submerge before switching to depth hold
FORWARD_TIME = 12      # seconds to run forward in depth hold
# ======================

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
    time.sleep(10)

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


if __name__ == "__main__":
    master = connect()

    # Step 1: Set to MANUAL and arm
    set_mode(master, "MANUAL")
    arm(master)

    # Step 2: Submerge in manual
    print("Submerging...")
    start_time = time.time()
    while time.time() - start_time < SUBMERGE_TIME:
        #send_manual_control(master, 0, 0, SUBMERGE_THRUST, 0)
        time.sleep(0.1)
    print("Target depth reached.")

    # Step 3: Switch to DEPTH_HOLD
    #set_mode(master, "STABILIZE")
    #time.sleep(2)  # Let controller stabilize

    # Step 4: Move forward in depth hold
    print("Moving forward")
    start_time = time.time()
    while time.time() - start_time < FORWARD_TIME:
        send_manual_control(master, FORWARD_THRUST, 0, SUBMERGE_THRUST, 0)
        
        time.sleep(0.1)

    # Step 5: Stop movement
    #send_manual_control(master, 0, 0, 500, 0)
    #print("Movement stopped.")

    # Step 6: Disarm
    disarm(master)
    print("Mission complete.")