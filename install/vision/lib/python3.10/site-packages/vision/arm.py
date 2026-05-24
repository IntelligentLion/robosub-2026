from pymavlink import mavutil
import time

print("Starting Countdown")
time.sleep(7)


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

print("manual thrust")

print("submerging")
for i in range(50):
    master.mav.manual_control_send(
        master.target_system, 
        x=0, 
        y=0, 
        z=700, 
        r = 0, 
        buttons = 0
    )
    time.sleep(0.1)
print("submerged")


print("running")
for i in range(10):
    master.mav.manual_control_send(
        master.target_system,
        x=1000,     # Forward (0 to -1000 is backwards) (0 to 1000 is forward)
        y=0,      # (0 to -1000 is left) (O to 1000 is right)
        z=500,     # Up 0 to 500 down, 500 to 1000 up, 500 neutral
        r=0,    # Yaw (0 to -1000 is counterclockwise) (0 to 1000 is clockwise)
        buttons=0 
    )
    time.sleep(0.1)
print("done")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("disarmed")
time.sleep(2)