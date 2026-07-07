import time
from pymavlink import mavutil

def set_servo_pwm(master, servo_instance, pwm_value):
    """
    Sends a MAVLink command to set a specific servo channel's PWM.
    
    :param master: The established MAVLink connection object.
    :param servo_instance: The pin/servo number configured on the autopilot.
    :param pwm_value: Target PWM in microseconds (typically 1000 to 2000).
    """
    master.mav.command_long_send(
        master.target_system,                # Target system ID
        master.target_component,             # Target component ID
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # The MAVLink command
        0,                                   # Confirmation (0 = first transmission)
        servo_instance,                      # Param 1: Servo instance/number
        pwm_value,                           # Param 2: PWM pulse-width
        0, 0, 0, 0, 0                        # Param 3-7: Unused parameters
    )
    print(f"Sent PWM {pwm_value} to servo instance {servo_instance}")

# 1. Connect to the flight controller (Change connection string as needed)
# For USB/Serial: 'COM3' or '/dev/ttyACM0'. For UDP: 'udpin:localhost:14550'
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# 2. Wait for a heartbeat to identify the autopilot IDs
print("Waiting for autopilot heartbeat...")
master.wait_heartbeat()
print(f"Connected to System {master.target_system}, Component {master.target_component}")

# 3. Command the servo (Example: Pin/Instance 9 to 1500 microseconds)
# Ensure the chosen channel is mapped as a 'Servo' and not a 'Motor' in your firmware
set_servo_pwm(master, servo_instance=9, pwm_value=1500)