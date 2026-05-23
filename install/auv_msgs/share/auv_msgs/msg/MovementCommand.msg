# Modular movement command for thruster control
#
# Supported commands:
#   submerge       - move downward          (speed: 0.0–1.0)
#   emerge         - move upward            (speed: 0.0–1.0)
#   surge_forward  - move forward           (speed: 0.0–1.0)
#   surge_backward - move backward          (speed: 0.0–1.0)
#   strafe_left    - move left              (speed: 0.0–1.0)
#   strafe_right   - move right             (speed: 0.0–1.0)
#   rotate_cw      - yaw clockwise          (speed: 0.0–1.0)
#   rotate_ccw     - yaw counter-clockwise  (speed: 0.0–1.0)
#   stop           - halt all movement
#   depth_hold     - maintain current depth
#
string command
float32 speed        # 0.0 to 1.0 normalized intensity
float32 duration     # seconds (0 = until next command)
