# Modular movement command for thruster control.
#
# Two ways to drive the sub:
#
# 1) Single-primitive verbs (open-loop / scripted moves). Set `command` to a
#    verb below and `speed`/`duration`. The 4-axis fields are ignored.
#    Supported commands:
#      submerge       - move downward          (speed: 0.0-1.0)
#      emerge         - move upward            (speed: 0.0-1.0)
#      surge_forward  - move forward           (speed: 0.0-1.0)
#      surge_backward - move backward          (speed: 0.0-1.0)
#      strafe_left    - move left              (speed: 0.0-1.0)
#      strafe_right   - move right             (speed: 0.0-1.0)
#      rotate_cw      - yaw clockwise          (speed: 0.0-1.0)
#      rotate_ccw     - yaw counter-clockwise  (speed: 0.0-1.0)
#      stop           - halt all movement
#      depth_hold     - maintain current depth
#
# 2) Direct 4-axis setpoint (closed-loop centering / visual servoing). Set
#    `command` = "axes" and populate the four axis fields below. All four are
#    applied simultaneously every control tick (10 Hz) — this is the native
#    MAVLink manual_control form and the path used by autonomous_controller's
#    track_object centering. Fields are signed and clamped to [-1, 1].
#      surge     - forward (+) / backward (-)
#      strafe    - right (+) / left (-)
#      heave     - down (+) / up (-)        (0 = hold depth in ALT_HOLD)
#      yaw_rate  - clockwise (+) / counter-clockwise (-)
#
string command
float32 speed        # 0.0 to 1.0 normalized intensity (verb mode)
float32 duration     # seconds (0 = until next command)
float32 surge        # -1.0..1.0 (axes mode only; +forward)
float32 strafe       # -1.0..1.0 (axes mode only; +right)
float32 heave        # -1.0..1.0 (axes mode only; +down)
float32 yaw_rate     # -1.0..1.0 (axes mode only; +clockwise)
