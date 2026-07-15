

















#!/usr/bin/env python3
"""Hardcoded timed course runner — standalone, flat procedural style.

Drives the AUV through a fixed, open-loop course:

    1. submerge 5 ft
    2. forward 11 ft
    3. strafe right 4 ft
    4. forward 4 ft
    5. strafe left 7 ft
    6. turn 180° (to face the starting point)
    7. forward 5 ft
    8. strafe left 4 ft
    9. forward 14 ft

Independent of field_common.py / the ThrusterController node — this script owns
the Pixhawk serial link directly via MAVLink manual_control. Distances are hit
by TIME, not sensing: each leg runs its axis at a raw thrust for a set number of
seconds. Tune any single leg with the THRUST / TIME constants below.

A lightweight ZED heading-hold "drift trim" keeps the legs straight: it latches
the heading and applies a small, clamped yaw correction on non-turn legs so the
sub doesn't curve. Needs vslam/odometry on the network; if absent it runs pure
open-loop after one warning. Set USE_TRIM = False to skip it entirely.

manual_control axes:  x surge(+fwd)  y strafe(+right)  z depth(500 neutral,
<500 deeper)  r yaw(+CW)  — same mapping as the production thruster driver.

⚠ THRUSTERS WILL SPIN. Stop thruster_node first (single serial owner). Clear the
props / run on a tether, keep the kill switch reachable. Ctrl+C → stop+disarm.

  python3 run_course.py
"""

from pymavlink import mavutil
import math
import threading
import time

#from dropper import Dropper

# ============================ CONFIG ========================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

ARM_SETTLE_TIME = 3       # seconds to wait after arming before driving
FLIGHT_MODE = 'ALT_HOLD'  # autopilot holds depth between/after the submerge
CONFIRM = True            # ask "go" before arming; set False to run immediately

# ── Per-leg raw thrust + time. Edit freely; each leg is independent. ────────
#    Surge/strafe/yaw thrust: -1000..1000 (0 neutral). Submerge z: <500 = down.

# 1) submerge 5 ft        (z below 500 pushes down)
SUBMERGE_Z       = 150
SUBMERGE_TIME    = 3

# 2) forward 11 ft        (+x forward)
FWD1_THRUST      = 500
FWD1_TIME        = 10

# 3) strafe right 4 ft    (+y right)
STRAFE_R_THRUST  = 500
STRAFE_R_TIME    = 5

# 4) forward 4 ft
FWD2_THRUST      = 700
FWD2_TIME        = 6

# 5) strafe left 7 ft     (-y left)
STRAFE_L1_THRUST = 400
STRAFE_L1_TIME   = 7.5

# 6) turn 180°            (+r yaw CW)
TURN_THRUST      = 300
TURN_TIME        = 6      # tune until the sub sweeps a clean 180°

# 7) forward 5 ft
FWD3_THRUST      = 400
FWD3_TIME        = 5

# 8) strafe left 4 ft
STRAFE_L2_THRUST = 400
STRAFE_L2_TIME   = 4.5

# 9) forward 14 ft
FWD4_THRUST      = 400
FWD4_TIME        = 18

# ── Dropper (marker release servo on AUX1) ──────────────────────────────────
DROPPER_ENABLED = True   # False → never touch the dropper this run
DROP_AFTER_LEG = 1       # release markers after this leg number (1-based);
                         # None = carry them the whole course

NEUTRAL_Z = 500          # centred vertical stick (ALT_HOLD holds depth)
SETTLE_TIME = 0       # neutral depth-hold between legs (0 = chain directly)
RATE_HZ = 10             # manual_control + heartbeat cadence
PERIOD = 10.0 / RATE_HZ

# ── ZED heading-hold drift trim ─────────────────────────────────────────────
USE_TRIM = True         # False → pure open-loop, ignore ZED
KP_YAW = 0.1             # yaw effort (0..1) per radian of heading error
TRIM_MAX = 0.4          # hard cap on trim effort — keep small
TRIM_YAW_SIGN = -1.0     # flip to -1.0 if heading correction goes the wrong way
VERTICAL_AXIS = 'y'      # ZED world-up axis (Y_UP → heading is about Y)
ZED_STALE_S = 1.0        # treat the ZED fix as lost after this long

# Course: (label, x, y, z, r, seconds, is_turn). z=None → hold NEUTRAL_Z.
COURSE = [
    ('1. submerge 5 ft',     0,                0,                 SUBMERGE_Z, 0,           SUBMERGE_TIME,  False),
    #('6. turn 180°',         0,                0,                 400,       TURN_THRUST, 4.5,      True),
    ('2. forward 11 ft',     FWD1_THRUST,      0,                150,       0,           FWD1_TIME,      False),
    #('3. strafe right 4 ft', 0,                STRAFE_R_THRUST,   SUBMERGE_Z,       0,           STRAFE_R_TIME,  False),
 #   ('4. forward 4 ft',      FWD2_THRUST,      0,                 170,       0,           FWD2_TIME,      False),
  #  ('5. strafe left 7 ft',  0,                -STRAFE_L1_THRUST, SUBMERGE_Z,       0,           STRAFE_L1_TIME, False)
    #('6. turn 180°',         0,                0,                 400,       TURN_THRUST, 4.5,      True),
    #('6. turn 180°',         0,                0,                 400,       TURN_THRUST, 4.5,      True),
    ('2. forward 11 ft',     FWD1_THRUST,      0,                300,       0,           FWD1_TIME,      False)
   # ('7. forward 5 ft',      FWD3_THRUST,      0,                 190,       0,           FWD3_TIME,      False),
   # ('8. strafe left 4 ft',  0,                -STRAFE_L2_THRUST, 190,       0,           STRAFE_L2_TIME, False),
   # ('9. forward 14 ft',     FWD4_THRUST,      0,                 210,       0,           FWD4_TIME,      False),
]
# ============================================================================

# ── ZED heading source (background thread; degrades to "no fix" cleanly) ────
_heading = None
_heading_t = None
_heading_lock = threading.Lock()
_zed_executor = None
_zed_node = None


def start_heading_source():
    """Spin a minimal vslam/odometry subscriber in the background, if possible."""
    global _zed_executor, _zed_node
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from nav_msgs.msg import Odometry
    except Exception as exc:
        print(f"[trim] ZED/ROS unavailable ({exc}) — running pure open-loop.")
        return
    try:
        rclpy.init()
    except Exception:
        pass

    class _OdomNode(Node):
        def __init__(self):
            super().__init__('course_heading_source')
            self.create_subscription(Odometry, 'vslam/odometry', self._cb, 10)

        def _cb(self, msg):
            q = msg.pose.pose.orientation
            update_heading(q.x, q.y, q.z, q.w)

    try:
        _zed_node = _OdomNode()
        _zed_executor = SingleThreadedExecutor()
        _zed_executor.add_node(_zed_node)
        threading.Thread(target=_zed_executor.spin, daemon=True).start()
        print("[trim] ZED heading source up — subscribing vslam/odometry.")
    except Exception as exc:
        print(f"[trim] failed to start ZED subscriber ({exc}) — pure open-loop.")


def stop_heading_source():
    global _zed_executor, _zed_node
    try:
        if _zed_executor is not None:
            _zed_executor.shutdown()
        if _zed_node is not None:
            _zed_node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


def update_heading(x, y, z, w):
    """Store the latest heading (rad) from a quaternion."""
    global _heading, _heading_t
    h = heading_about_axis(x, y, z, w, VERTICAL_AXIS)
    if h is None or not math.isfinite(h):
        return
    with _heading_lock:
        _heading = h
        _heading_t = time.monotonic()


def get_heading():
    """Latest heading (rad), or None if no fresh ZED fix."""
    with _heading_lock:
        if _heading_t is None or time.monotonic() - _heading_t > ZED_STALE_S:
            return None
        return _heading


def heading_about_axis(x, y, z, w, axis):
    """Heading (rad) about the world-up axis from a quaternion (Y_UP → 'y').

    Only a continuous, consistent angle is needed (reference is latched per
    leg), so the exact convention isn't critical — TRIM_YAW_SIGN sets the sense.
    """
    if axis == 'y':
        return math.atan2(2.0 * (w * y + x * z), 1.0 - 2.0 * (y * y + z * z))
    if axis == 'z':
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def wrap(angle):
    """Wrap radians to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_trim(target_heading):
    """Clamped yaw correction (raw -1000..1000) to hold target_heading, or 0."""
    if not USE_TRIM or target_heading is None:
        return 0
    h = get_heading()
    if h is None:
        return 0
    trim = TRIM_YAW_SIGN * KP_YAW * wrap(target_heading - h)
    trim = max(-TRIM_MAX, min(TRIM_MAX, trim))
    return round(trim * 1000)


# ── MAVLink helpers ─────────────────────────────────────────────────────────

def connect():
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system}, "
          f"component {master.target_component}")
    return master


def set_mode(master, mode_name):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Mode {mode_name} not in available modes: "
                         f"{list(mode_map.keys())}")
    master.set_mode(mode_map[mode_name])
    print(f">>> Mode set to {mode_name}")


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Vehicle armed")
    time.sleep(ARM_SETTLE_TIME)


def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Vehicle disarmed")
    time.sleep(2)


def send_manual_control(master, x, y, z, r):
    """One manual_control frame (clamped) + a GCS heartbeat (failsafe guard)."""
    master.mav.manual_control_send(
        master.target_system,
        max(-1000, min(1000, int(x))),
        max(-1000, min(1000, int(y))),
        max(0,     min(1000, int(z))),
        max(-1000, min(1000, int(r))),
        buttons=0
    )
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def drive(master, label, x, y, z, r, seconds, is_turn, target_heading):
    """Hold (x, y, z, r) at RATE_HZ for `seconds`, adding heading trim.

    z=None → NEUTRAL_Z (ALT_HOLD holds depth). Turn legs are NOT heading-trimmed
    (we drive yaw on purpose). Returns the heading to hold afterwards.
    """
    z = NEUTRAL_Z if z is None else z
    trimmed = USE_TRIM and not is_turn
    print(f"\n>>> {label}  (x={x} y={y} z={z} r={r} time={seconds}s"
          + (" +heading-hold" if trimmed else "") + ")")

    start = time.time()
    while time.time() - start < seconds:
        r_cmd = r + (yaw_trim(target_heading) if trimmed else 0)
        send_manual_control(master, x, y, z, r_cmd)
        time.sleep(PERIOD)

    if is_turn and USE_TRIM:                       # re-latch new bearing
        h = get_heading()
        if h is not None:
            target_heading = h
            print(f"  [trim] re-latched heading after turn = "
                  f"{math.degrees(h):.0f}°")
    return target_heading


def settle(master, seconds, target_heading):
    """Neutral horizontal + held depth between legs, keeping heading."""
    start = time.time()
    while time.time() - start < seconds:
        send_manual_control(master, 0, 0, NEUTRAL_Z, yaw_trim(target_heading))
        time.sleep(PERIOD)


if __name__ == "__main__":
    # Step 0: pre-flight summary + safety confirm.
    total_t = sum(seg[5] for seg in COURSE) + SETTLE_TIME * (len(COURSE) - 1)
    print("\nWILL RUN THE FULL COURSE — THRUSTERS WILL SPIN, SUB WILL MOVE.")
    print(f"  {len(COURSE)} legs, ~{total_t:.0f}s of motion, "
          f"heading-trim {'ON' if USE_TRIM else 'OFF'}.")
    if CONFIRM:
        if input('Props clear? type "go" to run: ').strip().lower() != "go":
            print("Aborted.")
            raise SystemExit(1)

    # Step 1: start the ZED heading source (optional).
    if USE_TRIM:
        start_heading_source()
        time.sleep(0.5)

    # Step 2: connect, prep the dropper (works disarmed), set mode, arm.
    master = connect()
   # dropper = None
   # if DROPPER_ENABLED:
      #  dropper = Dropper(master)
     #   if dropper.prepare():
    #        dropper.hold()          # grip the markers before we get wet
   #     else:
  #          print("[dropper] prepare failed — drops disabled this run")
 #           dropper = None
#    set_mode(master, FLIGHT_MODE)
    arm(master)

    # Step 3: latch the starting heading so submerge + leg 1 hold it.
    target_heading = get_heading() if USE_TRIM else None
    if target_heading is not None:
        print(f"Start heading latched = {math.degrees(target_heading):.0f}°")

    # Step 4: run each leg in order, settling between them.
    try:
        for i, (label, x, y, z, r, seconds, is_turn) in enumerate(COURSE):
            target_heading = drive(master, label, x, y, z, r, seconds,
                                    is_turn, target_heading)

#            if dropper is not None and DROP_AFTER_LEG == i + 1:
 #               print("\n>>> DROP markers")
                # keepalive holds depth/heading so the pilot-input
                # failsafe can't fire during the release wait
   #             dropper.drop(keepalive=lambda: send_manual_control(
    #                master, 0, 0, NEUTRAL_Z, yaw_trim(target_heading)))

            if i < len(COURSE) - 1:
                settle(master, SETTLE_TIME, target_heading)
        time.sleep(20)
        print("\nCourse complete.")
        
    except KeyboardInterrupt:
        print("\nInterrupted — stopping.")
    
    
    # Step 5: neutral, park dropper, disarm, clean up.
    finally:
        for _ in range(5):
            send_manual_control(master, 0, 0, NEUTRAL_Z, 0)
            time.sleep(0.05)
     #   if dropper is not None:
      #      try:
       #         dropper.rest()   # never leave a stale PWM latched on the pin
        #    except Exception as exc:
         #       print(f"[dropper] rest failed: {exc}")
        disarm(master)
        stop_heading_source()
    print("Mission complete.")
