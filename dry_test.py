#!/usr/bin/env python3
"""Dry-test console for the RoboSub 2026 stack — NO hardware, NO real movement.

This is a bench / dry-land test harness. It does two things at once:

1.  **Print-only thruster stand-in.**  It subscribes to ``movement_command``
    (the exact topic the real ``mavlink_thruster_control/thruster_node``
    consumes) and, instead of talking to a Pixhawk, *prints* what the thrusters
    would do: the four ArduSub ``manual_control`` axes (surge/strafe/heave/yaw)
    and a plain-English description of the direction + which thruster group
    drives it.  Run the full autonomy stack against this and you can watch the
    behavior tree / autonomous_controller / prequalification node drive the sub
    on dry land with zero risk of spinning a prop.

2.  **Interactive subsystem tester.**  A menu lets you exercise every part of
    the sub *individually* without the rest of the stack:
      * movement primitives  (each command, every direction)
      * a full thruster/direction sweep
      * vision   — inject fake ``vision/detections`` to trigger the planner
      * depth    — publish fake ``depth/sub_depth`` + ``depth/info``
      * pose     — publish fake ``localization/pose``
      * safety   — publish fake battery % and toggle the leak flag
      * navigation — send ``navigation_command`` to the autonomous_controller
      * monitor  — watch ``navigation_command`` coming out of the behavior tree

Every outbound thruster command is rendered as text, so nothing ever moves.

Usage
-----
  # 1. Source the workspace so auv_msgs is importable
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  # 2. Run the console
  python3 dry_test.py                 # interactive menu
  python3 dry_test.py --sweep         # auto-cycle every direction, then exit
  python3 dry_test.py --monitor       # passive: just print what the stack commands
  python3 dry_test.py --camera        # camera-only: real detections + print thrusters

Camera-only dry test (ONLY the camera plugged in)
-------------------------------------------------
Verify the vision pipeline and the detection-based decision logic before the
thrusters are wired up:
  Terminal A:  python3 dry_test.py --camera          # detections in, thrusters printed
  Terminal B:  ros2 run vision detector ...          # real camera + detector
  Terminal C:  ros2 run control autonomous_controller   # (or a mission node)
You see the REAL detections the camera produces AND the thruster commands the
planner generates from them — so you confirm both "does detection work?" and
"do the conditions based on detections work?" without anything moving. Later,
with the Pixhawk connected, drop the --camera flag (or run the real
thruster_node) and the exact same setup tests the full pipeline end to end,
real detections driving real, correct thruster movement.

Typical dry run of the WHOLE sub:
  Terminal A:  python3 dry_test.py --monitor      # the print-only "thrusters"
  Terminal B:  ros2 run control autonomous_controller
  Terminal C:  ros2 run bt_mission bt_executor    # the behavior tree
  Terminal A's menu (or another dry_test.py) can inject fake vision/depth/pose
  so the behavior tree advances and you watch every command it would send.
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from auv_msgs.msg import (
    DepthInfo,
    MovementCommand,
    NavigationCommand,
    ObjectDetection,
    ObjectDetectionArray,
)


# ─── Thruster mapping (mirrors mavlink_thruster_control/thruster_node) ───────
#
# The real node maps each MovementCommand onto ArduSub `manual_control` axes:
#   x  surge  -1000..1000   (forward +, backward -)
#   y  strafe -1000..1000   (right   +, left     -)
#   z  heave     0..1000    (500 neutral; <500 down, >500 up)
#   r  yaw    -1000..1000   (cw      +, ccw      -)
# On a standard 8-thruster vectored frame, x/y/r are produced by the 4 vectored
# horizontal thrusters and z by the 4 vertical thrusters. We reproduce the exact
# arithmetic so the printed axes match what the Pixhawk would actually receive.

def decode_command(cmd: str, speed: float):
    """Return (x, y, z, r, human_description) for a MovementCommand.

    Mirrors thruster_node's submerge/emerge/surge/strafe/rotate/stop/depth_hold
    math exactly so the printed axes equal the real MAVLink output.
    """
    cmd = cmd.lower().strip()
    s = max(0.0, min(1.0, speed))

    # neutral defaults
    x, y, z, r = 0, 0, 500, 0

    if cmd == 'submerge':
        z = max(0, round(500 - s * 500))
        desc = f'DESCEND   — 4 vertical thrusters push DOWN  (heave z={z})'
    elif cmd == 'emerge':
        z = min(1000, round(500 + s * 500))
        desc = f'ASCEND    — 4 vertical thrusters push UP    (heave z={z})'
    elif cmd == 'surge_forward':
        x = round(s * 1000)
        desc = f'FORWARD   — horizontal thrusters drive FORWARD (surge x={x})'
    elif cmd == 'surge_backward':
        x = round(-s * 1000)
        desc = f'BACKWARD  — horizontal thrusters drive BACKWARD (surge x={x})'
    elif cmd == 'strafe_right':
        y = round(s * 1000)
        desc = f'STRAFE R  — horizontal thrusters slide RIGHT (strafe y={y})'
    elif cmd == 'strafe_left':
        y = round(-s * 1000)
        desc = f'STRAFE L  — horizontal thrusters slide LEFT  (strafe y={y})'
    elif cmd == 'rotate_cw':
        r = round(s * 1000)
        desc = f'YAW CW    — horizontal thrusters rotate CW   (yaw r={r})'
    elif cmd == 'rotate_ccw':
        r = round(-s * 1000)
        desc = f'YAW CCW   — horizontal thrusters rotate CCW  (yaw r={r})'
    elif cmd == 'depth_hold':
        desc = 'DEPTH HOLD — heave neutral, other axes unchanged'
    elif cmd == 'stop':
        desc = 'STOP      — all thrusters neutral'
    else:
        desc = f'UNKNOWN COMMAND "{cmd}" — real node would STOP'

    return x, y, z, r, desc


# All movement primitives, in a friendly menu order.
MOVE_PRIMITIVES = [
    'submerge', 'emerge',
    'surge_forward', 'surge_backward',
    'strafe_left', 'strafe_right',
    'rotate_cw', 'rotate_ccw',
    'stop', 'depth_hold',
]


class DryTestConsole(Node):
    """Print-only thruster stand-in + fake-sensor injector for dry testing."""

    def __init__(self, quiet_movement=False):
        super().__init__('dry_test_console')

        # When True, suppress the live movement_command printout (used so the
        # interactive menu isn't flooded). Off by default — the whole point is
        # to SEE what the thrusters would do.
        self._quiet_movement = quiet_movement
        self._monitor_nav = False
        self._monitor_vision = False
        self._move_count = 0

        # Vision-monitor throttling (camera detections arrive at frame rate).
        self._det_print_period = 0.5      # seconds between detection printouts
        self._last_det_print = 0.0
        self._had_dets = False
        # Heartbeat: prove the vision link is alive even when nothing is in
        # view (the detector publishes an empty array every frame).
        self._frames_seen = 0
        self._frames_with_dets = 0
        self._last_heartbeat = 0.0
        self._heartbeat_period = 3.0      # seconds between "alive" status lines
        self._last_depth = None           # only print depth when it changes

        # ── Publishers (inject fake inputs to the rest of the stack) ──
        self.move_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self.nav_pub = self.create_publisher(
            NavigationCommand, 'navigation_command', 10)
        self.det_pub = self.create_publisher(
            ObjectDetectionArray, 'vision/detections', 10)
        self.subdepth_pub = self.create_publisher(
            Float32, 'depth/sub_depth', 10)
        self.depthinfo_pub = self.create_publisher(
            DepthInfo, 'depth/info', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, 'localization/pose', 10)
        self.battery_pub = self.create_publisher(
            Float32, '/safety/battery_pct', 10)
        self.leak_pub = self.create_publisher(
            Bool, '/safety/leak_detected', 10)

        # ── Subscriptions (act as the print-only thruster + nav monitor) ──
        self.create_subscription(
            MovementCommand, 'movement_command', self._on_movement, 10)
        self.create_subscription(
            NavigationCommand, 'navigation_command', self._on_nav, 10)
        # Watch the REAL detector output (camera-only dry test).
        self.create_subscription(
            ObjectDetectionArray, 'vision/detections', self._on_detections, 10)
        self.create_subscription(
            Float32, 'depth/sub_depth', self._on_subdepth, 10)

        self.get_logger().info(
            'Dry-test console up — acting as PRINT-ONLY thrusters. '
            'Nothing will physically move.')

    # ─── Inbound: render commands as thruster output ────────────────────

    def _on_movement(self, msg: MovementCommand):
        if self._quiet_movement:
            return
        x, y, z, r, desc = decode_command(msg.command, msg.speed)
        self._move_count += 1
        dur = (f'{msg.duration:.1f}s then auto-stop'
               if msg.duration > 0 else 'until next command')
        # Printed (not logged) so it reads like a thruster console.
        print(f'\n[THRUSTERS #{self._move_count}] cmd="{msg.command}" '
              f'speed={msg.speed:.2f} ({dur})')
        print(f'   → {desc}')
        print(f'   → MAVLink manual_control: x={x:>5} y={y:>5} '
              f'z={z:>5} r={r:>5}')

    def _on_nav(self, msg: NavigationCommand):
        if not self._monitor_nav:
            return
        print(f'\n[NAV CMD] mode={msg.mode} label="{msg.target_label}" '
              f'speed={msg.speed:.2f} '
              f'target=({msg.target_x:.1f},{msg.target_y:.1f},'
              f'{msg.target_z:.1f}) yaw={msg.target_yaw:.2f}')

    def _on_detections(self, msg: ObjectDetectionArray):
        # Print REAL detections from the running detector (camera dry test).
        # The detector publishes EVERY frame (empty array when nothing is in
        # view), so we: (a) print detections when present, throttled; and
        # (b) emit a periodic heartbeat so you can see the vision link is alive
        # and how many frames have actually arrived — your "is the camera
        # pipeline running?" readout.
        if not self._monitor_vision:
            return
        self._frames_seen += 1
        dets = msg.detections
        now = time.monotonic()

        if dets:
            self._frames_with_dets += 1
            if now - self._last_det_print >= self._det_print_period:
                self._last_det_print = now
                self._last_heartbeat = now
                print(f'\n[VISION] ✓ {len(dets)} detection(s) '
                      f'(frame {self._frames_seen}):')
                for d in dets:
                    rng = f'{d.position.z:.2f}m' if d.position.z > 0 else 'unknown'
                    print(f'   • {d.label:<12} conf={d.confidence:.2f}  '
                          f'centre=({d.position.x:.2f},{d.position.y:.2f})  '
                          f'range={rng}  '
                          f'bbox={d.bbox_width:.2f}x{d.bbox_height:.2f}')
            self._had_dets = True
            return

        # No detections this frame. If we just lost a target, say so once.
        if self._had_dets:
            self._had_dets = False
            print('\n[VISION] … target lost (0 detections)')
            self._last_heartbeat = now
            return

        # Idle heartbeat — proves frames are flowing even with nothing in view.
        if now - self._last_heartbeat >= self._heartbeat_period:
            self._last_heartbeat = now
            print(f'[VISION] watching… {self._frames_seen} frames, '
                  f'0 detections in view (point camera at a '
                  f'pointed_nose / round_nose target)')

    def _on_subdepth(self, msg: Float32):
        # The detector also publishes depth/sub_depth (range-to-nearest-target).
        # It's ~0 when nothing is detected, so only print on a real change to
        # avoid spamming the console with 0.00 readings.
        if not self._monitor_vision:
            return
        d = round(float(msg.data), 2)
        if self._last_depth is None or abs(d - self._last_depth) >= 0.05:
            self._last_depth = d
            if d > 0.0:
                print(f'[DEPTH] range-to-target = {d:.2f} m')

    # ─── Outbound: inject fake data / commands ──────────────────────────

    def send_move(self, command, speed=0.5, duration=3.0):
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(max(0.0, min(1.0, speed)))
        msg.duration = float(max(0.0, duration))
        self.move_pub.publish(msg)

    def send_nav(self, mode, label='', speed=0.5, approach=0.0,
                 tx=0.0, ty=0.0, tz=0.0, tyaw=0.0):
        msg = NavigationCommand()
        msg.mode = mode
        msg.target_label = label
        msg.target_x = float(tx)
        msg.target_y = float(ty)
        msg.target_z = float(tz)
        msg.target_yaw = float(tyaw)
        msg.speed = float(speed)
        msg.approach_dist = float(approach)
        self.nav_pub.publish(msg)
        print(f'   ↑ sent navigation_command mode={mode} label="{label}"')

    def send_detection(self, label, conf=0.9, x=0.5, y=0.5, z=2.0,
                       bw=0.2, bh=0.3):
        det = ObjectDetection()
        det.label = label
        det.confidence = float(conf)
        det.position.x = float(x)
        det.position.y = float(y)
        det.position.z = float(z)
        det.bbox_width = float(bw)
        det.bbox_height = float(bh)
        arr = ObjectDetectionArray()
        arr.detections = [det]
        self.det_pub.publish(arr)
        print(f'   ↑ sent vision/detections: "{label}" conf={conf:.2f} '
              f'centre=({x:.2f},{y:.2f}) range={z:.1f}m')

    def send_depth(self, depth_m, stop_dist=1.5):
        f = Float32()
        f.data = float(depth_m)
        self.subdepth_pub.publish(f)
        di = DepthInfo()
        di.stamp = self.get_clock().now().to_msg()
        di.sub_depth_m = float(depth_m)
        di.stop_distance_m = float(stop_dist)
        self.depthinfo_pub.publish(di)
        print(f'   ↑ sent depth/sub_depth + depth/info: {depth_m:.2f} m')

    def send_pose(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # yaw → quaternion (roll=pitch=0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)
        print(f'   ↑ sent localization/pose: ({x:.2f},{y:.2f},{z:.2f}) '
              f'yaw={math.degrees(yaw):.0f}°')

    def send_battery(self, pct):
        f = Float32()
        f.data = float(pct)
        self.battery_pub.publish(f)
        print(f'   ↑ sent /safety/battery_pct: {pct:.0f}%')

    def send_leak(self, leaking):
        b = Bool()
        b.data = bool(leaking)
        self.leak_pub.publish(b)
        print(f'   ↑ sent /safety/leak_detected: {leaking}')


# ─── Non-interactive modes ──────────────────────────────────────────────────

def run_sweep(node: DryTestConsole):
    """Auto-cycle through every direction the sub can move, printing each."""
    print('\n=== THRUSTER / DIRECTION SWEEP (dry) ===')
    print('Publishing each movement primitive at speed 0.5; the print-only '
          'thruster stand-in renders what would happen.\n')
    for cmd in MOVE_PRIMITIVES:
        speed = 0.0 if cmd in ('stop', 'depth_hold') else 0.5
        node.send_move(cmd, speed, duration=0.0)
        time.sleep(0.6)          # let the subscription print
    node.send_move('stop', 0.0, 0.0)
    time.sleep(0.4)
    print('\n=== SWEEP COMPLETE — every direction exercised, nothing moved ===')


def _f(prompt, default):
    raw = input(f'  {prompt} [{default}]: ').strip()
    if raw == '':
        return default
    try:
        return float(raw)
    except ValueError:
        print('  (not a number — using default)')
        return default


def _s(prompt, default):
    raw = input(f'  {prompt} [{default}]: ').strip()
    return raw if raw else default


def print_menu():
    print("""
╔════════════════════════════════════════════════════════════════╗
║              ROBOSUB 2026 — DRY TEST CONSOLE                     ║
║          (print-only thrusters — nothing physically moves)       ║
╠════════════════════════════════════════════════════════════════╣
║  MOVEMENT PRIMITIVES                                             ║
║   1) submerge        2) emerge                                  ║
║   3) surge_forward   4) surge_backward                          ║
║   5) strafe_left     6) strafe_right                            ║
║   7) rotate_cw       8) rotate_ccw                              ║
║   9) stop            0) depth_hold                              ║
║   w) full direction sweep (all of the above)                    ║
║                                                                  ║
║  SUBSYSTEM INJECTION (test each part of the sub on its own)      ║
║   v) inject fake vision detection                               ║
║   d) publish fake depth                                         ║
║   p) publish fake pose (x,y,z,yaw)                              ║
║   b) publish fake battery %                                     ║
║   l) toggle leak flag                                           ║
║   n) send navigation_command (drives autonomous_controller)     ║
║   c) custom raw movement command                                ║
║                                                                  ║
║  MONITORS / MISC                                                 ║
║   V) toggle REAL vision-detection monitor (camera dry test)     ║
║   M) toggle movement_command printing (currently shown)         ║
║   N) toggle navigation_command monitor (watch the BT)           ║
║   h) help / show this menu        q) quit                       ║
╚════════════════════════════════════════════════════════════════╝""")


def run_menu(node: DryTestConsole):
    print_menu()
    primitive_keys = {
        '1': 'submerge', '2': 'emerge',
        '3': 'surge_forward', '4': 'surge_backward',
        '5': 'strafe_left', '6': 'strafe_right',
        '7': 'rotate_cw', '8': 'rotate_ccw',
        '9': 'stop', '0': 'depth_hold',
    }
    leak_state = False

    while rclpy.ok():
        try:
            choice = input('\ndry-test> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if choice == '':
            continue

        if choice in primitive_keys:
            cmd = primitive_keys[choice]
            if cmd in ('stop', 'depth_hold'):
                node.send_move(cmd, 0.0, 0.0)
            else:
                speed = _f('speed (0.0-1.0)', 0.5)
                dur = _f('duration s (0 = until next)', 3.0)
                node.send_move(cmd, speed, dur)

        elif choice == 'w':
            run_sweep(node)

        elif choice == 'v':
            label = _s('label', 'gate')
            conf = _f('confidence', 0.9)
            x = _f('centre-x (0-1, 0.5=mid)', 0.5)
            y = _f('centre-y (0-1, 0.5=mid)', 0.5)
            z = _f('range m (-1 unknown)', 2.0)
            node.send_detection(label, conf, x, y, z)

        elif choice == 'd':
            depth = _f('depth m (positive down)', 1.0)
            node.send_depth(depth)

        elif choice == 'p':
            x = _f('x m', 0.0)
            y = _f('y m', 0.0)
            z = _f('z m', 0.0)
            yaw = _f('yaw deg', 0.0)
            node.send_pose(x, y, z, math.radians(yaw))

        elif choice == 'b':
            pct = _f('battery %', 100.0)
            node.send_battery(pct)

        elif choice == 'l':
            leak_state = not leak_state
            node.send_leak(leak_state)

        elif choice == 'n':
            mode = _s('mode (idle/station_keep/track_object/search/'
                      'waypoint/heading_hold)', 'search')
            label = _s('target_label', 'gate')
            speed = _f('speed', 0.4)
            node.send_nav(mode, label, speed)

        elif choice == 'c':
            cmd = _s('raw command', 'surge_forward')
            speed = _f('speed', 0.5)
            dur = _f('duration', 3.0)
            node.send_move(cmd, speed, dur)

        elif choice == 'V':
            node._monitor_vision = not node._monitor_vision
            state = 'ON' if node._monitor_vision else 'OFF'
            print(f'  vision-detection monitor now {state} '
                  '(shows REAL detector output)')

        elif choice == 'M':
            node._quiet_movement = not node._quiet_movement
            state = 'HIDDEN' if node._quiet_movement else 'SHOWN'
            print(f'  movement_command printing now {state}')

        elif choice == 'N':
            node._monitor_nav = not node._monitor_nav
            state = 'ON' if node._monitor_nav else 'OFF'
            print(f'  navigation_command monitor now {state}')

        elif choice in ('h', '?'):
            print_menu()

        elif choice in ('q', 'Q'):
            break

        else:
            print('  unknown choice — press h for the menu')

    print('\nSending stop and exiting…')
    node.send_move('stop', 0.0, 0.0)
    time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(
        description='Dry-test console for the RoboSub 2026 stack '
                    '(print-only, no hardware).')
    parser.add_argument('--sweep', action='store_true',
                        help='Auto-cycle every movement direction then exit.')
    parser.add_argument('--monitor', action='store_true',
                        help='Passive: only print commands from the stack '
                             '(acts as the print-only thrusters).')
    parser.add_argument('--camera', action='store_true',
                        help='Camera-only dry test: print REAL vision/detections '
                             'AND the resulting (print-only) thruster commands. '
                             'Run the detector + a planner alongside; thrusters '
                             'need not be connected.')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DryTestConsole()

    # Passive modes (camera / monitor) just watch topics — spin in the MAIN
    # thread so Ctrl+C unwinds cleanly through rclpy.spin(). The interactive
    # menu and the sweep need stdin / to publish from the main thread, so for
    # those we spin ROS in a background thread instead.
    passive = args.camera or args.monitor
    spin_thread = None

    try:
        if passive:
            if args.camera:
                node._monitor_vision = True
                print('\n[CAMERA DRY TEST] Watching REAL vision/detections and '
                      'printing the thruster commands a planner produces '
                      'from them.')
                print('  Start these in other terminals:')
                print('    ros2 run vision detector ...        # camera + '
                      'detector')
                print('    ros2 run control autonomous_controller   # or a '
                      'mission node, to exercise detection-based conditions')
                print('  Thrusters need NOT be connected — movement is '
                      'print-only. Ctrl+C to quit.\n')
            else:
                print('\n[MONITOR] Acting as print-only thrusters. '
                      'Run the stack in other terminals; commands appear here. '
                      'Ctrl+C to quit.\n')
            rclpy.spin(node)          # blocks until Ctrl+C / shutdown
        else:
            # Background spin so run_sweep / run_menu own the main thread.
            def _spin():
                try:
                    rclpy.spin(node)
                except Exception:
                    pass

            spin_thread = threading.Thread(target=_spin, daemon=True)
            spin_thread.start()

            if args.sweep:
                time.sleep(0.5)       # let publishers/subscribers match up
                run_sweep(node)
            else:
                time.sleep(0.3)
                run_menu(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort final stop while the context is still up.
        try:
            if rclpy.ok():
                node.send_move('stop', 0.0, 0.0)
                time.sleep(0.1)
        except Exception:
            pass
        # Tear down in the right order to avoid a C++ abort: shut the context
        # down first (this makes any spinning thread return), join it, then
        # destroy the node.
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
