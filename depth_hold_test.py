#!/usr/bin/env python3
"""Closed-loop depth test — submerge 3 ft using ZED positional tracking, then hold.

⚠ SAFETY: this script ARMS the Pixhawk and drives the REAL vertical thrusters.
The sub WILL move. Keep clear of the props, run with props removed for a bench
check, and have an e-stop / kill switch ready.

What it does
------------
1. Runs the production ``mavlink_thruster_control.ThrusterController`` in-process
   (arm, MANUAL mode, 10 Hz loop, heartbeat, watchdog) so every
   ``movement_command`` it publishes drives the actual Pixhawk heave axis.
2. Subscribes to the ZED SDK positional-tracking pose. By default it spawns the
   ``localization.vslam_node.VSLAMZedNode`` in-process, which opens the ZED2i,
   enables positional tracking and publishes ``vslam/odometry``
   (nav_msgs/Odometry, WORLD frame, RIGHT_HANDED_Y_UP → vertical = Y axis).
   Pass ``--external-vslam`` to instead subscribe to a ``vslam/odometry`` topic
   already being published by a separately-launched node.
3. On the first pose it latches the start vertical as depth-zero, then runs a
   simple P controller on the heave axis:
       depth_below_start = sign * (current_vertical - start_vertical)
       error             = target_depth - depth_below_start
       |error| <= deadband  → depth_hold  (neutral heave, ZED holds station)
       error > 0 (too shallow) → submerge at clamp(kp*error, min, max)
       error < 0 (too deep)    → emerge   at clamp(kp*|error|, min, max)
   It publishes the command at 10 Hz so the sub first descends to 3 ft then
   actively holds that depth against drift, reading depth ONLY from the ZED
   positional tracking (no pressure sensor involved).

A hard safety abort emerges + stops if the measured depth ever exceeds
``--max-depth`` (default 2× target), guarding a runaway descend.

Usage
-----
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  # ZED spawned in-process (one command does everything):
  python3 depth_hold_test.py                 # submerge 3 ft, then hold
  python3 depth_hold_test.py --depth 1.5     # different target (feet)

  # Use a vslam node you launch yourself in another terminal:
  #   ros2 run localization vslam_node
  python3 depth_hold_test.py --external-vslam

Ctrl+C at any time → stop + disarm (ThrusterController cleanup).
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from auv_msgs.msg import MovementCommand

# Production thruster driver — reused so this script drives the real Pixhawk
# (arming, MANUAL mode, 10 Hz control loop, heartbeat, watchdog) exactly like
# the real mission does. Requires the workspace to be sourced.
from mavlink_thruster_control.thruster_node import ThrusterController

_FEET_TO_M = 0.3048


def _axis_value(position, axis: str) -> float:
    """Pull the requested axis (x/y/z) out of a geometry_msgs Point."""
    return float(getattr(position, axis))


class DepthHoldController(Node):
    """P controller that drives heave from the ZED positional-tracking pose."""

    # 10 Hz command cadence — matches the thruster control loop so every loop
    # has a fresh axis value to send to the Pixhawk.
    _CONTROL_PERIOD_S = 0.1

    def __init__(self, target_depth_m, axis, sign, kp,
                 min_speed, max_speed, deadband_m, settle_tol_m,
                 max_depth_m):
        super().__init__('depth_hold_test')

        self._target = float(target_depth_m)
        self._axis = axis
        self._sign = float(sign)
        self._kp = float(kp)
        self._min_speed = float(min_speed)
        self._max_speed = float(max_speed)
        self._deadband = float(deadband_m)
        self._settle_tol = float(settle_tol_m)
        self._max_depth = float(max_depth_m)

        self._start_vert = None      # latched on first pose
        self._depth = 0.0            # current depth below start (m)
        self._have_pose = False
        self._last_pose_time = None
        self._aborted = False
        self._reached = False        # first time we hit target tolerance
        self._loops = 0

        self.move_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self.create_subscription(
            Odometry, 'vslam/odometry', self._on_odom, 10)

        self.create_timer(self._CONTROL_PERIOD_S, self._control_loop)

        self.get_logger().info(
            f'Depth-hold test up — target {self._target:.3f} m '
            f'({self._target / _FEET_TO_M:.2f} ft) below start, '
            f'reading ZED axis "{self._axis}" (sign {self._sign:+.0f}). '
            f'Waiting for vslam/odometry…')

    # ─── ZED positional-tracking pose ───────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        vert = _axis_value(msg.pose.pose.position, self._axis)
        if not math.isfinite(vert):
            self.get_logger().warn('Non-finite vslam vertical — ignoring frame')
            return
        if self._start_vert is None:
            self._start_vert = vert
            self.get_logger().info(
                f'Latched start vertical = {vert:.3f} m. Descending…')
        self._depth = self._sign * (vert - self._start_vert)
        self._have_pose = True
        self._last_pose_time = time.monotonic()

    # ─── Outbound command helper ────────────────────────────────────────────

    def _send(self, command, speed):
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(max(0.0, min(1.0, speed)))
        msg.duration = 0.0           # held until the next command (we send 10 Hz)
        self.move_pub.publish(msg)

    # ─── 10 Hz P control loop ───────────────────────────────────────────────

    def _control_loop(self):
        self._loops += 1

        # No pose yet → keep heave neutral; never descend blind.
        if not self._have_pose:
            self._send('depth_hold', 0.0)
            return

        # Stale pose (ZED tracking dropped) → neutral + warn. The sub stays put
        # rather than running open-loop on a frozen depth estimate.
        if (self._last_pose_time is not None
                and time.monotonic() - self._last_pose_time > 1.0):
            self._send('depth_hold', 0.0)
            if self._loops % 10 == 0:
                self.get_logger().warn(
                    'No vslam pose for >1 s — holding neutral (ZED tracking lost?)')
            return

        # Hard safety abort — runaway descend.
        if not self._aborted and self._depth > self._max_depth:
            self._aborted = True
            self.get_logger().error(
                f'ABORT: depth {self._depth:.2f} m exceeds max '
                f'{self._max_depth:.2f} m — emerging + stopping')
        if self._aborted:
            # Drive up gently, then neutral once back near the surface.
            if self._depth > self._deadband:
                self._send('emerge', self._min_speed)
            else:
                self._send('stop', 0.0)
            return

        error = self._target - self._depth      # +ve: need to go deeper

        if abs(error) <= self._deadband:
            self._send('depth_hold', 0.0)
            if not self._reached and abs(error) <= self._settle_tol:
                self._reached = True
                self.get_logger().info(
                    f'✓ Reached target depth {self._depth:.3f} m — holding.')
        elif error > 0:
            self._send('submerge', self._clamp_speed(error))
        else:
            self._send('emerge', self._clamp_speed(-error))

        if self._loops % 10 == 0:     # ~1 Hz telemetry
            state = ('HOLD' if abs(error) <= self._deadband
                     else 'DESCEND' if error > 0 else 'CORRECT-UP')
            self.get_logger().info(
                f'[{state}] depth={self._depth:.3f} m '
                f'target={self._target:.3f} m err={error:+.3f} m')

    def _clamp_speed(self, magnitude):
        return max(self._min_speed, min(self._max_speed, self._kp * magnitude))


def main():
    p = argparse.ArgumentParser(
        description='Submerge a fixed depth using ZED positional tracking, '
                    'then hold (drives REAL Pixhawk thrusters).')
    p.add_argument('--depth', type=float, default=3.0,
                   help='Target depth in FEET below start (default: 3.0).')
    p.add_argument('--axis', default='y', choices=['x', 'y', 'z'],
                   help='ZED odometry axis that is vertical. ZED Y_UP world '
                        'frame → "y" (default).')
    p.add_argument('--sign', type=float, default=-1.0,
                   help='Multiplier so depth-below-start is POSITIVE while '
                        'descending. Y_UP descend lowers Y → -1.0 (default).')
    p.add_argument('--kp', type=float, default=1.2,
                   help='Proportional gain (speed per metre of error).')
    p.add_argument('--min-speed', type=float, default=0.12,
                   help='Minimum heave speed when moving (0-1).')
    p.add_argument('--max-speed', type=float, default=1.0,
                   help='Maximum heave speed (0-1). Full power — thrusters too '
                        'weak to push the sub under at lower values.')
    p.add_argument('--deadband', type=float, default=0.05,
                   help='Half-width (m) of the neutral hold band.')
    p.add_argument('--settle-tol', type=float, default=0.08,
                   help='Error (m) under which we declare target reached.')
    p.add_argument('--max-depth', type=float, default=0.0,
                   help='Abort+emerge above this depth (m). 0 → 2× target.')
    p.add_argument('--external-vslam', action='store_true',
                   help='Do NOT spawn the ZED vslam node in-process; subscribe '
                        'to a vslam/odometry topic published elsewhere.')
    args, _ = p.parse_known_args()

    target_m = args.depth * _FEET_TO_M
    max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * target_m

    rclpy.init()

    controller = DepthHoldController(
        target_depth_m=target_m, axis=args.axis, sign=args.sign, kp=args.kp,
        min_speed=args.min_speed, max_speed=args.max_speed,
        deadband_m=args.deadband, settle_tol_m=args.settle_tol,
        max_depth_m=max_depth_m)
    # MANUAL: the ZED P-controller below is the SOLE depth authority. ALT_HOLD
    # would make the autopilot hold depth off the baro and fight this loop.
    thrusters = ThrusterController(flight_mode='MANUAL')   # real Pixhawk driver

    nodes = [controller, thrusters]
    vslam = None
    if not args.external_vslam:
        # Spawn the ZED positional-tracking node in-process so a single command
        # opens the camera, tracks, and feeds vslam/odometry to the controller.
        from localization.vslam_node import VSLAMZedNode
        vslam = VSLAMZedNode()
        nodes.append(vslam)
    else:
        print('\n[external-vslam] Subscribing to vslam/odometry. Start it with:'
              '\n    ros2 run localization vslam_node\n')

    print('\n⚠ ARMING PIXHAWK — vertical thrusters WILL drive. Keep clear. '
          'Ctrl+C to stop + disarm.\n')

    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort neutral heave before teardown.
        try:
            if rclpy.ok():
                controller._send('stop', 0.0)
                time.sleep(0.1)
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        # Destroy thruster node first so its stop+disarm cleanup runs while the
        # MAVLink link is still up.
        for n in (thrusters, controller, vslam):
            if n is None:
                continue
            try:
                n.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
