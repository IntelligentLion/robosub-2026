#!/usr/bin/env python3
"""Submerge-only test — reuses the prequalification node's closed-loop depth
logic to descend to a target depth, then hold it.

⚠ SAFETY: this script ARMS the Pixhawk and drives the REAL vertical thrusters.
The sub WILL move. Keep clear of the props, run with props removed for a bench
check, and have an e-stop / kill switch ready.

What it does
------------
1. Runs the production ``mavlink_thruster_control.ThrusterController`` in-process
   (arm, MANUAL mode, 10 Hz loop, heartbeat, watchdog) so every
   ``movement_command`` published drives the actual Pixhawk heave axis.
2. Subscribes to ``depth/sub_depth`` (std_msgs/Float32, metres below surface,
   positive down) — the SAME depth stream the prequalification mission uses
   (published by the vision detector from ZED positional tracking). You must
   have a depth publisher running, e.g.:
       ros2 run vision detector          # ZED-derived sub_depth
3. Phase 1 (DESCEND): commands ``submerge`` at ``--submerge-speed`` until the
   measured depth reaches ``--target-depth``, capturing the live depth as the
   depth to hold (exactly like the prequal node's submerge state).
4. Phase 2 (HOLD): runs the prequal node's ``_apply_depth_hold`` closed-loop:
       err = hold_depth - depth        (+ve → go deeper)
       |err| <= tol → depth_hold (neutral heave)
       err > 0      → submerge at clamp(gain*|err|, min, max)
       err < 0      → emerge   at clamp(gain*|err|, min, max)
   so the sub actively holds the captured depth against drift.

A hard safety abort emerges + stops if the measured depth ever exceeds
``--max-depth`` (default 2× target), guarding a runaway descend.

Usage
-----
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  # depth publisher in another terminal:
  ros2 run vision detector

  python3 submerge_test.py                       # descend 1.0 m, then hold
  python3 submerge_test.py --target-depth 1.5    # different target (metres)

Tune thruster power from the command line:
  --submerge-speed       heave power used during the descent (0-1)
  --depth-hold-min-speed minimum heave power while correcting depth (0-1)
  --depth-hold-max-speed maximum heave power while correcting depth (0-1)
  --depth-hold-gain      heave power per metre of depth error

Ctrl+C at any time → stop + disarm (ThrusterController cleanup).
"""

import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32
from auv_msgs.msg import MovementCommand

# Production thruster driver — reused so this script drives the real Pixhawk
# (arming, MANUAL mode, 10 Hz control loop, heartbeat, watchdog) exactly like
# the real mission does. Requires the workspace to be sourced.
from mavlink_thruster_control.thruster_node import ThrusterController


class SubmergeTest(Node):
    """Descend to a target depth then hold it, mirroring the prequal node's
    submerge + ``_apply_depth_hold`` closed loop."""

    # 10 Hz command cadence — matches the thruster control loop.
    _CONTROL_PERIOD_S = 0.1

    def __init__(self, target_depth_m, submerge_speed,
                 hold_gain, hold_min_speed, hold_max_speed, hold_tol_m,
                 max_depth_m, ramp_time_s=0.0):
        super().__init__('submerge_test')

        self._target = float(target_depth_m)
        self._submerge_speed = float(submerge_speed)
        self._ramp_time_s = float(ramp_time_s)
        self._descend_start_s = None   # set when descent begins (ramp clock)
        self._hold_gain = float(hold_gain)
        self._hold_min_speed = float(hold_min_speed)
        self._hold_max_speed = float(hold_max_speed)
        self._hold_tol = float(hold_tol_m)
        self._max_depth = float(max_depth_m)

        self._depth_m = 0.0           # latest depth/sub_depth (metres)
        self._have_depth = False      # True once first reading arrives
        self._hold_depth_m = None     # captured once descent ends
        self._descending = True       # phase 1 until target reached
        self._aborted = False
        self._loops = 0

        self._cmd_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self.create_subscription(
            Float32, 'depth/sub_depth', self._depth_cb, 10)

        self.create_timer(self._CONTROL_PERIOD_S, self._control_loop)

        self.get_logger().info(
            f'Submerge test up — target {self._target:.3f} m below surface, '
            f'submerge_speed={self._submerge_speed:.2f}, '
            f'hold[gain={self._hold_gain:.2f} '
            f'min={self._hold_min_speed:.2f} max={self._hold_max_speed:.2f} '
            f'tol={self._hold_tol:.2f}m]. Waiting for depth/sub_depth…')

    # ─── Depth feedback ─────────────────────────────────────────────────────

    def _depth_cb(self, msg: Float32):
        if msg.data is not None:
            self._depth_m = float(msg.data)
            self._have_depth = True

    # ─── Outbound command helper (matches prequal node) ─────────────────────

    def _send(self, command, speed=0.0):
        msg = MovementCommand()
        msg.command = command
        msg.speed = float(max(0.0, min(1.0, speed)))
        msg.duration = 0.0           # held until the next command (10 Hz)
        self._cmd_pub.publish(msg)

    # ─── 10 Hz control loop ─────────────────────────────────────────────────

    def _control_loop(self):
        self._loops += 1

        # No depth yet → keep heave neutral; never descend blind.
        if not self._have_depth:
            self._send('depth_hold')
            if self._loops % 20 == 0:
                self.get_logger().warn(
                    'No depth/sub_depth yet — holding neutral (is the depth '
                    'publisher running?)')
            return

        # Hard safety abort — runaway descend.
        if not self._aborted and self._depth_m > self._max_depth:
            self._aborted = True
            self.get_logger().error(
                f'ABORT: depth {self._depth_m:.2f} m exceeds max '
                f'{self._max_depth:.2f} m — emerging + stopping')
        if self._aborted:
            if self._depth_m > self._hold_tol:
                self._send('emerge', self._hold_min_speed)
            else:
                self._send('stop')
            return

        # Phase 1: descend until the target depth is reached, capturing the
        # live depth as the depth to hold (prequal submerge behaviour).
        if self._descending:
            self._hold_depth_m = self._depth_m
            if self._depth_m >= self._target:
                self._descending = False
                self._hold_depth_m = self._depth_m
                self.get_logger().info(
                    f'✓ Reached target depth {self._depth_m:.3f} m — holding.')
            else:
                # Ramp heave power 0 → submerge_speed over ramp_time_s so the
                # thrusters spool up gradually instead of slamming to full power.
                if self._descend_start_s is None:
                    self._descend_start_s = time.monotonic()
                if self._ramp_time_s > 0.0:
                    frac = (time.monotonic() - self._descend_start_s) \
                        / self._ramp_time_s
                    frac = max(0.0, min(1.0, frac))
                else:
                    frac = 1.0
                speed = self._submerge_speed * frac
                self._send('submerge', speed)
                if self._loops % 10 == 0:
                    self.get_logger().info(
                        f'[DESCEND] depth={self._depth_m:.3f} m '
                        f'target={self._target:.3f} m '
                        f'speed={speed:.2f} ({frac*100:.0f}%)')
                return

        # Phase 2: closed-loop hold (prequal node's _apply_depth_hold).
        self._apply_depth_hold()

    def _apply_depth_hold(self):
        if self._hold_depth_m is None or self._depth_m < 0.0:
            return
        err = self._hold_depth_m - self._depth_m   # +ve → need to go deeper
        if abs(err) <= self._hold_tol:
            self._send('depth_hold')
        else:
            speed = min(self._hold_max_speed,
                        max(self._hold_min_speed, abs(err) * self._hold_gain))
            self._send('submerge' if err > 0 else 'emerge', speed)

        if self._loops % 10 == 0:      # ~1 Hz telemetry
            state = 'HOLD' if abs(err) <= self._hold_tol else \
                ('DEEPER' if err > 0 else 'SHALLOWER')
            self.get_logger().info(
                f'[{state}] depth={self._depth_m:.3f} m '
                f'hold={self._hold_depth_m:.3f} m err={err:+.3f} m')


def main():
    p = argparse.ArgumentParser(
        description='Submerge to a target depth using the prequal depth-hold '
                    'logic, then hold (drives REAL Pixhawk thrusters).')
    p.add_argument('--target-depth', type=float, default=1.0,
                   help='Target depth in METRES below surface (default: 1.0).')
    p.add_argument('--max-depth', type=float, default=0.0,
                   help='Abort+emerge above this depth (m). 0 → 2× target.')
    # ── Thruster power knobs ──────────────────────────────────────────
    p.add_argument('--submerge-speed', type=float, default=0.40,
                   help='Heave power during the descent (0-1, default 0.40).')
    p.add_argument('--ramp-time', type=float, default=3.0,
                   help='Seconds to ramp descent power 0→submerge-speed '
                        '(default 3.0; 0 → no ramp, full power immediately).')
    p.add_argument('--depth-hold-gain', type=float, default=1.0,
                   help='Heave power per metre of depth error (default 1.0).')
    p.add_argument('--depth-hold-min-speed', type=float, default=0.10,
                   help='Min heave power while correcting depth (default 0.10).')
    p.add_argument('--depth-hold-max-speed', type=float, default=0.40,
                   help='Max heave power while correcting depth (default 0.40).')
    p.add_argument('--depth-hold-tol', type=float, default=0.10,
                   help='Half-width (m) of the neutral hold band (default 0.10).')
    args, _ = p.parse_known_args()

    max_depth_m = args.max_depth if args.max_depth > 0 else 2.0 * args.target_depth

    rclpy.init()

    controller = SubmergeTest(
        target_depth_m=args.target_depth,
        submerge_speed=args.submerge_speed,
        hold_gain=args.depth_hold_gain,
        hold_min_speed=args.depth_hold_min_speed,
        hold_max_speed=args.depth_hold_max_speed,
        hold_tol_m=args.depth_hold_tol,
        max_depth_m=max_depth_m,
        ramp_time_s=args.ramp_time)
    thrusters = ThrusterController()        # real Pixhawk driver

    print('\n⚠ ARMING PIXHAWK — vertical thrusters WILL drive. Keep clear. '
          'Ctrl+C to stop + disarm.\n')

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(thrusters)

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
                controller._send('stop')
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
        for n in (thrusters, controller):
            try:
                n.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
