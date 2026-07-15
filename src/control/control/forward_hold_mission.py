#!/usr/bin/env python3
"""forward_hold_mission — the one-command run: dive, hold, drive forward.

This is the node the launch file adds on top of `submerge_hold.launch.py`. It
owns NO control law. It gates on the stack being genuinely alive, then drives
`control.api.Auv`, which publishes intent to motion_node like any other client.

The gate exists because launch can only sequence PROCESSES, and a live process
is not a live vehicle. `thruster_node` starts, and comes up perfectly happily
with a Pixhawk that never answered, or a Bar02 that dropped off I2C — both are
routine on this sub (see the Bar02 30BA notes). So before a dive is commanded:

  1. pixhawk/mode + pixhawk/armed publishers exist   → thruster_node is up
  2. a MAVLink-sourced mode string has arrived        → the Pixhawk is TALKING
  3. pixhawk/{preflight,set_mode} services are ready  → the gateway can act
  4. N consecutive finite pixhawk/depth samples       → the Bar02 is alive
     (thruster_node publishes NaN when it has no depth, so a topic that merely
     ticks proves nothing — the VALUES have to be real)
  5. motion_node is subscribed to motion/submerge     → intent will land

Only then does it submerge. Any gate that times out fails the mission with a
nonzero exit, and the launch file turns that into a clean shutdown of the whole
stack rather than a half-running one.

Depth hold itself is ArduSub's (ALT_HOLD) — SubmergeController inside
motion_node sequences into it. Nothing here re-implements it.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from auv_msgs.srv import SetFlightMode

from control.api import Auv, SubmergeError

# Consecutive finite depth samples required before the Bar02 counts as alive.
# thruster_node publishes at 10 Hz, so this is ~0.5 s of real data. One sample
# is not enough: the intermittent I2C failure drops out mid-run, and a single
# lucky reading is exactly what it looks like on the way down.
DEPTH_SAMPLES_REQUIRED = 5


class MissionAborted(RuntimeError):
    """A readiness gate failed. The stack is not safe to dive."""


class ForwardHoldMission(Node):
    def __init__(self):
        super().__init__('forward_hold_mission')
        self.declare_parameter('target_depth', 2.0)
        self.declare_parameter('forward_speed', 0.4)
        self.declare_parameter('forward_duration', 10.0)
        self.declare_parameter('startup_timeout', 30.0)
        self.declare_parameter('dive_timeout', 60.0)
        self.declare_parameter('surface_on_finish', True)

        self.target_depth = float(self.get_parameter('target_depth').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_duration = float(
            self.get_parameter('forward_duration').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout').value)
        self.dive_timeout = float(self.get_parameter('dive_timeout').value)
        self.surface_on_finish = bool(
            self.get_parameter('surface_on_finish').value)

        if not (0.0 < self.target_depth <= 30.0):
            raise ValueError(
                f'target_depth must be a positive depth below the surface, '
                f'0 < d <= 30 (got {self.target_depth})')
        if not (0.0 <= self.forward_speed <= 1.0):
            raise ValueError(
                f'forward_speed is normalized 0.0-1.0, matching '
                f'MovementCommand (got {self.forward_speed})')

        self._depth_streak = 0
        self._mode = None
        self._armed = False

        self.create_subscription(Float32, 'pixhawk/depth', self._on_depth, 10)
        self.create_subscription(String, 'pixhawk/mode', self._on_mode, 10)
        self.create_subscription(Bool, 'pixhawk/armed', self._on_armed, 10)

        self._preflight_cli = self.create_client(Trigger, 'pixhawk/preflight')
        self._mode_cli = self.create_client(SetFlightMode, 'pixhawk/set_mode')

    # ─── telemetry ──────────────────────────────────────────────────

    def _on_depth(self, msg: Float32):
        # NaN is thruster_node's "I have no depth", not a value. A streak, not a
        # counter: one good sample between dropouts must not add up to alive.
        if math.isfinite(msg.data):
            self._depth_streak += 1
        else:
            self._depth_streak = 0

    def _on_mode(self, msg: String):
        self._mode = msg.data

    def _on_armed(self, msg: Bool):
        self._armed = bool(msg.data)

    # ─── gates ──────────────────────────────────────────────────────

    def _await(self, what, predicate, timeout, hint=''):
        """Spin until predicate() or timeout. Raises MissionAborted, naming what
        was being waited on — the operator should never have to guess."""
        self.get_logger().info(f'waiting for {what} ...')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info(f'OK  {what}')
                return
        raise MissionAborted(
            f'timed out after {timeout:.0f}s waiting for {what}'
            + (f' — {hint}' if hint else ''))

    def wait_until_ready(self):
        t = self.startup_timeout

        self._await(
            'thruster_node telemetry (pixhawk/mode, pixhawk/armed)',
            lambda: (self.count_publishers('pixhawk/mode') > 0
                     and self.count_publishers('pixhawk/armed') > 0),
            t, hint='thruster_node did not start')

        self._await(
            'Pixhawk MAVLink link (a mode string from the autopilot)',
            lambda: self._mode not in (None, '', 'UNKNOWN'),
            t,
            hint='thruster_node is up but the Pixhawk is not answering. Check '
                 'the serial port and that nothing else has /dev/ttyACM0 open '
                 '(pix_imu/pixhawk_imu_bridge must NOT be running).')

        self._await(
            'pixhawk/preflight + pixhawk/set_mode services',
            lambda: (self._preflight_cli.service_is_ready()
                     and self._mode_cli.service_is_ready()),
            t, hint='the MAVLink gateway never advertised its services')

        self._await(
            f'depth data ({DEPTH_SAMPLES_REQUIRED} consecutive finite '
            'pixhawk/depth samples)',
            lambda: self._depth_streak >= DEPTH_SAMPLES_REQUIRED,
            t,
            hint='the Bar02 is not reporting. ArduSub REFUSES ALT_HOLD with no '
                 'depth sensor, so depth hold would not exist — refusing to '
                 'dive. Reseat the Bar02 I2C cable.')

        # motion_node INHIBITS itself if it is not the only publisher of
        # movement_command (two publishers = two things fighting over the
        # thrusters with no arbiter), and then refuses the submerge. Caught
        # here, that is a named abort in a second; left to motion_node alone it
        # is a silent dive that never starts and a dive_timeout a minute later.
        # A stale field_common script or autonomous_controller from an earlier
        # run is the usual culprit.
        movers = self.count_publishers('movement_command')
        if movers > 1:
            raise MissionAborted(
                f'{movers} publishers on movement_command — motion_node must be '
                'the only one, and inhibits itself when it is not. Stop the '
                'other publisher (autonomous_controller, a field_common script, '
                'or a motion_node left over from a previous run) and retry.')

        self.get_logger().info(
            f'stack ready — Pixhawk in {self._mode}, armed={self._armed}, '
            f'depth live, movement_command uncontested')

    # ─── the run ────────────────────────────────────────────────────

    def run(self):
        self.wait_until_ready()

        # Auv shares THIS node: one executor, one set of callbacks. It waits for
        # motion_node's motion/submerge subscription itself (a Float32 published
        # into the void is silently lost), which is gate 5.
        auv = Auv(node=self)
        try:
            # dive_speed is motion_node's parameter, not an argument here: it is
            # the node that runs DepthController. The launch file sets it there.
            self.get_logger().info(
                f'DIVE — target {self.target_depth:.2f} m (preflight -> '
                'ALT_HOLD -> arm -> descend). Depth hold is ArduSub\'s '
                'ALT_HOLD; heading is ours.')
            auv.submerge_to_depth(
                target_depth=self.target_depth, timeout=self.dive_timeout)
            self.get_logger().info(
                f'HOLD — at {self.target_depth:.2f} m, depth+heading+attitude '
                'held. Heading captured.')

            if self.forward_speed > 0.0 and self.forward_duration > 0.0:
                self.get_logger().info(
                    f'FORWARD — surge {self.forward_speed:.2f} for '
                    f'{self.forward_duration:.1f}s')
                auv.move_forward(
                    speed=self.forward_speed, duration=self.forward_duration)
                self.get_logger().info('FORWARD complete — stopped')
            else:
                self.get_logger().warn(
                    f'no forward leg (forward_speed={self.forward_speed}, '
                    f'forward_duration={self.forward_duration}) — holding only')

            if self.surface_on_finish:
                # Releases the dive: motion_node stops and unlocks. The vehicle
                # stays in ALT_HOLD at its current depth — this does NOT ascend.
                self.get_logger().info(
                    'RELEASE — dive released; vehicle holds depth in ALT_HOLD. '
                    'Disarm to surface.')
                auv.surface()
        finally:
            auv.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ForwardHoldMission()
        node.run()
    except (MissionAborted, SubmergeError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(f'MISSION ABORTED — {exc}')
        else:
            print(f'MISSION ABORTED — {exc}', file=sys.stderr)
        _shutdown(node)
        # Nonzero: the launch file turns this into a clean shutdown of the whole
        # stack. A mission that failed must not leave the thrusters live.
        sys.exit(1)
    except KeyboardInterrupt:
        pass
    _shutdown(node)


def _shutdown(node):
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
