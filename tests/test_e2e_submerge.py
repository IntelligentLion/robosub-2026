"""End-to-end: the Auv API drives a live motion_node against a simulated vehicle.

No hardware and no thruster_node. A FakeVehicle node stands in for the gateway:
it serves pixhawk/preflight and pixhawk/set_mode, publishes pixhawk/{depth,mode,
armed} and imu/rpy, and — crucially — *responds* to movement_command, so the
depth it reports is the depth the commanded heave would actually produce.

It also drifts to the right whenever the sub is driving forward. That is the
2026-07-13 veer-right symptom, and it is what the heading lock exists to cancel;
the last test here asserts it is cancelled.
"""
import math
import threading

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from auv_msgs.msg import MovementCommand
from auv_msgs.srv import SetFlightMode

from control.api import Auv, SubmergeError
from control.motion_node import MotionNode

TICK = 0.02                # 50 Hz simulation
DESCENT_RATE = 1.0         # m/s at heave = 1.0
VEER_RATE = 0.35           # rad/s of uncommanded right yaw at full forward
YAW_AUTHORITY = 2.0        # rad/s at yaw_rate = 1.0


class FakeVehicle(Node):
    """A sub that actually responds to what it is told."""

    def __init__(self, preflight_ok=True, mode_ok=True, veer=True):
        super().__init__('fake_vehicle')
        self.preflight_ok = preflight_ok
        self.mode_ok = mode_ok
        self.veer = veer

        self.depth = 0.0
        self.yaw = 0.0
        self.mode = 'MANUAL'
        self.armed = True
        self.cmd = MovementCommand()
        self.cmd.command = 'stop'
        self.yaw_history = []

        self.create_subscription(
            MovementCommand, 'movement_command', self._on_cmd, 10)
        self._depth_pub = self.create_publisher(Float32, 'pixhawk/depth', 10)
        self._mode_pub = self.create_publisher(String, 'pixhawk/mode', 10)
        self._armed_pub = self.create_publisher(Bool, 'pixhawk/armed', 10)
        self._rpy_pub = self.create_publisher(Vector3Stamped, 'imu/rpy', 10)

        self.create_service(Trigger, 'pixhawk/preflight', self._on_preflight)
        self.create_service(SetFlightMode, 'pixhawk/set_mode', self._on_set_mode)
        self.create_timer(TICK, self._tick)

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_preflight(self, req, resp):
        resp.success = self.preflight_ok
        resp.message = ('ok' if self.preflight_ok
                        else 'MOT_3_DIRECTION = -1 but backup says +1')
        return resp

    def _on_set_mode(self, req, resp):
        if self.mode_ok:
            self.mode = req.mode           # the vehicle actually enters it
            resp.success = True
            resp.reason = ''
        else:
            resp.success = False           # refused: stays where it was
            resp.reason = 'Depth sensor is not connected.'
        return resp

    def _tick(self):
        surge = heave = yaw_rate = 0.0
        if self.cmd.command == 'axes':
            surge, heave, yaw_rate = (
                self.cmd.surge, self.cmd.heave, self.cmd.yaw_rate)

        self.depth = max(0.0, self.depth + heave * DESCENT_RATE * TICK)

        # +yaw_rate is CW; imu/rpy yaw is CCW-positive (REP-103), so a CW
        # command DECREASES yaw. Get this backwards and the "lock" is positive
        # feedback that spins the sub — which is exactly why it is asserted.
        self.yaw -= yaw_rate * YAW_AUTHORITY * TICK
        if self.veer:
            self.yaw -= abs(surge) * VEER_RATE * TICK     # drifts right
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.yaw_history.append(self.yaw)

        self._depth_pub.publish(Float32(data=self.depth))
        self._mode_pub.publish(String(data=self.mode))
        self._armed_pub.publish(Bool(data=self.armed))
        rpy = Vector3Stamped()
        rpy.vector.z = self.yaw
        self._rpy_pub.publish(rpy)


class Rig:
    def __init__(self, **vehicle_kw):
        self.vehicle = FakeVehicle(**vehicle_kw)
        self.motion = MotionNode()
        self.api_node = Node('auv_api_test')
        self.auv = Auv(node=self.api_node)

        self.executor = MultiThreadedExecutor()
        for n in (self.vehicle, self.motion, self.api_node):
            self.executor.add_node(n)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def close(self):
        self.executor.shutdown()
        for n in (self.vehicle, self.motion, self.api_node):
            n.destroy_node()


@pytest.fixture
def rig(request):
    rclpy.init()
    made = {}

    def _make(**kw):
        made['rig'] = Rig(**kw)
        return made['rig']

    yield _make
    if 'rig' in made:
        made['rig'].close()
    rclpy.shutdown()


def test_submerge_reaches_depth_and_enters_alt_hold(rig):
    r = rig()
    r.auv.submerge_to_depth(target_depth=2.0, timeout=30.0)

    assert r.auv.state == 'hold'
    assert r.vehicle.mode == 'ALT_HOLD'
    assert r.vehicle.depth == pytest.approx(2.0, abs=0.2)


def test_heave_is_released_once_at_depth_so_alt_hold_owns_it(rig):
    r = rig()
    r.auv.submerge_to_depth(target_depth=2.0, timeout=30.0)
    r.auv._spin(0.5)
    assert r.vehicle.cmd.heave == 0.0


def test_forward_motion_holds_the_captured_heading(rig):
    # The whole point. The fake vehicle veers right whenever it drives forward;
    # with the lock closed, the heading must stay put anyway.
    r = rig()
    r.auv.submerge_to_depth(target_depth=2.0, timeout=30.0)
    captured = r.vehicle.yaw

    r.auv.move_forward(speed=0.4, duration=6.0)

    drift = abs(math.atan2(math.sin(r.vehicle.yaw - captured),
                           math.cos(r.vehicle.yaw - captured)))
    # Open loop the vehicle would have swung VEER_RATE * 6 s ≈ 2.1 rad.
    assert drift < 0.15, f'heading drifted {math.degrees(drift):.1f}° — lock failed'


def test_veer_is_real_without_the_lock(rig):
    # Guard against the previous test passing because the simulated veer is not
    # actually there. Drive movement_command directly, bypassing the lock.
    r = rig()
    cmd = MovementCommand()
    cmd.command = 'axes'
    cmd.surge = 0.4
    start = r.vehicle.yaw
    pub = r.api_node.create_publisher(MovementCommand, 'movement_command', 10)
    for _ in range(60):
        pub.publish(cmd)
        r.auv._spin(0.1)
    drift = abs(r.vehicle.yaw - start)
    assert drift > 0.5, 'the simulated veer is not present — the lock test is vacuous'


def test_refused_alt_hold_aborts_the_dive_on_the_surface(rig):
    # A dead Bar02: ArduSub refuses ALT_HOLD. We must fail without descending.
    r = rig(mode_ok=False)
    with pytest.raises(SubmergeError, match='Depth sensor is not connected'):
        r.auv.submerge_to_depth(target_depth=2.0, timeout=20.0)
    assert r.vehicle.depth == 0.0, 'descended despite having no depth hold'


def test_failed_preflight_aborts_the_dive_on_the_surface(rig):
    r = rig(preflight_ok=False)
    with pytest.raises(SubmergeError, match='MOT_3_DIRECTION'):
        r.auv.submerge_to_depth(target_depth=2.0, timeout=20.0)
    assert r.vehicle.depth == 0.0
