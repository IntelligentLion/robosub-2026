#!/usr/bin/env python3
"""Behavior tree: Heading Out (Coin Flip) — RoboSub 2026 Task 0 (new ROS API).

The coin flip randomizes the start orientation on the surface. Our approach:
submerge in the start zone (ALT_HOLD depth hold via motion_node), then TURN LEFT
until the front camera detects the GATE. The ffc_rs_26 model has no "gate"
class, so the gate is identified by the role images hung on it: seeing ANY of
compass / hammer_and_wrench (Survey side) or buoy / sos (Rescue side) stops the
turn. Pointing at the gate hands off to the gate task.

NEW API — a pure ROS client, built on **py_trees** (the standard Python behavior
tree library; the old hand-rolled bt.py engine is gone). Install once on the
sub:  pip install py_trees   (or apt: ros-humble-py-trees). It drives
`control.api.Auv` (which publishes intent to motion_node) and subscribes to
`vision/detections` from the already-running front-camera detector node. Bring
the stack + vision up FIRST, in their own processes — or use the mission launch:

    ros2 launch control mission_stack.launch.py        # stack + both cameras
    python3 bt_coinflip.py --depth-ft 3 --yes          # this tree
  (or, cameras/stack by hand:)
    ros2 launch control submerge_hold.launch.py
    ros2 run vision detector

Division of labour (unchanged intent, new plumbing):
  * depth hold      — ArduSub ALT_HOLD, sequenced into by motion_node
  * heading hold    — motion_node's heading lock (yaw)
  * the turn        — operator yaw_rate streamed on motion/cmd via Auv
  * gate detection  — the `detector` node, over vision/detections

Tree (ticked at RATE_HZ; each tick pumps rclpy so callbacks + publishes flow):

  Sequence "HeadingOutCoinFlip" (memory)
  ├─ Submerge            auv.submerge_to_depth(target) — blocks until ALT_HOLD
  │                      confirmed + heading captured (raises -> FAILURE)
  └─ TurnUntilDetect     stream yaw_rate (CCW = left) until any gate label is
                         seen fresh on vision/detections; FAILURE after
                         --search-timeout

⚠ Thrusters WILL spin. The sub ARMS the moment thruster_node connects; the turn
commands real yaw. Props clear, kill switch in reach.
"""

import argparse
import time

import rclpy
from rclpy.node import Node

from auv_msgs.msg import ObjectDetectionArray

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import Timeout

from control.api import Auv, SubmergeError

RATE_HZ = 10.0
FEET_TO_M = 0.3048

# The gate carries role images, not a 'gate' class — seeing ANY of these on the
# front camera means we are pointing at the gate. (survey: compass /
# hammer_and_wrench; rescue: buoy / sos.)
DEFAULT_GATE_LABELS = ('compass', 'hammer_and_wrench', 'buoy', 'sos')


# ─── Detection-aware ROS node ────────────────────────────────────────────────

class CoinFlipMission(Node):
    """Owns the vision subscription + a per-label freshness cache. Auv shares
    THIS node, so movement publishes and detection callbacks pump on one spin."""

    def __init__(self):
        super().__init__('bt_coinflip')
        # label -> (monotonic_seen, confidence). Kept per-label (not just the
        # latest array) so the detector's empty-array frames between hits don't
        # erase a good detection — mirrors the old DetectionMonitor semantics.
        self._last_seen = {}
        self.create_subscription(
            ObjectDetectionArray, 'vision/detections', self._on_det, 10)

    def _on_det(self, msg: ObjectDetectionArray):
        now = time.monotonic()
        for det in msg.detections:
            prev = self._last_seen.get(det.label)
            if prev is None or det.confidence >= prev[1] or now - prev[0] > 0.2:
                self._last_seen[det.label] = (now, float(det.confidence))

    def seen(self, labels, conf, window_s):
        """Return the first of `labels` seen at >= conf within window_s s, else
        None."""
        now = time.monotonic()
        for label in labels:
            entry = self._last_seen.get(label)
            if entry and (now - entry[0]) <= window_s and entry[1] >= conf:
                return label
        return None


# ─── Leaves (py_trees behaviours) ────────────────────────────────────────────

class Submerge(Behaviour):
    """Dive to target depth and hold. Blocking (Auv.submerge_to_depth spins its
    own state watch) — nothing else needs to tick while we descend, and HOLD is
    guaranteed before the turn starts."""

    def __init__(self, auv, target_m, timeout_s=60.0):
        super().__init__('Submerge')
        self.auv = auv
        self.target_m = target_m
        self.timeout_s = timeout_s

    def initialise(self):
        print(f'[bt] Submerge → {self.target_m:.2f} m (ALT_HOLD)')

    def update(self):
        try:
            self.auv.submerge_to_depth(self.target_m, timeout=self.timeout_s)
        except (SubmergeError, ValueError) as exc:
            print(f'[bt] ✗ submerge FAILED — {exc}')
            return Status.FAILURE
        print(f'[bt] ✓ HOLD at {self.target_m:.2f} m — heading captured')
        return Status.SUCCESS


class TurnUntilDetect(Behaviour):
    """Stream a yaw_rate (CCW = turn left, by default) until any gate label is
    seen fresh on vision/detections, then stop. FAILURE after timeout_s.

    Condition-bounded: one axes command per tick (also the motion_node keepalive)
    ending the moment a fresh detection lands. On stop, motion_node's heading
    lock re-captures the new heading and holds it (pointing at the gate)."""

    def __init__(self, auv, mission, labels, conf=0.5, yaw_rate=0.25,
                 det_window_s=1.0, timeout_s=60.0):
        super().__init__('TurnUntilDetect')
        self.auv = auv
        self.mission = mission
        self.labels = tuple(labels)
        self.conf = conf
        self.yaw_rate = yaw_rate          # signed: CW-positive (MovementCommand)
        self.det_window_s = det_window_s
        self.timeout_s = timeout_s
        self._t0 = None

    def initialise(self):
        self._t0 = time.monotonic()
        side = 'CCW (left)' if self.yaw_rate < 0 else 'CW (right)'
        print(f'[bt] TurnUntilDetect: {side} @ |{self.yaw_rate:.2f}| until one '
              f'of {list(self.labels)} seen (≤{self.timeout_s:.0f}s)')

    def update(self):
        hit = self.mission.seen(self.labels, self.conf, self.det_window_s)
        if hit is not None:
            print(f'[bt] ✓ gate detected ("{hit}") — stopping turn')
            return Status.SUCCESS
        if time.monotonic() - self._t0 > self.timeout_s:
            print('[bt] TurnUntilDetect: no gate label seen — FAILURE')
            return Status.FAILURE
        self.auv._publish_axes(yaw_rate=self.yaw_rate)
        return Status.RUNNING

    def terminate(self, new_status):
        # Fires on SUCCESS/FAILURE and on reset-to-INVALID: neutralise. The
        # heading lock re-captures the new heading and holds it.
        self.auv.stop()


# ─── Tree assembly + ticking ─────────────────────────────────────────────────

def build_tree(auv, mission, args):
    target_m = args.depth_ft * FEET_TO_M
    # CW-positive convention: turn left (CCW) = negative yaw_rate.
    yaw_rate = (-abs(args.turn_speed) if args.direction == 'ccw'
                else abs(args.turn_speed))
    seq = Sequence('HeadingOutCoinFlip', memory=True, children=[
        Submerge(auv, target_m, timeout_s=args.dive_timeout),
        TurnUntilDetect(auv, mission, args.labels, conf=args.conf,
                        yaw_rate=yaw_rate, det_window_s=args.det_timeout,
                        timeout_s=args.search_timeout),
    ])
    return Timeout('MissionTimeout', seq, duration=args.mission_timeout)


def run_ros(root, node, rate_hz):
    """Tick the py_trees tree while pumping rclpy — the tree's cadence IS the
    control loop. spin_once both paces the loop and delivers detection callbacks
    / flushes the movement publishes each tick."""
    period = 1.0 / rate_hz
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            root.tick_once()
            if root.status != Status.RUNNING:
                print(f'[bt] tree finished: {root.status.name}')
                return root.status
    except (KeyboardInterrupt, Exception):
        root.stop(Status.INVALID)         # triggers terminate() cleanup (stop)
        raise


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--depth-ft', type=float, default=3.0,
                    help='submerge target (feet)')
    ap.add_argument('--labels', type=lambda s: [x.strip() for x in s.split(',')
                                                 if x.strip()],
                    default=list(DEFAULT_GATE_LABELS),
                    help='comma-separated gate-role labels; ANY stops the turn')
    ap.add_argument('--conf', type=float, default=0.5,
                    help='min detection confidence to count')
    ap.add_argument('--direction', choices=('ccw', 'cw'), default='ccw',
                    help='turn direction while searching (ccw = left)')
    ap.add_argument('--turn-speed', type=float, default=0.25,
                    help='yaw effort while searching, 0..1')
    ap.add_argument('--det-timeout', type=float, default=1.0,
                    help='how fresh a detection must be to count (s)')
    ap.add_argument('--search-timeout', type=float, default=60.0,
                    help='give up if no gate label seen in this long')
    ap.add_argument('--dive-timeout', type=float, default=60.0,
                    help='abort the dive if HOLD not reached in this long')
    ap.add_argument('--mission-timeout', type=float, default=120.0)
    ap.add_argument('--yes', action='store_true',
                    help='skip the confirmation prompt')
    args = ap.parse_args()

    if not args.yes:
        print('BT: Heading Out (Coin Flip) — submerge, turn until a gate role '
              'image is detected. Thrusters WILL spin.')
        if input('type "go" to run: ').strip().lower() != 'go':
            print('Aborted.')
            return 1

    rclpy.init()
    mission = CoinFlipMission()
    auv = Auv(node=mission)               # shares the node: movement + vision co-spin
    try:
        root = build_tree(auv, mission, args)
        status = run_ros(root, mission, RATE_HZ)
        return 0 if status == Status.SUCCESS else 1
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            auv.stop()
        except Exception:
            pass
        auv.close()
        mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
