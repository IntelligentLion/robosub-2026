#!/usr/bin/env python3
"""Behavior tree: Heading Out (Coin Flip) — RoboSub 2026 Task 0.

The coin flip randomizes the start orientation on the surface. Our approach:
submerge in the start zone, then simply TURN LEFT until the gate is detected.
That's the whole task — pointing at the gate hands off to the gate task.

Tree (ticked at 10 Hz by bt.run):

  Sequence "HeadingOutCoinFlip"
  ├─ Submerge              move_down until depth/sub_depth ≥ target
  │                        (timed fallback if the ZED depth topic is silent)
  └─ TurnLeftUntilGate     rotate CCW until the gate label is seen
                           (fails after --search-timeout — nothing detected)

Leaves are built on the basic-function API in field_common.RampedDriver
(move_down / stop_move + per-tick rotate streaming) and DetectionMonitor /
DepthMonitor.

  python3 bt_coinflip.py --depth-ft 3 --yes

Model path: pass --onnx /path/to/ffc_model.onnx (defaults to the deployed
vision/ffc_rs_26.onnx). ⚠ Thrusters WILL spin — field_common safety notes.
"""

import argparse
import time

import bt
from bt import Status
from field_common import (RATE_HZ, FEET_TO_M, DepthMonitor, DetectionMonitor,
                          add_move_args, find_node, session,
                          spawn_vision_factory)


# ─── Leaves ──────────────────────────────────────────────────────────────────

class Submerge(bt.Leaf):
    """move_down until DepthMonitor reads target depth; timed fallback."""

    def __init__(self, driver, depth_mon, target_m, speed=0.4,
                 timed_fallback_s=4.0, timeout_s=20.0):
        super().__init__('Submerge')
        self.driver, self.depth_mon = driver, depth_mon
        self.target_m = target_m
        self.speed = speed
        self.timed_fallback_s = timed_fallback_s
        self.timeout_s = timeout_s
        self._t0 = None

    def on_start(self):
        self._t0 = time.monotonic()
        print(f'[bt] Submerge → {self.target_m:.2f} m')
        self.driver.move_down(self.speed)

    def update(self):
        elapsed = time.monotonic() - self._t0
        d = self.depth_mon.depth()
        if d is not None:
            if d >= self.target_m:
                print(f'[bt] ✓ depth {d:.2f} m')
                return Status.SUCCESS
            if elapsed > self.timeout_s:
                print(f'[bt] Submerge timeout at {d:.2f} m — proceeding')
                return Status.SUCCESS
        elif elapsed >= self.timed_fallback_s:
            print('[bt] no depth telemetry — timed submerge done')
            return Status.SUCCESS
        return Status.RUNNING

    def on_end(self, status):
        self.driver.stop_move()          # ALT_HOLD locks the depth


class TurnLeftUntilGate(bt.Leaf):
    """Rotate CCW (turn left) until the gate is detected, then stop.

    Condition-bounded, not time-based: one 'rotate_ccw' command per tick at
    the BT's 10 Hz cadence (doubles as the failsafe keepalive), ending the
    moment DetectionMonitor reports a fresh gate. FAILURE only if a full
    --search-timeout passes with no detection.
    """

    def __init__(self, driver, det, label='gate', conf=0.5, speed=0.25,
                 timeout_s=60.0):
        super().__init__('TurnLeftUntilGate')
        self.driver, self.det = driver, det
        self.label, self.conf = label, conf
        self.speed = speed
        self.timeout_s = timeout_s
        self._t0 = None

    def on_start(self):
        self._t0 = time.monotonic()
        print(f'[bt] TurnLeftUntilGate: rotating CCW @ {self.speed:.2f} '
              f'until "{self.label}" seen (≤{self.timeout_s:.0f}s)')
        self.driver.idle()               # single sender: we stream per tick

    def update(self):
        if self.det.seen(self.label, self.conf):
            print('[bt] ✓ gate detected — stopping turn')
            return Status.SUCCESS
        if time.monotonic() - self._t0 > self.timeout_s:
            print('[bt] TurnLeftUntilGate: no gate detected — FAILURE')
            return Status.FAILURE
        self.driver.send('rotate_ccw', self.speed)
        return Status.RUNNING

    def on_end(self, status):
        self.driver.stop_move()          # neutral + depth_hold keepalive


# ─── Tree assembly ───────────────────────────────────────────────────────────

def build_tree(driver, det, depth_mon, args):
    target_m = args.depth_ft * FEET_TO_M
    return bt.Sequence('HeadingOutCoinFlip', [
        Submerge(driver, depth_mon, target_m, speed=args.submerge_speed,
                 timed_fallback_s=args.submerge_time),
        TurnLeftUntilGate(driver, det, args.label, args.conf,
                          speed=args.turn_speed,
                          timeout_s=args.search_timeout),
    ])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    add_move_args(ap, speed=0.4)
    ap.add_argument('--label', default='gate')
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--depth-ft', type=float, default=3.0,
                    help='submerge target (feet)')
    ap.add_argument('--submerge-speed', type=float, default=0.4)
    ap.add_argument('--submerge-time', type=float, default=4.0,
                    help='timed submerge if no depth telemetry')
    ap.add_argument('--turn-speed', type=float, default=0.25,
                    help='CCW rotate effort while searching')
    ap.add_argument('--search-timeout', type=float, default=60.0,
                    help='give up if no gate detected in this long')
    ap.add_argument('--mission-timeout', type=float, default=120.0)
    ap.add_argument('--onnx', default=None,
                    help='forward-camera model .onnx (default: deployed '
                         'ffc_rs_26.onnx)')
    args = ap.parse_args()

    with session(
            spawn_vision_factory(monitor_extra=lambda: [DepthMonitor()],
                                 model_onnx=args.onnx),
            confirm_msg='BT: Heading Out (Coin Flip) — submerge, turn left '
                        'until gate detected. Thrusters WILL spin.',
            skip_confirm=args.yes) as (driver, extra):
        det = find_node(extra, DetectionMonitor)
        depth_mon = find_node(extra, DepthMonitor)

        root = bt.Timeout(build_tree(driver, det, depth_mon, args),
                          args.mission_timeout, name='MissionTimeout')
        status = bt.run(root, rate_hz=RATE_HZ)
        driver.stop_move()
        return 0 if status == Status.SUCCESS else 1


if __name__ == '__main__':
    raise SystemExit(main())
