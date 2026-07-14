"""DepthController — performs the DIVE. It is not a depth PID.

ArduSub's ALT_HOLD already holds depth on the Bar02, and it does it better than
we would: it has the vehicle's thrust model and runs at the autopilot's rate.
Re-implementing that here would mean two controllers fighting over the same
axis. So this class does exactly one thing — drive the sub down until it
reaches the target — and then hands the axis back by commanding zero heave
forever after.

Sign contract: +heave is DOWN, matching MovementCommand.heave. dive_speed is
accepted with either sign and treated as a descent magnitude, because the
original brief wrote it negative (raw MAVLink units, where down is negative)
and an accidental ascent is not a failure mode worth allowing.

Time is injected (`now_s`), never read: the tests must not sleep.
"""
import math
from enum import Enum


class DiveState(Enum):
    IDLE = 'idle'
    DIVING = 'diving'
    AT_DEPTH = 'at_depth'
    TIMEOUT = 'timeout'
    NO_DEPTH_DATA = 'no_depth_data'


class DepthController:
    def __init__(self, tolerance_m=0.15, min_heave=0.12, timeout_s=30.0):
        self.tolerance_m = float(tolerance_m)
        # ArduSub ignores a z within ±THR_DZ (default 100 of 1000) of neutral,
        # so any heave under ~0.1 is silently NO COMMAND AT ALL. Flooring the
        # magnitude here is what stops a small dive_speed from looking like it
        # works while the sub sits on the surface.
        self.min_heave = float(min_heave)
        self.timeout_s = float(timeout_s)
        self._state = DiveState.IDLE
        self._target = 0.0
        self._speed = 0.0
        self._started_at = 0.0

    @property
    def state(self):
        return self._state

    @property
    def target_depth(self):
        return self._target

    def start(self, target_depth_m, dive_speed, now_s):
        if not math.isfinite(target_depth_m):
            raise ValueError(
                f'target depth must be finite (got {target_depth_m})')
        if not math.isfinite(dive_speed):
            raise ValueError(f'dive speed must be finite (got {dive_speed})')
        self._target = float(target_depth_m)
        self._speed = min(1.0, max(self.min_heave, abs(float(dive_speed))))
        self._started_at = float(now_s)
        self._state = DiveState.DIVING

    def stop(self):
        self._state = DiveState.IDLE

    def update(self, depth_m, now_s):
        """(heave, state). depth_m is None when the depth source is stale."""
        # AT_DEPTH and TIMEOUT latch. Re-entering DIVING on a bit of overshoot
        # would put us in a tug-of-war with ALT_HOLD's own depth controller.
        if self._state in (DiveState.IDLE, DiveState.AT_DEPTH,
                           DiveState.TIMEOUT):
            return 0.0, self._state

        if now_s - self._started_at > self.timeout_s:
            self._state = DiveState.TIMEOUT
            return 0.0, self._state

        if depth_m is None or not math.isfinite(depth_m):
            # Never dive blind. A dropout does NOT pause the timeout above — a
            # dead sensor must not be able to hold the dive open forever.
            self._state = DiveState.NO_DEPTH_DATA
            return 0.0, self._state

        if self._target - depth_m <= self.tolerance_m:
            self._state = DiveState.AT_DEPTH
            return 0.0, self._state

        self._state = DiveState.DIVING
        return self._speed, self._state
