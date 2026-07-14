"""SubmergeController — sequences preflight/mode/arm/dive/capture into HOLD.

    IDLE → PREFLIGHT → MODE_SET(ALT_HOLD) → ARMING → DIVING → HOLD
                                                                ↓
                          (any step fails) ──────────────→ FAILED

Two orderings here are load-bearing, and both were paid for:

  * PREFLIGHT before anything else. A flipped MOT_x_DIRECTION makes a forward
    command spin the sub or a heave command roll it (2026-07-13). Checking
    after we are already wet is too late.

  * ALT_HOLD CONFIRMED before the first centimetre of heave. ArduSub refuses
    ALT_HOLD when the depth sensor is missing and silently stays in its old
    mode. Diving first and checking later means descending with no depth hold
    and no way to stop — so if the mode cannot be confirmed, we fail while
    still on the surface, having commanded zero thrust.

ARMING waits for the vehicle to be armed; it does not arm it. Arming (and
re-arming) belongs to thruster_node, and two arming authorities is exactly the
overlap this design exists to remove.

Side effects go through the `Effects` protocol, split into request/result so the
caller can drive it from a ROS timer without blocking on a service future.
Time is injected; nothing here sleeps or reads a clock.
"""
import math
from enum import Enum

from control.depth_controller import DiveState


class SubmergeState(Enum):
    IDLE = 'idle'
    PREFLIGHT = 'preflight'
    MODE_SET = 'mode_set'
    ARMING = 'arming'
    DIVING = 'diving'
    HOLD = 'hold'
    FAILED = 'failed'


class Effects:
    """What SubmergeController needs the outside world to do for it.

    Every side effect is split in two: `request_*` fires and returns
    immediately, `*_result` returns None while still pending. That keeps the
    sequencer callable from inside a ROS timer, where blocking on a service
    future would deadlock a single-threaded executor.
    """

    def request_preflight(self):
        raise NotImplementedError

    def preflight_result(self):
        """(ok, reason), or None while pending."""
        raise NotImplementedError

    def request_mode(self, name):
        raise NotImplementedError

    def mode_result(self):
        """(ok, reason), or None while pending."""
        raise NotImplementedError

    def is_armed(self):
        raise NotImplementedError


class SubmergeController:
    HOLD_MODE = 'ALT_HOLD'

    def __init__(self, depth_ctl, heading_ctl, effects, phase_timeout_s=15.0):
        self._depth = depth_ctl
        self._heading = heading_ctl
        self._fx = effects
        self.phase_timeout_s = float(phase_timeout_s)
        self._state = SubmergeState.IDLE
        self._reason = ''
        self._phase_started = 0.0
        self._target = 0.0
        self._dive_speed = 0.0

    @property
    def state(self):
        return self._state

    @property
    def failure_reason(self):
        return self._reason

    @property
    def target_depth(self):
        return self._target

    def start(self, target_depth_m, dive_speed, now_s):
        if not math.isfinite(target_depth_m) or target_depth_m <= 0.0:
            raise ValueError(
                'target depth must be a positive finite depth below the '
                f'surface (got {target_depth_m})')
        self._target = float(target_depth_m)
        self._dive_speed = float(dive_speed)
        self._reason = ''
        self._heading.stop()
        self._depth.stop()
        self._enter(SubmergeState.PREFLIGHT, now_s)
        self._fx.request_preflight()

    def abort(self, reason):
        self._reason = reason
        self._state = SubmergeState.FAILED
        self._heading.stop()
        self._depth.stop()

    def stop(self):
        self._state = SubmergeState.IDLE
        self._reason = ''
        self._heading.stop()
        self._depth.stop()

    def _enter(self, state, now_s):
        self._state = state
        self._phase_started = float(now_s)

    def _timed_out(self, now_s):
        return (now_s - self._phase_started) > self.phase_timeout_s

    def update(self, depth_m, yaw_rad, now_s):
        """One tick. Returns (heave, state). heave is 0 in every state except
        DIVING — nothing else on this path may touch the axis."""
        if self._state in (SubmergeState.IDLE, SubmergeState.FAILED,
                           SubmergeState.HOLD):
            return 0.0, self._state

        if self._state is SubmergeState.PREFLIGHT:
            result = self._fx.preflight_result()
            if result is None:
                if self._timed_out(now_s):
                    self.abort('preflight timed out — is thruster_node up?')
                return 0.0, self._state
            ok, reason = result
            if not ok:
                self.abort(f'preflight failed: {reason}')
                return 0.0, self._state
            self._enter(SubmergeState.MODE_SET, now_s)
            self._fx.request_mode(self.HOLD_MODE)
            return 0.0, self._state

        if self._state is SubmergeState.MODE_SET:
            result = self._fx.mode_result()
            if result is None:
                if self._timed_out(now_s):
                    self.abort(f'{self.HOLD_MODE} request timed out')
                return 0.0, self._state
            ok, reason = result
            if not ok:
                # Not confirmed = no depth hold. Fail dry.
                self.abort(f'cannot enter {self.HOLD_MODE}: {reason}')
                return 0.0, self._state
            self._enter(SubmergeState.ARMING, now_s)
            return 0.0, self._state

        if self._state is SubmergeState.ARMING:
            if not self._fx.is_armed():
                if self._timed_out(now_s):
                    self.abort('vehicle did not arm — check pre-arm checks / '
                               'safety switch')
                return 0.0, self._state
            self._depth.start(self._target, self._dive_speed, now_s)
            self._enter(SubmergeState.DIVING, now_s)
            # Fall through into the DIVING block below rather than returning:
            # returning here would burn a whole control cycle reporting DIVING
            # while commanding zero heave.

        if self._state is SubmergeState.DIVING:
            heave, dive_state = self._depth.update(depth_m, now_s)
            if dive_state is DiveState.TIMEOUT:
                self.abort(
                    f'dive timeout — never reached {self._target:.2f} m')
                return 0.0, self._state
            if dive_state is DiveState.AT_DEPTH:
                if yaw_rad is None or not math.isfinite(yaw_rad):
                    # Locking a garbage heading would steer us into a wall on
                    # the first forward command. Reaching depth without a yaw
                    # source is a failure, not a HOLD.
                    self.abort('reached depth but yaw is unavailable — cannot '
                               'capture a heading to hold')
                    return 0.0, self._state
                self._heading.start(yaw_rad, base_speed=0.0)
                self._enter(SubmergeState.HOLD, now_s)
                return 0.0, self._state
            # DIVING or NO_DEPTH_DATA: DepthController already returns 0 heave
            # on a dropout, and its own timeout bounds how long we wait.
            return heave, self._state

        return 0.0, self._state
