"""Pure heading-lock logic: capture yaw at forward-start, PID it straight.

Why: MANUAL-mode surge has no yaw feedback, so any thruster imbalance
integrates into a turn (the 2026-07-13 veer-right symptom). This closes the
loop on the ZED 2i IMU yaw.

Sign contract (pinned by tests/test_heading_lock.py):
  * input yaw: REP-103, CCW-positive radians (imu/rpy from orientation_node)
  * output yaw_rate: MovementCommand convention, CW-positive
  * error = wrap(current - target). Drifted CW -> yaw decreased -> error < 0
    -> yaw_rate < 0 = CCW command -> ArduSub's vectored mixer raises the
    RIGHT pair (motors 1 FR & 3 RR) and lowers the LEFT pair (2 FL & 4 RL)
    -> nose returns. The convention flip is folded into the error sign, so
    the PID output IS the yaw_rate command.

Stale handling: the caller passes yaw=None when its source is stale.
Correction zeroes immediately (never steer blind), forward continues for
grace_s hoping the stream recovers, then ABORTED (caller must stop). A
recovery within grace resumes against the ORIGINAL target — the sub is
still supposed to be going that way.
"""
import math
from enum import Enum


def wrap(a):
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class LockState(Enum):
    IDLE = 'idle'
    LOCKED = 'locked'
    STALE_GRACE = 'stale_grace'
    ABORTED = 'aborted'


class HeadingLock:
    def __init__(self, pid, max_yaw_authority=0.4, grace_s=1.0):
        self._pid = pid
        self.max_yaw_authority = float(max_yaw_authority)
        self.grace_s = float(grace_s)
        self._state = LockState.IDLE
        self._target = 0.0
        self._base = 0.0
        self._stale_since = None
        self._last_error = 0.0

    @property
    def state(self):
        return self._state

    @property
    def target_yaw(self):
        return self._target

    @property
    def base_speed(self):
        return self._base

    @property
    def last_error(self):
        return self._last_error

    def start(self, current_yaw, base_speed):
        """Capture the current yaw as the heading to hold and go LOCKED."""
        if not math.isfinite(current_yaw):
            raise ValueError('cannot lock on non-finite yaw')
        self._target = wrap(current_yaw)
        self._base = float(base_speed)
        self._pid.reset()
        self._stale_since = None
        self._last_error = 0.0
        self._state = LockState.LOCKED

    def set_base_speed(self, speed):
        """Change forward speed mid-run WITHOUT re-locking the target."""
        self._base = float(speed)

    def set_target(self, target_yaw):
        """Command a NEW heading to hold, without re-capturing the current yaw.

        This is a deliberate slew: unlike start() (which snaps the target to
        wherever the sub is pointing now), this moves the target to an absolute
        heading so the lock drives there. The PID is reset so the slew is a clean
        step response — the auto-tuner relies on that. No-op unless currently
        LOCKED/STALE_GRACE (a set_target on an idle or aborted lock would hold a
        target nothing is driving toward)."""
        if not math.isfinite(target_yaw):
            raise ValueError('cannot set a non-finite heading target')
        if self._state in (LockState.IDLE, LockState.ABORTED):
            return
        self._target = wrap(target_yaw)
        self._pid.reset()
        self._stale_since = None
        self._last_error = 0.0
        self._state = LockState.LOCKED

    def stop(self):
        """Release the lock; next start() captures a fresh target."""
        self._state = LockState.IDLE
        self._stale_since = None
        self._pid.reset()

    def update(self, yaw, now_s, dt_s):
        """One control tick. yaw=None means the source is stale.

        Returns (surge, yaw_rate, state). On ABORTED the caller must
        publish a stop; ABORTED latches until stop().
        """
        if self._state in (LockState.IDLE, LockState.ABORTED):
            return (0.0, 0.0, self._state)

        if yaw is None:
            if self._stale_since is None:
                self._stale_since = now_s
            if now_s - self._stale_since >= self.grace_s:
                self._state = LockState.ABORTED
                return (0.0, 0.0, self._state)
            # Never steer blind: correction off, forward continues briefly.
            self._state = LockState.STALE_GRACE
            return (self._base, 0.0, self._state)

        self._stale_since = None
        self._state = LockState.LOCKED
        error = wrap(yaw - self._target)
        self._last_error = error
        correction = self._pid.update(error, dt_s)
        correction = max(-self.max_yaw_authority,
                         min(self.max_yaw_authority, correction))
        return (self._base, correction, self._state)
