# heading_lock Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Lock the ZED 2i IMU yaw when forward motion starts and PID it straight, balancing motors 1–4 through ArduSub's mixer (`MANUAL_CONTROL` x + r), per `docs/superpowers/specs/2026-07-14-heading-lock-design.md`.

**Architecture:** Pure-logic `HeadingLock` class (state machine + control law, unit-tested) + thin `heading_lock_node` ROS wrapper in the existing `control` package. The node subscribes zeroed ZED-IMU yaw (`imu/rpy`) and a Float32 speed command, publishes `movement_command{command:'axes', surge, yaw_rate}` to the existing `ThrusterController`, plus 8 debug topics. The shared `PID` class is extracted from `autonomous_controller.py` into `control/pid.py` first.

**Tech Stack:** Python 3.10 / ROS 2 Humble (rclpy) / auv_msgs / pytest.

## Global Constraints

- This machine IS the vehicle (Jetson Orin). Never arm the Pixhawk while executing this plan; bench steps must not start `thruster_node` (it arms on connect). **WATER TEST** steps are for humans.
- Sign conventions (load-bearing): input yaw on `imu/rpy` is REP-103 **CCW-positive** radians; `MovementCommand.yaw_rate` is **CW-positive**. `error = wrap(current_yaw − target_yaw)`; PID output feeds `yaw_rate` directly (drifted CW → error < 0 → yaw_rate < 0 = CCW command → nose returns).
- Motor debug topics are commanded INTENT, not measured PWM: motor2 = motor4 = base + correction (left FL/RL), motor1 = motor3 = base − correction (right FR/RR).
- Workspace must be sourced for tests and runs: `source /opt/ros/humble/setup.bash && source install/setup.bash`.
- Build with symlink install (stale `install/` is the recurring "code not taking effect" trap): `colcon build --symlink-install --packages-select control`.
- Run pytest as: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/ -v`. Test files MUST be named `test_*.py` under `tests/` only (pytest.ini restricts collection for hardware safety — never relax it).
- Single writer: while `heading_lock_node` drives `movement_command`, root field scripts must not run (they send raw MANUAL_CONTROL).
- Commit after every task, conventional style, end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `src/control/control/pid.py` (new) | shared `PID` class (moved verbatim from autonomous_controller) | 1 |
| `tests/test_pid.py` (new) | PID unit tests | 1 |
| `src/control/control/autonomous_controller.py` | drop inline PID, import from `control.pid` | 1 |
| `src/control/control/heading_lock.py` (new) | `wrap`, `LockState`, `HeadingLock` — pure logic | 2 |
| `tests/test_heading_lock.py` (new) | lock/sign/wrap/stale/clamp tests | 2 |
| `src/control/control/heading_lock_node.py` (new) | ROS wrapper: subs, pubs, params, tick | 3 |
| `src/control/setup.py` | `heading_lock_node` console script | 3 |
| `tests/test_heading_lock_node.py` (new) | node wiring smoke tests | 3 |
| `README.md` + `docs/water-tests/2026-heading-lock.md` (new) | run commands, bench sign-check, water procedure | 4 |

---

### Task 1: Extract PID into `control/pid.py`

**Files:**
- Create: `src/control/control/pid.py`
- Modify: `src/control/control/autonomous_controller.py` (PID class ~lines 76–133; imports ~line 22)
- Test: `tests/test_pid.py`

**Interfaces:**
- Produces: `control.pid.PID(kp, ki, kd, limit=1.0, i_limit=0.3)` with `.update(error, dt) -> float`, `.reset()`, `.set_gains(kp=None, ki=None, kd=None, limit=None, i_limit=None)`. Tasks 2–3 import exactly this.
- `control.autonomous_controller.PID` must still resolve (re-export via import) — existing tooling/scripts may reference it.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_pid.py
"""PID unit tests — the class is shared by autonomous_controller and
heading_lock, so its behavior is pinned here."""
import math

from control.pid import PID


def test_proportional_only():
    pid = PID(kp=2.0, ki=0.0, kd=0.0)
    # first update initializes prev_error -> derivative 0, integral tiny
    assert pid.update(0.3, 0.05) == 0.3 * 2.0


def test_output_clamped_to_limit():
    pid = PID(kp=100.0, ki=0.0, kd=0.0, limit=0.5)
    assert pid.update(1.0, 0.05) == 0.5
    assert pid.update(-1.0, 0.05) == -0.5


def test_integral_windup_clamped():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, limit=10.0, i_limit=0.2)
    for i in range(100):
        out = pid.update(1.0, 0.1)          # raw integral would reach 10.0
    assert out == 0.2                        # ki * clamped integral


def test_reset_clears_state():
    pid = PID(kp=0.0, ki=1.0, kd=1.0, limit=10.0, i_limit=5.0)
    pid.update(1.0, 0.1)
    pid.update(1.0, 0.1)
    pid.reset()
    # zero error after reset -> exactly zero output (no leftover I or D)
    assert pid.update(0.0, 0.1) == 0.0


def test_set_gains_live():
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    pid.set_gains(kp=3.0)
    assert pid.update(1.0, 0.05) == 3.0


def test_nonfinite_error_neutralized():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    assert pid.update(float('nan'), 0.05) == 0.0
    assert pid.update(math.inf, 0.05) == 0.0


def test_bad_dt_neutralized():
    pid = PID(kp=1.0, ki=0.0, kd=0.0)
    assert pid.update(1.0, 0.0) == 0.0       # dt <= 0
    assert pid.update(1.0, 2.0) == 0.0       # dt > 1.0


def test_reexport_from_autonomous_controller():
    # existing code/tools reference the old location; must stay importable
    from control.autonomous_controller import PID as ReExported
    assert ReExported is PID
```

- [ ] **Step 2: Run to verify failure**

Run: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/test_pid.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'control.pid'`.

- [ ] **Step 3: Create `src/control/control/pid.py`**

The class body is MOVED VERBATIM from `autonomous_controller.py` (do not re-derive it — copy, then delete the original in Step 4). Only the finite-guard helper is inlined:

```python
# src/control/control/pid.py
"""Shared PID with output limit + integral anti-windup.

Extracted verbatim from autonomous_controller (2026-07-14) so heading_lock
can reuse it. Semantics unchanged: derivative clamped to ±10, integral
clamped to ±i_limit, output clamped to ±limit, non-finite inputs/outputs
reset the controller and return 0.
"""
import math


def _is_finite(v):
    return math.isfinite(v)


class PID:
    def __init__(self, kp, ki, kd, limit=1.0, i_limit=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i_limit = i_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def set_gains(self, kp=None, ki=None, kd=None, limit=None, i_limit=None):
        """Live-update gains (called from the on-set-parameters callback).

        Any of kp/ki/kd/limit/i_limit may be None to leave unchanged. This
        lets you tune at the pool via `ros2 param set` without restarting.
        """
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd
        if limit is not None: self.limit = limit
        if i_limit is not None: self.i_limit = i_limit

    def update(self, error, dt):
        if dt <= 0 or dt > 1.0:
            return 0.0
        if not _is_finite(error):
            self.reset()
            return 0.0
        if not self._initialized:
            self._prev_error = error
            self._initialized = True

        self._integral += error * dt
        self._integral = max(-self.i_limit, min(self.i_limit, self._integral))

        derivative = (error - self._prev_error) / dt
        derivative = max(-10.0, min(10.0, derivative))
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        if not _is_finite(output):
            self.reset()
            return 0.0
        return max(-self.limit, min(self.limit, output))
```

- [ ] **Step 4: Point `autonomous_controller.py` at it**

Delete the entire inline `class PID: ...` block (from `class PID:` down to the blank lines before `class AutonomousController(Node):`). Add to the `from control.centering import (...)` import area:

```python
from control.pid import PID
```

Keep `autonomous_controller`'s own `_is_finite` — it is still used by `_pose_cb`/`_depth_cb` (lines ~305/320).

- [ ] **Step 5: Build + run tests**

```bash
cd ~/robosub2026/robosub-2026
colcon build --symlink-install --packages-select control
source install/setup.bash
python3 -m pytest tests/test_pid.py tests/ -v
```
Expected: all `test_pid.py` PASS; full suite green (test_msg_compat, test_imu_math, test_diagnose_veer unaffected).

- [ ] **Step 6: Commit**

```bash
git add src/control/control/pid.py src/control/control/autonomous_controller.py tests/test_pid.py
git commit -m "refactor(control): extract shared PID into control/pid.py"
```

---

### Task 2: `HeadingLock` pure logic

**Files:**
- Create: `src/control/control/heading_lock.py`
- Test: `tests/test_heading_lock.py`

**Interfaces:**
- Consumes: `control.pid.PID` (Task 1).
- Produces (Task 3 relies on these exact names):
  - `wrap(a) -> float` — radians to [-π, π]
  - `LockState` enum: `IDLE`, `LOCKED`, `STALE_GRACE`, `ABORTED`
  - `HeadingLock(pid, max_yaw_authority=0.4, grace_s=1.0)` with:
    - `.start(current_yaw, base_speed)` — capture target, reset PID, → LOCKED
    - `.update(yaw_or_none, now_s, dt_s) -> (surge, yaw_rate, LockState)`
    - `.stop()` — → IDLE, reset PID
    - `.set_base_speed(speed)` — update speed WITHOUT re-locking target
    - properties `.state`, `.target_yaw`, `.base_speed`, `.last_error`
    - mutable attrs `.max_yaw_authority`, `.grace_s` (live tuning)

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_heading_lock.py
"""HeadingLock control-law tests. The sign convention here is THE contract:
imu/rpy yaw is REP-103 CCW-positive, MovementCommand.yaw_rate is CW-positive,
error = wrap(current - target) feeds the PID directly."""
import math

import pytest

from control.heading_lock import HeadingLock, LockState, wrap
from control.pid import PID


def make_lock(kp=1.0, ki=0.0, kd=0.0, authority=0.4, grace_s=1.0):
    return HeadingLock(PID(kp=kp, ki=ki, kd=kd, limit=1.0, i_limit=0.3),
                       max_yaw_authority=authority, grace_s=grace_s)


def test_wrap():
    assert wrap(math.radians(340)) == pytest.approx(math.radians(-20))
    assert wrap(math.radians(-340)) == pytest.approx(math.radians(20))
    assert wrap(0.0) == 0.0


def test_start_captures_target_and_locks():
    lock = make_lock()
    assert lock.state is LockState.IDLE
    lock.start(0.7, base_speed=0.3)
    assert lock.state is LockState.LOCKED
    assert lock.target_yaw == pytest.approx(0.7)
    assert lock.base_speed == pytest.approx(0.3)


def test_idle_update_is_neutral():
    lock = make_lock()
    assert lock.update(0.5, now_s=0.0, dt_s=0.05) == (0.0, 0.0, LockState.IDLE)


def test_cw_drift_gets_ccw_correction():
    # REP-103: clockwise drift DECREASES yaw. Correction must be negative
    # (CCW on the CW-positive yaw_rate axis) -> mixer raises right pair.
    lock = make_lock(kp=1.0)
    lock.start(0.0, base_speed=0.3)
    surge, yaw_rate, state = lock.update(-0.1, now_s=0.0, dt_s=0.05)
    assert state is LockState.LOCKED
    assert surge == pytest.approx(0.3)          # forward speed untouched
    assert yaw_rate == pytest.approx(-0.1)      # kp * wrap(-0.1 - 0.0)
    assert lock.last_error == pytest.approx(-0.1)


def test_ccw_drift_gets_cw_correction():
    lock = make_lock(kp=1.0)
    lock.start(0.0, base_speed=0.3)
    _, yaw_rate, _ = lock.update(0.1, now_s=0.0, dt_s=0.05)
    assert yaw_rate == pytest.approx(0.1)


def test_error_wraps_across_pi():
    # target +170deg, current -170deg: shortest path is +20deg CCW of target
    # -> error +20deg -> CW (positive) correction, NOT -340deg.
    lock = make_lock(kp=1.0)
    lock.start(math.radians(170), base_speed=0.3)
    _, yaw_rate, _ = lock.update(math.radians(-170), now_s=0.0, dt_s=0.05)
    assert yaw_rate == pytest.approx(math.radians(20), abs=1e-9)


def test_correction_clamped_to_authority():
    lock = make_lock(kp=100.0, authority=0.4)
    lock.start(0.0, base_speed=0.3)
    _, yaw_rate, _ = lock.update(-0.5, now_s=0.0, dt_s=0.05)
    assert yaw_rate == -0.4
    _, yaw_rate, _ = lock.update(0.5, now_s=0.1, dt_s=0.05)
    assert yaw_rate == 0.4


def test_stale_grace_then_abort():
    lock = make_lock(grace_s=1.0)
    lock.start(0.0, base_speed=0.3)
    # stale: correction zeroed, forward continues
    surge, yaw_rate, state = lock.update(None, now_s=10.0, dt_s=0.05)
    assert (surge, yaw_rate, state) == (0.3, 0.0, LockState.STALE_GRACE)
    surge, yaw_rate, state = lock.update(None, now_s=10.9, dt_s=0.05)
    assert state is LockState.STALE_GRACE
    # past grace: abort, everything neutral
    surge, yaw_rate, state = lock.update(None, now_s=11.0, dt_s=0.05)
    assert (surge, yaw_rate, state) == (0.0, 0.0, LockState.ABORTED)
    # aborted latches until stop(): even a fresh sample stays aborted
    assert lock.update(0.0, now_s=11.1, dt_s=0.05) == (
        0.0, 0.0, LockState.ABORTED)


def test_recovery_within_grace_keeps_original_target():
    lock = make_lock(kp=1.0, grace_s=1.0)
    lock.start(0.5, base_speed=0.3)
    _, _, state = lock.update(None, now_s=0.0, dt_s=0.05)
    assert state is LockState.STALE_GRACE
    _, yaw_rate, state = lock.update(0.4, now_s=0.5, dt_s=0.05)
    assert state is LockState.LOCKED
    assert lock.target_yaw == pytest.approx(0.5)     # NOT re-locked at 0.4
    assert yaw_rate == pytest.approx(-0.1)


def test_second_stale_episode_gets_fresh_grace_budget():
    lock = make_lock(grace_s=1.0)
    lock.start(0.0, base_speed=0.3)
    lock.update(None, now_s=0.0, dt_s=0.05)            # grace starts
    lock.update(0.0, now_s=0.5, dt_s=0.05)             # recovers
    _, _, state = lock.update(None, now_s=5.0, dt_s=0.05)
    assert state is LockState.STALE_GRACE              # new episode, new clock
    _, _, state = lock.update(None, now_s=5.9, dt_s=0.05)
    assert state is LockState.STALE_GRACE


def test_set_base_speed_does_not_relock():
    lock = make_lock()
    lock.start(0.5, base_speed=0.3)
    lock.set_base_speed(0.6)
    surge, _, _ = lock.update(0.5, now_s=0.0, dt_s=0.05)
    assert surge == pytest.approx(0.6)
    assert lock.target_yaw == pytest.approx(0.5)


def test_stop_resets_integrator():
    lock = make_lock(kp=0.0, ki=1.0)
    lock.start(0.0, base_speed=0.3)
    for i in range(20):                                # wind the integral up
        lock.update(0.3, now_s=i * 0.05, dt_s=0.05)
    lock.stop()
    assert lock.state is LockState.IDLE
    lock.start(0.0, base_speed=0.3)
    _, yaw_rate, _ = lock.update(0.0, now_s=100.0, dt_s=0.05)
    assert yaw_rate == 0.0                             # no windup carryover


def test_start_rejects_nonfinite_yaw():
    lock = make_lock()
    with pytest.raises(ValueError):
        lock.start(float('nan'), base_speed=0.3)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_heading_lock.py -v` (workspace sourced)
Expected: FAIL — `ModuleNotFoundError: No module named 'control.heading_lock'`.

- [ ] **Step 3: Implement**

```python
# src/control/control/heading_lock.py
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
```

- [ ] **Step 4: Run tests**

Run: `python3 -m pytest tests/test_heading_lock.py tests/ -v`
Expected: all 13 new tests PASS; full suite green.

- [ ] **Step 5: Commit**

```bash
git add src/control/control/heading_lock.py tests/test_heading_lock.py
git commit -m "feat(control): HeadingLock — yaw capture + PID straight-line logic"
```

---

### Task 3: `heading_lock_node` ROS wrapper

**Files:**
- Create: `src/control/control/heading_lock_node.py`
- Modify: `src/control/setup.py` (entry_points, line ~22)
- Test: `tests/test_heading_lock_node.py`

**Interfaces:**
- Consumes: `control.pid.PID` (Task 1); `control.heading_lock.HeadingLock`, `LockState` (Task 2); `auv_msgs/MovementCommand` axes mode (existing, consumed by `ThrusterController`).
- Produces: executable `ros2 run control heading_lock_node`; topics `heading_lock/cmd` (in), `movement_command` (out), `heading_lock/{current_yaw,target_yaw,error,pid_output,motor1,motor2,motor3,motor4}` (out, Float32).

- [ ] **Step 1: Write the failing smoke tests**

These test the node's wiring (params, lock/refuse/unlock paths) by calling callbacks directly — no executor, no hardware. Message-flow verification happens on the bench (Task 4).

```python
# tests/test_heading_lock_node.py
"""heading_lock_node wiring tests: cmd handling, speed clamp, staleness.
Callbacks invoked directly; no spinning, no hardware."""
import pytest
import rclpy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

from control.heading_lock import LockState
from control.heading_lock_node import HeadingLockNode


def make_node():
    return HeadingLockNode()


def setup_module(_m):
    rclpy.init()


def teardown_module(_m):
    rclpy.shutdown()


def yaw_msg(yaw):
    m = Vector3Stamped()
    m.vector.z = yaw
    return m


def test_cmd_without_yaw_is_refused():
    node = make_node()
    try:
        node._on_cmd(Float32(data=0.4))
        assert node._lock.state is LockState.IDLE
    finally:
        node.destroy_node()


def test_cmd_locks_and_clamps_speed():
    node = make_node()
    try:
        node._on_yaw(yaw_msg(0.3))
        node._on_cmd(Float32(data=5.0))          # way over max_forward_speed
        assert node._lock.state is LockState.LOCKED
        assert node._lock.target_yaw == pytest.approx(0.3)
        assert node._lock.base_speed == 0.6      # max_forward_speed default
    finally:
        node.destroy_node()


def test_second_cmd_updates_speed_without_relock():
    node = make_node()
    try:
        node._on_yaw(yaw_msg(0.3))
        node._on_cmd(Float32(data=0.3))
        node._on_yaw(yaw_msg(0.9))               # sub has drifted
        node._on_cmd(Float32(data=0.5))          # speed change only
        assert node._lock.target_yaw == pytest.approx(0.3)   # NOT re-captured
        assert node._lock.base_speed == 0.5
    finally:
        node.destroy_node()


def test_zero_and_nan_cmd_unlock():
    node = make_node()
    try:
        node._on_yaw(yaw_msg(0.0))
        node._on_cmd(Float32(data=0.4))
        node._on_cmd(Float32(data=0.0))
        assert node._lock.state is LockState.IDLE
        node._on_cmd(Float32(data=0.4))
        node._on_cmd(Float32(data=float('nan')))
        assert node._lock.state is LockState.IDLE
    finally:
        node.destroy_node()


def test_declared_params_have_spec_defaults():
    node = make_node()
    try:
        for name, want in [('kp', 1.2), ('ki', 0.0), ('kd', 0.3),
                           ('max_yaw_authority', 0.4),
                           ('max_forward_speed', 0.6),
                           ('stale_timeout_s', 0.5), ('grace_s', 1.0),
                           ('rate_hz', 20.0)]:
            assert node.get_parameter(name).value == want, name
        assert node.get_parameter('yaw_topic').value == 'imu/rpy'
    finally:
        node.destroy_node()
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_heading_lock_node.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'control.heading_lock_node'`.

- [ ] **Step 3: Implement the node**

```python
#!/usr/bin/env python3
# src/control/control/heading_lock_node.py
"""heading_lock_node — drive straight on the ZED 2i IMU yaw.

Subscribes:
  - imu/rpy           (geometry_msgs/Vector3Stamped) — vector.z = yaw rad,
                       REP-103 CCW+ (orientation_node); topic via yaw_topic
  - heading_lock/cmd  (std_msgs/Float32) — data > 0: lock current yaw, drive
                       forward at that speed (clamped to max_forward_speed);
                       repeat while locked = speed change, target kept;
                       data <= 0 or non-finite: stop + unlock

Publishes:
  - movement_command  (auv_msgs/MovementCommand) — 'axes' (surge+yaw_rate)
                       every tick while active; 'stop' on unlock/abort
  - heading_lock/{current_yaw,target_yaw,error,pid_output} (Float32, rad)
  - heading_lock/motor1..motor4 (Float32) — commanded INTENT into the mixer
                       (motor2=motor4=base+corr left, motor1=motor3=base-corr
                       right), NOT measured PWM (that belongs to ArduSub)

Control law lives in control.heading_lock (pure, unit-tested); this file is
wiring only: staleness detection (node-clock arrival age, immune to source
clock skew), live-tunable params, debug topics.

Safety: yaw stale > stale_timeout_s -> correction zeroed; still stale after
grace_s -> stop + unlock (blind forward is how the veer-right symptom hits
walls). Any tick exception -> stop + unlock. heave stays 0 so ALT_HOLD keeps
owning depth.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from auv_msgs.msg import MovementCommand

from control.heading_lock import HeadingLock, LockState
from control.pid import PID

DEBUG_TOPICS = ('current_yaw', 'target_yaw', 'error', 'pid_output',
                'motor1', 'motor2', 'motor3', 'motor4')


def _pf(value, default):
    """Total float coercion for ROS param reads (matches autonomous_controller)."""
    try:
        return float(value) if value is not None else float(default)
    except (TypeError, ValueError):
        return float(default)


class HeadingLockNode(Node):
    def __init__(self):
        super().__init__('heading_lock_node')
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)     # PD start; raise at the pool
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('i_limit', 0.3)
        self.declare_parameter('max_yaw_authority', 0.4)
        self.declare_parameter('max_forward_speed', 0.6)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('grace_s', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('yaw_topic', 'imu/rpy')

        def p(name, default):
            return _pf(self.get_parameter(name).value, default)

        self._pid = PID(kp=p('kp', 1.2), ki=p('ki', 0.0), kd=p('kd', 0.3),
                        limit=1.0, i_limit=p('i_limit', 0.3))
        self._lock = HeadingLock(
            self._pid,
            max_yaw_authority=p('max_yaw_authority', 0.4),
            grace_s=p('grace_s', 1.0))
        self._stale_timeout_s = p('stale_timeout_s', 0.5)
        self._max_forward_speed = p('max_forward_speed', 0.6)

        self._last_yaw = None          # (arrival_monotonic_s, yaw_rad)
        self._last_tick = time.monotonic()
        self._warned_stale = False

        self.add_on_set_parameters_callback(self._on_params)

        self._cmd_pub = self.create_publisher(
            MovementCommand, 'movement_command', 10)
        self._dbg = {
            name: self.create_publisher(Float32, f'heading_lock/{name}', 10)
            for name in DEBUG_TOPICS}
        yaw_topic = str(self.get_parameter('yaw_topic').value)
        self.create_subscription(
            Vector3Stamped, yaw_topic, self._on_yaw, 10)
        self.create_subscription(
            Float32, 'heading_lock/cmd', self._on_cmd, 10)
        rate_hz = max(1.0, p('rate_hz', 20.0))
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'heading_lock_node up — yaw from "{yaw_topic}", '
            f'{rate_hz:.0f} Hz, authority ±{self._lock.max_yaw_authority}')

    # ─── inputs ─────────────────────────────────────────────────────

    def _on_yaw(self, msg: Vector3Stamped):
        if math.isfinite(msg.vector.z):
            self._last_yaw = (time.monotonic(), msg.vector.z)

    def _fresh_yaw(self, now_s):
        """Latest yaw, or None if stale/never seen (arrival-time based)."""
        if self._last_yaw is None:
            return None
        arrival, yaw = self._last_yaw
        if now_s - arrival > self._stale_timeout_s:
            return None
        return yaw

    def _on_cmd(self, msg: Float32):
        speed = msg.data
        if not math.isfinite(speed) or speed <= 0.0:
            if self._lock.state is not LockState.IDLE:
                self.get_logger().info('cmd <= 0 — stop + unlock')
            self._lock.stop()
            self._publish_stop()
            return
        speed = min(speed, self._max_forward_speed)
        if self._lock.state in (LockState.LOCKED, LockState.STALE_GRACE):
            self._lock.set_base_speed(speed)     # speed change, target kept
            return
        yaw = self._fresh_yaw(time.monotonic())
        if yaw is None:
            self.get_logger().error(
                'cmd refused — no fresh yaw to lock '
                '(is orientation_node publishing?)')
            self._publish_stop()
            return
        self._lock.start(yaw, speed)
        self._warned_stale = False
        self.get_logger().info(
            f'heading LOCKED at {math.degrees(self._lock.target_yaw):+.1f}° '
            f'— forward {speed:.2f}')

    # ─── control tick ───────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if self._lock.state is LockState.IDLE:
            return
        try:
            yaw = self._fresh_yaw(now)
            surge, yaw_rate, state = self._lock.update(yaw, now, dt)

            if state is LockState.ABORTED:
                self.get_logger().error(
                    f'yaw stale > {self._lock.grace_s:.1f}s grace — '
                    'STOP + unlock')
                self._lock.stop()
                self._publish_stop()
                return
            if state is LockState.STALE_GRACE and not self._warned_stale:
                self.get_logger().warn(
                    'yaw stale — correction zeroed, forward continues '
                    f'{self._lock.grace_s:.1f}s grace')
                self._warned_stale = True
            elif state is LockState.LOCKED:
                self._warned_stale = False

            out = MovementCommand()
            out.command = 'axes'
            out.surge = float(surge)
            out.yaw_rate = float(yaw_rate)
            self._cmd_pub.publish(out)
            self._publish_debug(yaw, surge, yaw_rate)
        except Exception as e:
            self.get_logger().error(f'tick error: {e} — stop + unlock')
            self._lock.stop()
            self._publish_stop()

    # ─── outputs ────────────────────────────────────────────────────

    def _publish_stop(self):
        msg = MovementCommand()
        msg.command = 'stop'
        self._cmd_pub.publish(msg)

    def _publish_debug(self, yaw, surge, yaw_rate):
        left = surge + yaw_rate       # motors 2 (FL) & 4 (RL) intent
        right = surge - yaw_rate      # motors 1 (FR) & 3 (RR) intent
        values = {
            'current_yaw': yaw if yaw is not None else float('nan'),
            'target_yaw': self._lock.target_yaw,
            'error': self._lock.last_error,
            'pid_output': yaw_rate,
            'motor1': right, 'motor2': left,
            'motor3': right, 'motor4': left,
        }
        for name, v in values.items():
            self._dbg[name].publish(Float32(data=float(v)))

    # ─── live tuning ────────────────────────────────────────────────

    def _on_params(self, params):
        for prm in params:
            name, val = prm.name, prm.value
            if name in ('kp', 'ki', 'kd'):
                self._pid.set_gains(**{name: _pf(val, 0.0)})
            elif name == 'i_limit':
                self._pid.set_gains(i_limit=_pf(val, 0.3))
            elif name == 'max_yaw_authority':
                self._lock.max_yaw_authority = _pf(val, 0.4)
            elif name == 'grace_s':
                self._lock.grace_s = _pf(val, 1.0)
            elif name == 'stale_timeout_s':
                self._stale_timeout_s = _pf(val, 0.5)
            elif name == 'max_forward_speed':
                self._max_forward_speed = _pf(val, 0.6)
            # rate_hz / yaw_topic changes need a restart; accepted silently
        return SetParametersResult(successful=True)

    def destroy_node(self):
        # Best effort: leave the thruster node commanding neutral.
        try:
            self._publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadingLockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Add the console script**

In `src/control/setup.py`, `entry_points` becomes:

```python
    entry_points={
        'console_scripts': [
            'autonomous_controller = control.autonomous_controller:main',
            'heading_lock_node = control.heading_lock_node:main',
        ],
    },
```

- [ ] **Step 5: Build + run tests**

```bash
cd ~/robosub2026/robosub-2026
colcon build --symlink-install --packages-select control
source install/setup.bash
python3 -m pytest tests/test_heading_lock_node.py tests/ -v
ros2 run control heading_lock_node --ros-args -p yaw_topic:=imu/rpy &
sleep 3 && ros2 topic list | grep heading_lock && kill %1
```
Expected: all tests PASS; `ros2 topic list` shows `/heading_lock/cmd` and the 8 debug topics. (Running the node alone is safe: it publishes to `movement_command` but nothing consumes it — do NOT start `thruster_node` here, it arms the FC.)

- [ ] **Step 6: Commit**

```bash
git add src/control/control/heading_lock_node.py src/control/setup.py tests/test_heading_lock_node.py
git commit -m "feat(control): heading_lock_node — ZED-IMU straight-line driving"
```

---

### Task 4: Docs — run guide, bench sign-check, water procedure

**Files:**
- Create: `docs/water-tests/2026-heading-lock.md`
- Modify: `README.md` (add a "Heading lock (drive straight)" section near the other node run instructions)

- [ ] **Step 1: Write `docs/water-tests/2026-heading-lock.md`**

```markdown
# heading_lock bench + water verification

Spec: docs/superpowers/specs/2026-07-14-heading-lock-design.md

## Bench sign-check (dry, FC untouched — do NOT start thruster_node)

1. ZED + orientation up (owns the ZED; detector must be off; ethernet/5 GHz
   — ZED USB3 jams 2.4 GHz WiFi):
   `ros2 launch imu imu_viz.launch.py rviz:=false`
2. `ros2 run control heading_lock_node`
3. Watch: `ros2 topic echo /heading_lock/pid_output` (and /heading_lock/error)
4. `ros2 topic pub --once /heading_lock/cmd std_msgs/msg/Float32 "{data: 0.3}"`
   — log line "heading LOCKED at ..." appears.
5. Rotate the sub NOSE-RIGHT (clockwise from above) by hand:
   - error goes NEGATIVE, pid_output NEGATIVE
   - /heading_lock/motor1 & motor3 (right pair) rise ABOVE motor2 & motor4
   Nose-left mirrors (all signs flip). If not — STOP, the sign chain is
   wrong; do not water-test.
6. Stale drill: Ctrl-C the imu launch while locked → within ~0.5 s a
   "yaw stale" WARN, ~1.0 s later "STOP + unlock" ERROR and a stop command.
7. `ros2 topic pub --once /heading_lock/cmd std_msgs/msg/Float32 "{data: 0.0}"`

## Water procedure

Order matters (heading lock MASKS thruster imbalance): run the veer
workflow first — sweep → trim → re-sweep (see forward-veer plans) — then
this, which closes the residual.

1. Preflight: normal non-skippable thruster param gate (root scripts).
2. Bring up: imu launch (no rviz), `thruster_node`, `heading_lock_node`.
   NO root field scripts at the same time (single MANUAL_CONTROL writer).
3. Submerge to test depth (ALT_HOLD holds it; heading_lock leaves heave 0).
4. `ros2 topic pub --once /heading_lock/cmd std_msgs/msg/Float32 "{data: 0.3}"`
   for a 10 m leg; stop with `{data: 0.0}`.
5. Compare against an uncorrected leg (same speed via movement_command
   surge_forward): veer-right should be nulled; log
   `ros2 topic echo /heading_lock/error` — steady-state |error| < ~3°.
6. Tune live if oscillating / sluggish:
   `ros2 param set /heading_lock_node kp 0.8`   (down = less oscillation)
   `ros2 param set /heading_lock_node kd 0.4`   (up = more damping)
   `ros2 param set /heading_lock_node ki 0.05`  (only for steady-state bias;
   watch for slow oscillation — i_limit clamps windup)
   Record final gains here:

   | date | kp | ki | kd | max_yaw_authority | notes |
   |---|---|---|---|---|---|
```

- [ ] **Step 2: Add README section**

Append to `README.md` (near other node run docs; adapt placement to the file's existing structure):

```markdown
### Heading lock — drive straight on the ZED IMU

Locks the current yaw when commanded forward and PIDs motors 1–4 (via
ArduSub's mixer) to hold it. See
`docs/superpowers/specs/2026-07-14-heading-lock-design.md`.

```bash
ros2 launch imu imu_viz.launch.py rviz:=false   # ZED + orientation_node
ros2 run mavlink_thruster_control thruster_node # arms FC — water only
ros2 run control heading_lock_node
ros2 topic pub --once /heading_lock/cmd std_msgs/msg/Float32 "{data: 0.3}"  # go
ros2 topic pub --once /heading_lock/cmd std_msgs/msg/Float32 "{data: 0.0}"  # stop
```

Debug: `ros2 topic echo /heading_lock/error` (rad; negative = drifted CW).
Live gains: `ros2 param set /heading_lock_node kp 1.0`. Rules: one command
writer at a time (no root field scripts while this runs); wrapper owns the
ZED (no detector). Bench sign-check before water:
`docs/water-tests/2026-heading-lock.md`.
```

- [ ] **Step 3: Verify docs render + commit**

```bash
git add docs/water-tests/2026-heading-lock.md README.md
git commit -m "docs(control): heading_lock run guide + bench/water procedure"
```

---

## Self-Review Notes

- **Spec coverage:** capture-on-start ✔ (T2 `start`), lock/release semantics ✔ (T2/T3 cmd paths), PID with configurable kp/ki/kd ✔ (T1/T3 params, live-tunable), motor 1–4 balancing via mixer ✔ (axes surge+yaw_rate, T3), clamps ✔ (max_yaw_authority T2, max_forward_speed T3), all 8 debug topics ✔ (T3 `_publish_debug`), normalize [-π,π] ✔ (`wrap`, T2), stale grace→stop ✔ (T2/T3), original-target recovery ✔ (T2 test), bench+water validation ✔ (T4).
- **Deviation from user's original prose:** "CW drift → increase left" was positive feedback; spec §2 documents the corrected law, tests pin it (T2 `test_cw_drift_gets_ccw_correction`).
- **Type consistency:** `HeadingLock.update(yaw|None, now_s, dt_s) -> (float, float, LockState)` used identically in T2 tests and T3 node; `PID.update(error, dt) -> float` matches T1; debug topic names in T3 node match T4 docs (`/heading_lock/...`).
- **Test env note:** tests import `control.*` and `auv_msgs` — require sourced workspace (global constraints); node smoke tests use direct callback invocation, no spin, safe on the bench.
