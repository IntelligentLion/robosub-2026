# RoboSub 2026 Audit Remediation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement all 26 findings (F1–F26) from `docs/robosub-audit-2026-07-12.html` — repo sync, sink-prevention safety fixes, control correctness, vision gating, and BT/mission integration.

**Architecture:** The stack is root-level Python field scripts (`field_common.py` engine + mission scripts) over 7 ROS 2 Humble packages. Fixes land in four layers: (1) repo sync + generated-message rebuild, (2) the shared MAVLink/thruster layer (`thruster_node.py`), (3) the field-script control engine (`field_common.py`, `depth_hold_bar02_test.py`), (4) vision/localization/BT packages. Order follows the audit's §4.16 priority: sync first, sink-preventers second, everything else after.

**Tech Stack:** Python 3.10 / ROS 2 Humble / pymavlink 2.4.49 / ZED SDK (pyzed) / TensorRT (cuda-python) / BehaviorTree.CPP (C++17) / pytest.

## Global Constraints

- This machine IS the vehicle (Jetson Orin). Never run scripts that arm the Pixhawk as part of this plan — bench verification means `--dry-run`, `python3 -c` import checks, and pytest only.
- Workspace must be sourced for anything importing ROS or workspace packages: `source /opt/ros/humble/setup.bash && source install/setup.bash`.
- After any change to `src/auv_msgs/msg/*` or C++ packages: `colcon build --symlink-install --packages-select <pkg>` then re-source. Python-only package edits take effect live (symlink-install), root scripts always live.
- `numpy<2` — do not bump numpy. TensorRT engines are built on-device with trtexec; never copy engines between machines.
- Single-serial-reader rule: only one thread may call `recv_match` on a MAVLink master. Other consumers read passively from `master.messages` (timestamp-gated) or serialize behind a shared lock.
- pymavlink 2.4.49 `add_message` crash: any NEW standalone script must carry the `_safe_add_message` monkeypatch (copy from `field_common.py:57-70`).
- Commit after every task. Commit messages: conventional style, end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Run pytest as: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/ -v`.
- Constants that need water calibration are marked `CALIBRATE IN WATER` in code — never invent values; keep the marker.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| (git merge) | converge this machine with origin/main | 1 |
| `tests/` (new) | pytest suite, first tests in the repo | 2+ |
| `depth_hold_bar02_test.py` | standalone depth test: staleness + mode watch + ack filter | 3, 5 |
| `field_common.py` | RotationIntegrator, signed turns, msg-compat assert, thread guards, DetectionMonitor.all | 2, 4, 18, 23 |
| `src/mavlink_thruster_control/.../thruster_node.py` | ack filter, re-arm policy, simulate opt-in, THR_DZ heave map, non-blocking arm, watchdog, disarm cmd, dropper service | 5–10, 20 |
| `src/mavlink_thruster_control/.../dropper_driver.py` (new) | Dropper moved into the package, passive-recv mode | 19, 20 |
| `dropper.py` (root) | thin CLI shim re-exporting dropper_driver | 19 |
| `src/vision/vision/detector.py` | tracking-state gate, IoU depth enrichment | 11, 13 |
| `src/vision/vision/bottom_camera_node.py` | BGRA fix, topic rename, loud model failure, IoU enrichment | 12, 13 |
| `src/localization/localization/vslam_node.py` | tracking-state gate, dead-code cleanup | 11, 25 |
| `src/localization/localization/localization_node.py` | Y-up yaw, single VIO source | 16 |
| `src/control/control/autonomous_controller.py` | multi-axis dispatch, supervised surfacing | 14, 15 |
| `src/mavlink_thruster_control/.../safety_monitor_node.py` | unknown-voltage fix, simulate opt-in | 17 |
| `src/robosub2026/src/bt_executor.cpp` | honest blackboard seeds, surface+disarm epilogue | 21, 24 |
| `src/robosub2026/src/manipulation_nodes.cpp` + `mission_io.cpp` | ReleaseMarker → dropper service | 20 |
| `src/robosub2026/src/nav_nodes.cpp` + `perception_nodes.cpp` | calibrated primitives, real YawSweep, honest stubs | 21, 22 |
| `src/robosub2026/launch/mission.launch.py` (new) | one integrated bring-up | 21 |
| `attic/` (new) | superseded runners out of the working dir | 25 |

**Post-pull note:** Tasks 4+ edit the ORIGIN version of `field_common.py`, `detector.py`, `thruster_node.py` (line numbers in this plan refer to post-merge files). Task 1 must complete first; nothing else compiles/imports without it.

---

### Task 1: Repo sync — merge origin/main (F1)

**Files:**
- Modify: (git state), `motor_test.py` (conflict resolution)
- Commit-then-merge: `depth_hold_bar02_test.py`, `motor_test.py`, `.gitignore`, `gate_spin_pass.py`, `check_vertical_direction.py`, `motor_trim.py`, `docs/robosub-audit-2026-07-12.html`

**Interfaces:**
- Produces: working tree at merge of `a440c08e` + `3ac71152`; `gate_task.py`, `depth_field_test.py`, origin `field_common.py`/`detector.py`/`thruster_node.py`/`MovementCommand.msg` on disk. Every later task depends on this.

- [ ] **Step 1: Snapshot state before touching anything**

```bash
cd ~/robosub2026/robosub-2026
git status --porcelain
git stash list          # expect empty
git log --oneline -1 origin/main   # expect 3ac71152
```

- [ ] **Step 2: Check the untracked motor_trim.py against origin's**

Origin adds its own `motor_trim.py`; an untracked local file with the same name blocks the merge.

```bash
git show origin/main:motor_trim.py > /tmp/motor_trim_origin.py
diff /tmp/motor_trim_origin.py motor_trim.py && echo IDENTICAL
```

If IDENTICAL: `rm motor_trim.py` (merge will restore it). If different: `mv motor_trim.py motor_trim.local.py`, merge, then hand-merge the differences into the origin file and delete the `.local` copy.

- [ ] **Step 3: Commit all local work as-is**

```bash
rm robosub-audit-2026-07-12.html    # duplicate of docs/ copy
git add -A
git commit -m "wip: local field-day edits before origin merge (depth test retune, motor test dead-man stream, gate_spin_pass, audit report)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

- [ ] **Step 4: Merge origin/main**

```bash
git merge origin/main
```

Expected: CONFLICT in `motor_test.py` (origin switched to `MOTOR_TEST_THROTTLE_PWM` because ArduSub 4.5.7 rejects PERCENT type; local was retuned for the dead-man 5 Hz stream / ORDER_BOARD p6 / 0-based p1). `depth_hold_bar02_test.py` should merge clean (origin did not touch it); if it conflicts, keep the LOCAL side (2026-07-12 THR_DZ + ZED-yaw-hold fixes) and re-apply origin hunks manually.

- [ ] **Step 5: Resolve motor_test.py keeping BOTH fixes**

Open the conflict. The merged file must have ALL of: `MOTOR_TEST_THROTTLE_PWM` throttle type (origin), `MOTOR_TEST_ORDER_BOARD` (=2) in param6, 0-based motor index in param1, and the 5 Hz re-send stream with ≤500 ms lapse handling (local). Verify after resolving:

```bash
grep -n "THROTTLE_PWM\|ORDER_BOARD\|0-based\|5 Hz\|resend\|re-send" motor_test.py
python3 -c "import ast; ast.parse(open('motor_test.py').read()); print('syntax OK')"
git add motor_test.py && git commit --no-edit
```

- [ ] **Step 6: Confirm converged tree**

```bash
git log --oneline -3          # merge commit on top of a440c08e + 3ac71152
ls gate_task.py depth_field_test.py gate_begin_assessment.py movement_test.py
git push origin main
```

Expected: all four files exist; push succeeds so both machines converge.

---

### Task 2: Rebuild messages, msg-compat guard, test infrastructure (F2)

**Files:**
- Modify: `field_common.py` (top, after the `MovementCommand` import)
- Create: `tests/__init__.py`, `tests/test_msg_compat.py`

**Interfaces:**
- Produces: `tests/` pytest layout used by every later task; guarantee that importing `field_common` fails loudly on stale `auv_msgs`.

- [ ] **Step 1: Rebuild generated messages and dependents**

```bash
cd ~/robosub2026/robosub-2026
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select auv_msgs mavlink_thruster_control bt_mission
source install/setup.bash
```

Expected: 3 packages build without errors. (`bt_mission` is C++ — it must recompile against the new msg.)

- [ ] **Step 2: Write the failing-on-stale test**

`tests/__init__.py`: empty file. `tests/test_msg_compat.py`:

```python
"""Message-compat smoke test (audit F2): catches a stale auv_msgs install
after a pull. Run under a sourced workspace."""


def test_movement_command_has_6dof_fields():
    from auv_msgs.msg import MovementCommand
    m = MovementCommand()
    for field in ('command', 'speed', 'duration', 'surge', 'strafe',
                  'heave', 'yaw_rate', 'pitch_rate', 'roll_rate'):
        assert hasattr(m, field), (
            f'MovementCommand missing "{field}" — stale auv_msgs: '
            f'colcon build --symlink-install --packages-select auv_msgs')
```

- [ ] **Step 3: Run it**

```bash
python3 -m pytest tests/test_msg_compat.py -v
```

Expected: PASS (Step 1 already rebuilt). If it fails, the rebuild didn't take — stop and fix before proceeding.

- [ ] **Step 4: Add the startup assertion to field_common.py**

Directly below `from auv_msgs.msg import MovementCommand, ObjectDetectionArray` insert:

```python
# F2 guard: symlink-install makes Python edits live but auv_msgs is GENERATED
# code in install/ — after any pull touching msg/, DepthKeeper._send would die
# on the first tick with AttributeError inside a daemon thread. Fail at import
# instead, with the fix in the message.
if not hasattr(MovementCommand(), 'pitch_rate'):
    raise ImportError(
        'auv_msgs is STALE (MovementCommand has no pitch_rate). Rebuild:\n'
        '  colcon build --symlink-install --packages-select auv_msgs '
        'mavlink_thruster_control bt_mission\n'
        '  source install/setup.bash')
```

- [ ] **Step 5: Verify imports + gate mission dry run**

```bash
python3 -c "import field_common; print('field_common OK')"
python3 gate_spin_pass.py --help
python3 gate_task.py --help
```

Expected: no ModuleNotFoundError / AttributeError. (If `gate_spin_pass.py` offers `--dry-run`, run it too.)

- [ ] **Step 6: Commit**

```bash
git add tests/ field_common.py
git commit -m "feat: msg-compat guard + first pytest suite (audit F2)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Stale-depth + flight-mode watch in the standalone depth test (F4)

**Files:**
- Modify: `depth_hold_bar02_test.py` — `drain_depth()` (~line 528), main loop (~731–830)
- Test: `tests/test_drain_depth.py`

**Interfaces:**
- Consumes: existing `send_frame(master, z, x, y, r)`, `vertical_z(effort, direction)`, `ALT_HOLD_MODE`, `NEUTRAL_Z`, `RATE_HZ`, `G`.
- Produces: `drain_depth(master, surface_hpa, rho, ptype) -> (depth_m|None, yaw_rad|None, custom_mode|None)` — 3-tuple now; both call sites updated.

- [ ] **Step 1: Write the failing test**

`tests/test_drain_depth.py`:

```python
"""drain_depth must surface HEARTBEAT custom_mode and tolerate empty buffers (F4)."""
import depth_hold_bar02_test as dhb


class FakeMsg:
    def __init__(self, mtype, src=1, **kw):
        self._mtype = mtype
        self._src = src
        self.__dict__.update(kw)

    def get_type(self):
        return self._mtype

    def get_srcSystem(self):
        return self._src


class FakeMaster:
    target_system = 1

    def __init__(self, msgs):
        self._msgs = list(msgs)

    def recv_match(self, type=None, blocking=False):
        return self._msgs.pop(0) if self._msgs else None


def test_returns_mode_from_heartbeat():
    master = FakeMaster([
        FakeMsg('SCALED_PRESSURE2', press_abs=1113.25),
        FakeMsg('HEARTBEAT', custom_mode=19, base_mode=0),
    ])
    depth, yaw, mode = dhb.drain_depth(master, 1013.25, 1000.0,
                                       'SCALED_PRESSURE2')
    assert abs(depth - (100.0 * 100.0 / (1000.0 * dhb.G))) < 1e-6
    assert mode == 19


def test_empty_buffer_returns_nones():
    assert dhb.drain_depth(FakeMaster([]), 1013.25, 1000.0,
                           'SCALED_PRESSURE2') == (None, None, None)


def test_foreign_heartbeat_ignored():
    master = FakeMaster([FakeMsg('HEARTBEAT', src=255, custom_mode=19,
                                 base_mode=0)])
    assert dhb.drain_depth(master, 1013.25, 1000.0,
                           'SCALED_PRESSURE2')[2] is None
```

- [ ] **Step 2: Run to verify failure**

`python3 -m pytest tests/test_drain_depth.py -v` — Expected: FAIL (drain_depth returns 2-tuple, no HEARTBEAT in filter).

- [ ] **Step 3: Rewrite drain_depth**

```python
def drain_depth(master, surface_hpa, rho, ptype):
    """Pull all buffered pressure + attitude + heartbeat msgs. Returns
    (depth_m, yaw_rad, custom_mode), each None if no fresh message of that
    kind was buffered. custom_mode lets the run loop notice ArduSub silently
    forcing MANUAL (mode 19) when the Bar02 drops off I2C (F4)."""
    depth = None
    yaw = None
    mode = None
    while True:
        msg = master.recv_match(type=[ptype, 'ATTITUDE', 'HEARTBEAT'],
                                blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == 'ATTITUDE':
            yaw = msg.yaw
        elif t == 'HEARTBEAT':
            if msg.get_srcSystem() == master.target_system:
                mode = msg.custom_mode
        else:
            depth = (msg.press_abs - surface_hpa) * 100.0 / (rho * G)
    return depth, yaw, mode
```

- [ ] **Step 4: Run tests — expect PASS**

`python3 -m pytest tests/test_drain_depth.py -v`

- [ ] **Step 5: Add staleness + mode-abort to the main loop**

Near the loop setup (after `pix_yaw0 = zed_yaw0 = None` ~line 729) add:

```python
    DEPTH_STALE_ABORT_S = 1.5       # Bar02 gap before declaring depth lost
    TIMED_SURFACE_S = 20.0          # blind ascent length when depth is lost
    last_depth_t = time.monotonic()
    depth_lost = False
```

Replace the loop's drain/update block (`d, py = drain_depth(...)` etc.) with:

```python
            d, py, mode = drain_depth(master, surface_hpa, rho, ptype)
            if d is not None:
                depth = d
                last_depth_t = time.monotonic()
            if py is not None:
                pix_yaw = py
                if pix_yaw0 is None:
                    pix_yaw0 = py

            # F4: a frozen reading defeats every depth check below. If the
            # Bar02 stops streaming, or ArduSub silently leaves ALT_HOLD
            # (mode 19 = MANUAL forced by depth-sensor loss), abort NOW —
            # in MANUAL our "descend" frames are raw down-thrust.
            if not aborted:
                if time.monotonic() - last_depth_t > DEPTH_STALE_ABORT_S:
                    aborted = True
                    depth_lost = True
                    print(f'ABORT: no Bar02 pressure for '
                          f'{DEPTH_STALE_ABORT_S}s — depth LOST. Timed '
                          f'surface ({TIMED_SURFACE_S:.0f}s) + disarm.')
                elif mode is not None and mode != ALT_HOLD_MODE:
                    aborted = True
                    print(f'ABORT: autopilot left ALT_HOLD '
                          f'(custom_mode={mode}) — Bar02 gone? Surfacing.')
```

Replace the abort branch with:

```python
            if aborted:
                if depth_lost:
                    # Frozen reading — closed loop impossible. Blind ascent,
                    # then break to the finally (neutral + disarm). Sub is
                    # positively buoyant; up-thrust just speeds it along.
                    deadline = time.time() + TIMED_SURFACE_S
                    while time.time() < deadline:
                        send_frame(master,
                                   vertical_z(args.speed * args.min_speed, +1))
                        time.sleep(period)
                    break
                if time.monotonic() - last_depth_t > DEPTH_STALE_ABORT_S:
                    depth_lost = True     # went stale mid-abort — go blind
                    continue
                if depth > args.deadband:
                    send_frame(master,
                               vertical_z(args.speed * args.min_speed, +1),
                               xc, yc, rc)
                    time.sleep(period)
                    continue
                break
```

- [ ] **Step 6: Same hole in the post-hold surfacing loop**

In the `print('Surfacing…')` loop, after `d, _ = drain_depth(...)` (update to 3-tuple: `d, _, _ = ...`) and the `if d is not None: depth = d` line, add `last_depth_t` maintenance and a stale check:

```python
            d, _, _ = drain_depth(master, surface_hpa, rho, ptype)
            if d is not None:
                depth = d
                last_depth_t = time.monotonic()
            if time.monotonic() - last_depth_t > DEPTH_STALE_ABORT_S:
                print('Surfacing: Bar02 went stale — finishing ascent blind '
                      '(15s), then disarm.')
                deadline = time.time() + 15.0
                while time.time() < deadline:
                    send_frame(master,
                               vertical_z(args.speed * args.max_speed, +1))
                    time.sleep(period)
                break
```

- [ ] **Step 7: Verify + commit**

```bash
python3 -m pytest tests/ -v
python3 -c "import depth_hold_bar02_test; print('import OK')"
git add depth_hold_bar02_test.py tests/test_drain_depth.py
git commit -m "fix: depth test aborts on stale Bar02 + unexpected mode exit (audit F4)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: Signed rotation integration — RotationIntegrator (F7)

**Files:**
- Modify: `field_common.py` (helpers + `RampedDriver._turn` + `DepthKeeper.turn` + `DepthKeeper.rotate`)
- Modify: `gate_spin_pass.py`, `gate_task.py` (any private copies of `abs(wrap(...))` / `abs(rates[...])` integration)
- Test: `tests/test_rotation_integrator.py`

**Interfaces:**
- Produces: `field_common.RotationIntegrator(direction, now=None)` with `.add(delta, now=None) -> net_rad`, `.done(target_rad)`, `.stalled(now=None)`, and module constant `ZED_CW_HEADING_SIGN`. Gyro sign convention: ArduSub ATTITUDE body rates are positive for CW yaw / right-roll / nose-up pitch, matching the `+1` command direction of `rotate_cw` / `roll_right` / `pitch_up`.

- [ ] **Step 1: Write the failing tests**

`tests/test_rotation_integrator.py`:

```python
import math
from field_common import RotationIntegrator


def test_rocking_never_completes():
    """±deltas that sum to zero (ALT_HOLD fighting a roll) must not advance —
    the old abs() integration 'verified' a roll that never happened."""
    ri = RotationIntegrator(direction=+1, now=0.0)
    t = 0.0
    for _ in range(200):
        t += 0.1
        ri.add(+0.05, now=t)
        t += 0.1
        ri.add(-0.05, now=t)
    assert not ri.done(math.radians(90))
    assert ri.stalled(now=t)


def test_steady_rotation_completes():
    ri = RotationIntegrator(direction=+1, now=0.0)
    t = 0.0
    for _ in range(100):
        t += 0.1
        ri.add(math.radians(2.0), now=t)   # 20 deg/s
        if ri.done(math.radians(90)):
            break
    assert ri.done(math.radians(90))
    assert not ri.stalled(now=t)


def test_wrong_direction_reads_negative_and_stalls():
    ri = RotationIntegrator(direction=+1, now=0.0)
    t = 0.0
    for _ in range(50):
        t += 0.1
        ri.add(math.radians(-2.0), now=t)
    assert ri.net < 0
    assert ri.stalled(now=t)
    assert not ri.done(math.radians(90))


def test_ccw_direction_normalises():
    ri = RotationIntegrator(direction=-1, now=0.0)
    ri.add(math.radians(-45), now=0.1)
    ri.add(math.radians(-45), now=0.2)
    assert ri.done(math.radians(90))
```

- [ ] **Step 2: Run — expect ImportError**

`python3 -m pytest tests/test_rotation_integrator.py -v` → FAIL: `cannot import name 'RotationIntegrator'`.

- [ ] **Step 3: Implement in field_common.py**

Below `clamp()` add:

```python
# Sign of a ZED heading delta (heading_about_axis) during a CW (rotate_cw)
# turn. The DepthKeeper yaw-hold law implies s_cw = -yaw_hold_sign (the hold
# steers OPPOSITE the error). CALIBRATE IN WATER: run a single turn_right(30)
# — if it times out at ~0° progress while the sub visibly turns, flip this.
ZED_CW_HEADING_SIGN = -1.0


class RotationIntegrator:
    """Net signed rotation toward a commanded direction (audit F7).

    The old integrators summed abs(delta), so heading jitter, current-induced
    rocking, and ALT_HOLD fighting a style roll all counted as progress —
    a stalled roll could "verify" at 90° while the sub just rocked at ±25°.
    Here: net += direction * delta. Rocking sums to ~0; rotating the wrong
    way goes negative; either trips stalled() instead of completing.
    """

    MIN_STEP = math.radians(1.0)   # progress smaller than this isn't progress
    STALL_S = 2.0                  # no net advance for this long → stalled

    def __init__(self, direction, now=None):
        self.direction = 1.0 if direction >= 0 else -1.0
        self.net = 0.0
        self._best = 0.0
        self._best_t = time.monotonic() if now is None else now

    def add(self, delta, now=None):
        """Integrate one signed increment (rad, sensor sign convention already
        mapped so `direction * delta > 0` means commanded-direction motion)."""
        now = time.monotonic() if now is None else now
        self.net += self.direction * delta
        if self.net > self._best + self.MIN_STEP:
            self._best = self.net
            self._best_t = now
        return self.net

    def done(self, target_rad):
        return self.net >= target_rad

    def stalled(self, now=None):
        now = time.monotonic() if now is None else now
        return now - self._best_t > self.STALL_S
```

Note: `add()` takes the raw signed delta and multiplies by `self.direction` internally — callers pass the sensor delta with the sensor→command sign mapping applied (see Steps 5–7), and pass `direction=±1` for the commanded turn direction.

- [ ] **Step 4: Run — expect PASS**

`python3 -m pytest tests/test_rotation_integrator.py -v`

- [ ] **Step 5: Rewrite RampedDriver._turn's integration**

In `_turn` (post-merge ~line 309): `dir_cw = +1.0 if command == 'rotate_cw' else -1.0`. Replace `turned = 0.0` with `integ = RotationIntegrator(dir_cw)` and the ZED branch body:

```python
                if closed_loop:
                    h = mon.heading()
                    if h is None:
                        remain = max(0.0, 1.0 - max(integ.net, 0.0) / target)
                        self.get_logger().warn(
                            f'turn: heading lost at '
                            f'{math.degrees(integ.net):.0f}° — timed '
                            f'remainder {remain * est:.1f}s')
                        time.sleep(remain * est)
                        closed_loop = False
                        break
                    integ.add(ZED_CW_HEADING_SIGN * wrap(h - h_prev))
                    h_prev = h
                    if integ.done(target):
                        break
                    if integ.stalled():
                        self.get_logger().error(
                            f'turn: STALLED at {math.degrees(integ.net):.0f}° '
                            f'of {degrees:.0f}° (no net progress 2s) — '
                            f'declaring failure, not completing.')
                        self.stop_move(ramp=0.3)
                        return False
```

(`integ.add` receives the CW-mapped delta; `direction` inside the integrator flips it for CCW commands.) Update the final log line to use `integ.net`.

- [ ] **Step 6: Rewrite DepthKeeper.turn's integration**

`direction` param is already ±1 (+1 = CW). Replace `turned = 0.0` with `integ = RotationIntegrator(direction)`; ZED branch: `integ.add(ZED_CW_HEADING_SIGN * wrap(h - h_prev))`; gyro branch: `integ.add(rates[2] * dt)` (ATTITUDE yawspeed +CW — direction flip happens inside). Completion: `if closed_loop and integ.done(target): break`. Add after each `add`:

```python
                if closed_loop and integ.stalled():
                    print(f'turn: STALLED at {math.degrees(integ.net):.0f}° '
                          f'of {degrees:.0f}° — failing (no silent complete).')
                    self.clear_move(ramp=0.3)
                    return False
```

All `math.degrees(turned)` prints → `math.degrees(integ.net)`.

- [ ] **Step 7: Rewrite DepthKeeper.rotate's integration**

Same pattern: `integ = RotationIntegrator(direction)`; `rotated += abs(rates[idx]) * dt` → `integ.add(rates[idx] * dt)` (rollspeed +right-roll, pitchspeed +nose-up — both match their `+1` command direction); `if integ.done(target): ok = True; break`; stalled → print + `clear_move` + `return False`. Style scoring depends on this: a stalled roll must report failure so the mission can retry or skip, not report a scored roll that never happened.

- [ ] **Step 8: Fix private copies in gate scripts**

```bash
grep -n "abs(wrap(\|abs(rates" gate_spin_pass.py gate_task.py gate_begin_assessment.py
```

For every hit (audit: `gate_spin_pass.py:301`): replace with `RotationIntegrator` exactly as in Steps 5–7 (import it from `field_common`). If a script re-implements the whole turn loop, prefer replacing the loop with a call to `DepthKeeper.turn` / `.rotate`.

- [ ] **Step 9: Verify + commit**

```bash
python3 -m pytest tests/ -v
python3 -c "import field_common, gate_task, gate_spin_pass; print('OK')"
git add field_common.py gate_spin_pass.py gate_task.py gate_begin_assessment.py tests/test_rotation_integrator.py
git commit -m "fix: signed rotation integration with stall detection (audit F7)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

**Water-day note (put in run sheet, not code):** first in-water turn verifies `ZED_CW_HEADING_SIGN`; first style roll verifies gyro signs. A wrong sign now fails loudly (stall) instead of faking success.

---

### Task 5: COMMAND_ACK filtering (F16)

**Files:**
- Modify: `src/mavlink_thruster_control/mavlink_thruster_control/thruster_node.py` (`_arm_vehicle`, `_connect_mavlink`), `depth_hold_bar02_test.py` (`arm()`, `set_alt_hold()`)
- Test: `tests/test_wait_ack.py`

**Interfaces:**
- Produces: `ThrusterController._wait_ack(command_id, timeout=3.0) -> ack|None` — passive (`master.messages`) when `master._external_recv_reader` is set, active recv otherwise. Task 9 reuses it from the worker thread; Task 20's Dropper uses the same passive pattern.

- [ ] **Step 1: Write the failing test**

`tests/test_wait_ack.py`:

```python
"""_wait_ack must ignore acks for unrelated commands (audit F16)."""
import types
from mavlink_thruster_control.thruster_node import ThrusterController


def make_ack(command, result=0, ts=None):
    a = types.SimpleNamespace(command=command, result=result)
    if ts is not None:
        a._timestamp = ts
    return a


class FakeMaster:
    def __init__(self, acks):
        self._acks = list(acks)
        self.messages = {}

    def recv_match(self, type=None, blocking=False, timeout=None):
        return self._acks.pop(0) if self._acks else None


def wait_ack(master, command_id, timeout=0.5):
    # bind the real method to a stub — no ROS init needed
    stub = types.SimpleNamespace(master=master)
    return ThrusterController._wait_ack(stub, command_id, timeout=timeout)


def test_unrelated_ack_skipped():
    master = FakeMaster([make_ack(511), make_ack(400, result=0)])  # 400 = ARM_DISARM
    ack = wait_ack(master, 400)
    assert ack is not None and ack.command == 400


def test_no_matching_ack_returns_none():
    assert wait_ack(FakeMaster([make_ack(511)]), 400) is None


def test_passive_path_reads_master_messages():
    import time
    master = FakeMaster([])
    master._external_recv_reader = True
    master.messages = {'COMMAND_ACK': make_ack(400, ts=time.time() + 1.0)}
    ack = wait_ack(master, 400)
    assert ack is not None and ack.command == 400
```

- [ ] **Step 2: Run — expect AttributeError (no _wait_ack)**

`python3 -m pytest tests/test_wait_ack.py -v`

- [ ] **Step 3: Implement _wait_ack in ThrusterController**

Add below `_connect_mavlink`:

```python
    def _wait_ack(self, command_id, timeout=3.0):
        """COMMAND_ACK for command_id only — a stream-request or param ack
        must not satisfy an arm/mode check (F16). Passive master.messages
        read when an external thread owns the serial recv path."""
        t0 = _time.time()
        if getattr(self.master, '_external_recv_reader', False):
            while _time.time() - t0 < timeout:
                ack = self.master.messages.get('COMMAND_ACK')
                if (ack is not None
                        and getattr(ack, '_timestamp', 0.0) >= t0
                        and ack.command == command_id):
                    return ack
                _time.sleep(0.05)
            return None
        while _time.time() - t0 < timeout:
            ack = self.master.recv_match(
                type='COMMAND_ACK', blocking=True,
                timeout=max(0.1, timeout - (_time.time() - t0)))
            if ack is None:
                return None
            if ack.command == command_id:
                return ack
            # ack for something else — noise, keep waiting
        return None
```

- [ ] **Step 4: Use it at both call sites**

In `_arm_vehicle`, replace `ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)` with:

```python
            ack = self._wait_ack(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
```

In `_connect_mavlink`, replace the set_mode ack read with:

```python
                ack = self._wait_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
```

(ArduPilot acks `set_mode_send` as `MAV_CMD_DO_SET_MODE` = 176; a missing ack stays non-fatal — the heartbeat verify in `_check_armed_status` is the real check.)

- [ ] **Step 5: Same fix in depth_hold_bar02_test.py**

In `arm()` (~line 477) replace the recv with a filtered loop:

```python
    deadline = time.time() + 3.0
    ack = None
    while time.time() < deadline:
        m = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=1)
        if m is None:
            continue
        if m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            ack = m
            break
```

In `set_alt_hold()` (~line 454) same pattern with `MAV_CMD_DO_SET_MODE` (keep the existing heartbeat verify as the authority).

- [ ] **Step 6: Run tests + commit**

```bash
python3 -m pytest tests/ -v
git add src/mavlink_thruster_control tests/test_wait_ack.py depth_hold_bar02_test.py
git commit -m "fix: filter COMMAND_ACK by command id (audit F16)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Re-arm policy — respect operator and failsafe disarms (F5)

**Files:**
- Modify: `thruster_node.py` (`__init__`, `_disarm_vehicle`, `_passive_heartbeat`, `_check_armed_status`)

**Interfaces:**
- Produces: `self._intentional_disarm` flag (Task 10's disarm command and Task 24's epilogue rely on it), `self._failsafe_latched`, `MAX_REARM_ATTEMPTS = 3`.

- [ ] **Step 1: Add state + constant**

In the class constants: `MAX_REARM_ATTEMPTS = 3    # lifetime cap — a vehicle that keeps disarming is telling you something`. In `__init__` near `_reconnect_attempts`:

```python
        self._intentional_disarm = False   # we disarmed on purpose — never re-arm
        self._failsafe_latched = False     # ArduSub failsafe disarm — never re-arm
        self._rearm_count = 0
```

- [ ] **Step 2: Mark intentional disarms**

First line of `_disarm_vehicle()` body: `self._intentional_disarm = True`.

- [ ] **Step 3: Latch failsafe STATUSTEXTs**

In `_passive_heartbeat`, inside the `if ts > getattr(...)` block after the error log, add:

```python
                low = st.text.lower()
                if any(k in low for k in ('failsafe', 'leak', 'batt')):
                    self._failsafe_latched = True
```

In the active drain branch of `_check_armed_status`, inside `if msg.severity <= 4:` after the error log, add the same three lines (using `msg.text`).

- [ ] **Step 4: Gate the re-arm**

Replace the `if not armed:` block in `_check_armed_status` with:

```python
            if not armed:
                # F5: every disarm is NOT an anomaly. Operator disarm (QGC/
                # tether), failsafe disarm (battery/leak), and our own disarm
                # must stay disarmed — blind re-arm can spin props next to a
                # diver.
                if self._intentional_disarm:
                    return
                if self._failsafe_latched:
                    self.get_logger().error(
                        'Disarmed by ArduSub FAILSAFE — NOT re-arming '
                        '(latched). Investigate before the next run.')
                    return
                if self._rearm_count >= self.MAX_REARM_ATTEMPTS:
                    self.get_logger().error(
                        f'Vehicle disarmed again after '
                        f'{self._rearm_count} re-arms — assuming operator '
                        f'intent, NOT re-arming. Restart the node to reset.')
                    return
                self._rearm_count += 1
                self.get_logger().warn(
                    f'Vehicle DISARMED unexpectedly – re-arming '
                    f'({self._rearm_count}/{self.MAX_REARM_ATTEMPTS}) …')
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    self.flight_mode_id)
                _time.sleep(0.3)
                if self._arm_vehicle():
                    self.get_logger().info('Re-armed successfully')
                else:
                    self.get_logger().error(
                        'Re-arm FAILED – vehicle may not respond')
```

(Cap is per node lifetime, deliberately NOT reset on success: three unexpected disarms in one run is a hardware conversation, not a retry loop.)

- [ ] **Step 5: Verify + commit**

```bash
python3 -m pytest tests/ -v          # existing suite still green
python3 -c "
from unittest.mock import MagicMock, patch
import rclpy
rclpy.init()
with patch.object(__import__('mavlink_thruster_control.thruster_node', fromlist=['ThrusterController']).ThrusterController, '_connect_mavlink'):
    from mavlink_thruster_control.thruster_node import ThrusterController
    n = ThrusterController()
    assert n._rearm_count == 0 and not n._intentional_disarm
    n.destroy_node(); rclpy.shutdown(); print('state OK')
"
git add src/mavlink_thruster_control
git commit -m "fix: cap re-arms, respect operator/failsafe disarms (audit F5)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: Simulation mode is opt-in only + loud link-lost escalation (F6)

**Files:**
- Modify: `thruster_node.py` (`__init__`, `_connect_mavlink`, `_reconnect_mavlink`, `_heartbeat_loop`)

**Interfaces:**
- Produces: `/safety/thruster_link_lost` (`std_msgs/Bool`, republished 1 Hz). Task 21 maps it to `critical_failure` in the BT. Constructor now RAISES when hardware is absent and `simulate` is false.

- [ ] **Step 1: Add the link-lost publisher**

Import `from std_msgs.msg import Bool` at top. In `__init__` (before the timers):

```python
        # F6: when the link dies mid-run, the rest of the stack must find out
        # — not keep "succeeding" against a limp vehicle.
        self._link_lost = False
        self.link_lost_pub = self.create_publisher(
            Bool, '/safety/thruster_link_lost', 1)
```

- [ ] **Step 2: Refuse to silently simulate**

Replace the `if not HAS_MAVLINK:` block in `__init__` with:

```python
        if not HAS_MAVLINK and not self.simulate:
            raise RuntimeError(
                'pymavlink not installed and simulate:=false — install '
                'pymavlink, or pass simulate:=true for desk testing.')
```

Replace the tail of `_connect_mavlink` (`self.get_logger().error('No MAVLink device found – falling back to SIMULATION mode'); self.simulate = True`) with:

```python
        raise RuntimeError(
            'No MAVLink device answered on any /dev/ttyACM*//dev/ttyUSB* '
            'port. Refusing to run without hardware (F6) — check the USB '
            'cable/port, or pass simulate:=true for desk testing.')
```

- [ ] **Step 3: Reconnect exhaustion escalates instead of simulating**

In `_reconnect_mavlink`, replace the `> MAX_RECONNECT_ATTEMPTS` block body with:

```python
        if self._reconnect_attempts > self.MAX_RECONNECT_ATTEMPTS:
            if not self._link_lost:
                self._link_lost = True
                self.get_logger().fatal(
                    f'THRUSTER LINK LOST: {self.MAX_RECONNECT_ATTEMPTS} '
                    f'reconnects failed. Publishing '
                    f'/safety/thruster_link_lost — mission layer must abort. '
                    f'Will keep retrying in the background.')
            self.link_lost_pub.publish(Bool(data=True))
            self._reconnect_attempts = 0     # keep trying, but stay loud
            return
```

Initial `_connect_mavlink()` raising inside `_reconnect_mavlink` would kill the timer callback — wrap the call:

```python
            try:
                self._connect_mavlink()
            except RuntimeError as exc:
                self.get_logger().error(f'Reconnect failed: {exc}')
            if self.connected:
                self._reconnect_attempts = 0
                if self._link_lost:
                    self._link_lost = False
                    self.get_logger().warn('Thruster link RESTORED.')
                self.link_lost_pub.publish(Bool(data=False))
```

- [ ] **Step 4: Republish at 1 Hz**

At the top of `_heartbeat_loop` (before the simulate check):

```python
        self.link_lost_pub.publish(Bool(data=self._link_lost))
```

- [ ] **Step 5: Verify + commit**

```bash
python3 -m pytest tests/ -v
# constructor must now raise without hardware and without simulate:
python3 -c "
import rclpy; rclpy.init()
from mavlink_thruster_control.thruster_node import ThrusterController
import mavlink_thruster_control.thruster_node as tn
tn.HAS_MAVLINK = False
try:
    ThrusterController()
    print('FAIL: should have raised')
except RuntimeError as e:
    print('raises OK:', e)
rclpy.shutdown()
"
git add src/mavlink_thruster_control
git commit -m "fix: simulate is opt-in; link loss escalates via /safety/thruster_link_lost (audit F6)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

Note for field scripts: `field_common.session()` constructs `ThrusterController()` — with this change a wrong port aborts the script at the dock with a clear error instead of five silent minutes. That is the desired behavior.

---

### Task 8: THR_DZ-aware heave mapping in set_axes (F10)

**Files:**
- Modify: `thruster_node.py` (module-level helper + `set_axes` + `__init__`/`_connect_mavlink`)
- Test: `tests/test_heave_to_z.py`

**Interfaces:**
- Produces: module function `heave_to_z(heave, thr_dz=100) -> int` (also usable by future scripts); `self.thr_dz` read from the vehicle at connect.

- [ ] **Step 1: Write the failing test**

`tests/test_heave_to_z.py`:

```python
"""ALT_HOLD ignores z within ±THR_DZ of 500 — small closed-loop heave efforts
must be offset past the deadzone (audit F10)."""
from mavlink_thruster_control.thruster_node import heave_to_z


def test_zero_is_neutral():
    assert heave_to_z(0.0) == 500


def test_small_down_effort_exceeds_deadzone():
    z = heave_to_z(0.15, thr_dz=100)
    assert z < 400            # outside 500-100; old mapping gave 425 (no-op)
    assert z == 500 - (100 + round(0.15 * 400))


def test_full_scale():
    assert heave_to_z(1.0) == 0
    assert heave_to_z(-1.0) == 1000


def test_up_symmetry():
    assert heave_to_z(-0.15, thr_dz=100) == 500 + (100 + round(0.15 * 400))
```

- [ ] **Step 2: Run — expect ImportError**

`python3 -m pytest tests/test_heave_to_z.py -v`

- [ ] **Step 3: Implement**

Module level in `thruster_node.py` (below the mavutil import block):

```python
def heave_to_z(heave, thr_dz=100):
    """Map signed heave [-1,1] (+down) to a MANUAL_CONTROL z (0..1000).

    ALT_HOLD treats z within ±THR_DZ (default 100) of 500 as "hold depth", so
    a linear map makes efforts under 0.2 a silent no-op — DepthKeeper's
    min_speed=0.15 did nothing in ALT_HOLD (F10). Offset every non-zero
    effort past the deadzone, exactly like depth_hold_bar02_test.vertical_z.
    """
    if not heave:
        return 500
    h = max(-1.0, min(1.0, float(heave)))
    mag = thr_dz + abs(h) * (500 - thr_dz)
    z = 500 - mag if h > 0 else 500 + mag
    return max(0, min(1000, int(round(z))))
```

In `set_axes`, replace `self.current_z = max(0, min(1000, round(500 - h * 500)))` with:

```python
        self.current_z = heave_to_z(h, self.thr_dz)
```

- [ ] **Step 4: Read THR_DZ from the vehicle at connect**

In `__init__` (before `_connect_mavlink()`): `self.thr_dz = 100   # ArduSub THR_DZ default; refreshed at connect`. In `_connect_mavlink` after the arm call add:

```python
                self._read_thr_dz()
```

and the method:

```python
    def _read_thr_dz(self):
        """Refresh THR_DZ so heave_to_z matches the vehicle's actual deadzone
        instead of a hardcoded 100 (F10). Best-effort — runs at connect, before
        any external recv owner exists."""
        try:
            self.master.mav.param_request_read_send(
                self.master.target_system, self.master.target_component,
                b'THR_DZ', -1)
            deadline = _time.time() + 2.0
            while _time.time() < deadline:
                msg = self.master.recv_match(type='PARAM_VALUE',
                                             blocking=True, timeout=1)
                if msg is None:
                    continue
                if str(msg.param_id).strip('\x00') == 'THR_DZ':
                    self.thr_dz = int(msg.param_value)
                    self.get_logger().info(f'THR_DZ = {self.thr_dz}')
                    return
            self.get_logger().warn('THR_DZ read timed out — using 100')
        except Exception as exc:
            self.get_logger().warn(f'THR_DZ read failed ({exc}) — using 100')
```

- [ ] **Step 5: Run tests — expect PASS — and commit**

```bash
python3 -m pytest tests/ -v
git add src/mavlink_thruster_control tests/test_heave_to_z.py
git commit -m "fix: offset set_axes heave past ALT_HOLD deadzone, read THR_DZ live (audit F10)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

Behavior note (expected, not a bug): DepthKeeper's inside-deadband micro-efforts now have real authority (minimum ~THR_DZ/500 = 0.2 of scale once non-zero) in BOTH modes — the controller's effective gain no longer changes 0→full when ArduSub flips ALT_HOLD→MANUAL. Watch the first water test for depth buzz; if present, raise DepthKeeper `deadband`, don't touch this mapping.

---

### Task 9: Non-blocking arm/reconnect — worker thread (F15)

**Files:**
- Modify: `thruster_node.py` (`__init__`, `_check_armed_status`, `_control_loop`)

**Interfaces:**
- Consumes: `_wait_ack` (Task 5), re-arm policy fields (Task 6).
- Produces: `self._recovery_thread`; timer callbacks never block >100 ms. Task 20 adds `self._mav_send_lock` — created here since the worker also sends.

- [ ] **Step 1: Add worker state + send lock**

Import `threading` at top. In `__init__`:

```python
        # F15: arming/reconnect block up to seconds on serial reads — never
        # from a timer callback (all callbacks share one MutuallyExclusive
        # group; manual_control streaming and the GCS heartbeat would stall).
        self._recovery_thread = None
        self._mav_send_lock = threading.Lock()   # 2 sender threads (loop+worker)
```

- [ ] **Step 2: Move the re-arm to the worker**

In `_check_armed_status`, replace the tail of the `if not armed:` block (from `self._rearm_count += 1` onward, keeping the Task 6 policy gates above it) with:

```python
                if (self._recovery_thread is not None
                        and self._recovery_thread.is_alive()):
                    return                       # recovery already in flight
                self._rearm_count += 1
                self.get_logger().warn(
                    f'Vehicle DISARMED unexpectedly – re-arming on worker '
                    f'({self._rearm_count}/{self.MAX_REARM_ATTEMPTS}) …')
                self._recovery_thread = threading.Thread(
                    target=self._rearm_worker, daemon=True)
                self._recovery_thread.start()
```

and add:

```python
    def _rearm_worker(self):
        try:
            with self._mav_send_lock:
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    self.flight_mode_id)
            _time.sleep(0.3)
            if self._arm_vehicle():
                self.get_logger().info('Re-armed successfully')
            else:
                self.get_logger().error('Re-arm FAILED – vehicle may not respond')
        except Exception as exc:
            self.get_logger().error(f'Re-arm worker error: {exc}')
```

- [ ] **Step 3: Move reconnect to the worker**

In `_control_loop`'s error path, replace `self._reconnect_mavlink()` with:

```python
                if (self._recovery_thread is None
                        or not self._recovery_thread.is_alive()):
                    self._recovery_thread = threading.Thread(
                        target=self._reconnect_mavlink, daemon=True)
                    self._recovery_thread.start()
```

`self.connected` goes false during reconnect, so the 10 Hz loop returns early (no frames — ArduSub's pilot-input handling coasts; that beats a frozen executor). The `_reconnecting` re-entry guard already exists.

- [ ] **Step 4: Serialize sends**

Wrap the `manual_control_send` in `_control_loop` and the sends in `_heartbeat_loop`, `_disarm_vehicle`, `_arm_vehicle`, and `set_flight_mode` with `with self._mav_send_lock:` (one-line indent change each; keep the recv paths untouched).

- [ ] **Step 5: Verify + commit**

```bash
python3 -m pytest tests/ -v
python3 -c "import mavlink_thruster_control.thruster_node; print('import OK')"
git add src/mavlink_thruster_control
git commit -m "fix: arm/reconnect on worker thread, serialized MAVLink sends (audit F15)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 10: Watchdog policy + software disarm command (F22 + F11 thruster half)

**Files:**
- Modify: `thruster_node.py` (constants, `_movement_cb` dispatch, `_control_loop`)

**Interfaces:**
- Produces: `MovementCommand.command == 'disarm'` → neutral + disarm + no auto re-arm. Task 24's BT epilogue and any mission script can now release the vehicle. Watchdog: 5 s → stop; 60 s → open-loop surface 20 s → disarm.

- [ ] **Step 1: Constants**

```python
    DEFAULT_WATCHDOG_S = 5.0        # stop if no command for this long (was 30:
                                    # 10 Hz streamers make anything longer 10+ m
                                    # of blind travel, F22)
    WATCHDOG_DISARM_S = 60.0        # abandoned this long → surface + disarm
    WATCHDOG_SURFACE_S = 20.0       # open-loop ascent before the disarm
```

In `__init__`: `self._watchdog_surface_started = None` and `self._watchdog_disarmed = False`.

- [ ] **Step 2: Disarm verb**

In `_movement_cb`'s dispatch dict add:

```python
                'disarm':         self._operator_disarm,
```

and the method:

```python
    def _operator_disarm(self):
        """Mission-commanded release (F11): neutral, disarm, and never
        auto-re-arm — until now NOTHING in the ROS graph could disarm; only
        process teardown did."""
        self.stop()
        self._intentional_disarm = True
        self._disarm_vehicle()
```

- [ ] **Step 3: Two-stage watchdog**

Replace the watchdog block in `_control_loop` with:

```python
        if (self._last_cmd_time is not None
                and self.watchdog_timeout > 0):
            elapsed = (now - self._last_cmd_time).nanoseconds / 1e9
            if elapsed > self.watchdog_timeout and not self._watchdog_triggered:
                self.get_logger().warn(
                    f'Watchdog: no command for {elapsed:.0f}s – stopping')
                self.stop()
                self._watchdog_triggered = True
            if elapsed > self.WATCHDOG_DISARM_S and not self._watchdog_disarmed:
                # F22: stop-without-disarm left the sub armed at depth
                # indefinitely. Truly abandoned → bring it up and release it.
                if self._watchdog_surface_started is None:
                    self.get_logger().error(
                        f'Watchdog: no command for {elapsed:.0f}s — '
                        f'open-loop surface ({self.WATCHDOG_SURFACE_S:.0f}s) '
                        f'then disarm.')
                    self._watchdog_surface_started = now
                    self.emerge(0.3)
                elif ((now - self._watchdog_surface_started).nanoseconds / 1e9
                        > self.WATCHDOG_SURFACE_S):
                    self.stop()
                    self._operator_disarm()
                    self._watchdog_disarmed = True
```

In `_movement_cb`, next to `self._watchdog_triggered = False` add `self._watchdog_surface_started = None` (a fresh command cancels the abandonment sequence; `_watchdog_disarmed` stays latched — re-arming after a watchdog disarm is a human decision).

- [ ] **Step 4: Verify + commit**

```bash
python3 -m pytest tests/ -v
python3 -c "import mavlink_thruster_control.thruster_node as t; \
  assert t.ThrusterController.DEFAULT_WATCHDOG_S == 5.0; print('OK')"
git add src/mavlink_thruster_control
git commit -m "feat: disarm movement command + 5s/60s two-stage watchdog (audit F11, F22)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

Caution for existing scripts: any tool that idles >5 s without streaming (e.g. waiting at an `input()` prompt mid-session) now gets a watchdog stop — that is neutral thrust + ALT_HOLD, which is safe and correct; resume by streaming again.

---

### Task 11: Gate ZED pose/depth on tracking state (F8)

**Files:**
- Modify: `src/vision/vision/detector.py` (`run_detector`, ~line 995 post-merge), `src/localization/localization/vslam_node.py` (~line 189)

**Interfaces:**
- Consumes: `sl.POSITIONAL_TRACKING_STATE.OK`; `zed.get_position()` RETURNS the state (both files currently discard it).
- Produces: `depth/sub_depth` + `vslam/odometry` simply stop when tracking ≠ OK — every downstream staleness monitor (`HeadingMonitor.heading()`, `CoordMonitor.have_fix()`, `DepthMonitor.depth()`) then does the right thing automatically.

- [ ] **Step 1: detector.py**

Replace the pose-publish block in `run_detector`:

```python
                if positional_tracking_enabled:
                    zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
                    translation = zed_pose.get_translation(sl.Translation()).get()
                    sub_depth_m = -float(translation[1])
                    node.publish_sub_depth(sub_depth_m)
                    orientation = zed_pose.get_orientation(sl.Orientation()).get()
                    node.publish_odometry(translation, orientation)
```

with:

```python
                if positional_tracking_enabled:
                    track_state = zed.get_position(
                        zed_pose, sl.REFERENCE_FRAME.WORLD)
                    if track_state == sl.POSITIONAL_TRACKING_STATE.OK:
                        if _tracking_lost_since is not None:
                            print(f'[Vision] ZED tracking recovered after '
                                  f'{time.time() - _tracking_lost_since:.1f}s')
                            _tracking_lost_since = None
                        translation = zed_pose.get_translation(
                            sl.Translation()).get()
                        sub_depth_m = -float(translation[1])
                        node.publish_sub_depth(sub_depth_m)
                        orientation = zed_pose.get_orientation(
                            sl.Orientation()).get()
                        node.publish_odometry(translation, orientation)
                    else:
                        # F8: SEARCHING/OFF still returns the LAST pose — 
                        # publishing it with fresh stamps defeats every
                        # staleness check downstream (frozen heading burns
                        # turn timeouts, frozen fix corrupts cross-track).
                        # Go silent; monitors handle silence correctly.
                        if _tracking_lost_since is None:
                            _tracking_lost_since = time.time()
                            print(f'[Vision] ZED tracking {track_state} — '
                                  f'pose/depth muted until OK')
```

Initialize `_tracking_lost_since = None` next to the other loop-local state at the top of `run_detector`, and confirm `import time` exists (the file uses `from time import sleep` — if so, add `import time` at top and keep both).

- [ ] **Step 2: vslam_node.py**

Same pattern at ~line 189:

```python
                state = self.zed.get_position(zed_pose,
                                              sl.REFERENCE_FRAME.WORLD)
                if state != sl.POSITIONAL_TRACKING_STATE.OK:
                    if state != self._last_bad_state:
                        self.get_logger().warn(
                            f'ZED tracking {state} — odometry muted')
                        self._last_bad_state = state
                    sleep(0.005)
                    continue
                self._last_bad_state = None
```

(keep the existing translation/orientation code below; add `self._last_bad_state = None` in `__init__`).

- [ ] **Step 3: Verify + commit**

```bash
python3 -c "import ast; ast.parse(open('src/vision/vision/detector.py').read()); ast.parse(open('src/localization/localization/vslam_node.py').read()); print('syntax OK')"
python3 -m pytest tests/ -v
git add src/vision src/localization
git commit -m "fix: publish ZED pose/depth only when tracking is OK (audit F8)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

(Import-testing detector.py needs pyzed + GPU; on this Jetson `python3 -c "from vision import detector"` should also pass — run it.)

---

### Task 12: Bottom camera — BGRA fix, topic split, loud model failure (F12)

**Files:**
- Modify: `src/vision/vision/bottom_camera_node.py` (lines 36, 55, ~144, and the enrich call ~319)

**Interfaces:**
- Produces: bottom depth topic RENAMED `depth/sub_depth` → `depth/sub_depth_bottom` (front detector keeps `depth/sub_depth`; Bar02 remains the depth authority). Consumers of `vision/path_markers` and `odom/bottom` unchanged.

- [ ] **Step 1: Channel swap**

Line ~144: `img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)` → 

```python
                img = cv2.cvtColor(image_net, cv2.COLOR_BGRA2RGB)
```

with comment: `# ZED get_data() is BGRA — RGBA2RGB swapped red/blue (F12/F3)`.

(No letterbox change needed here: both branches call `model.predict(...)`, and ultralytics does its own aspect-preserving letterbox internally — unlike the raw TRT path that F3 fixed in detector.py.)

- [ ] **Step 2: Topic split**

Line 55:

```python
        # NOT depth/sub_depth: the front detector owns that topic, and two
        # publishers with different tracking origins interleave garbage (F12).
        # Bar02 is the depth authority anyway — this is diagnostic only.
        self.depth_pub = self.create_publisher(
            Float32, 'depth/sub_depth_bottom', 10)
```

```bash
grep -rn "sub_depth" src/ --include=*.py --include=*.cpp | grep -v sub_depth_bottom
```

Confirm every subscriber of `depth/sub_depth` intends the FRONT camera (DepthMonitor in field_common, depth_node, MissionIO if present). No changes expected — the bottom feed had no legitimate subscribers.

- [ ] **Step 3: Model load failure must be unmissable**

In the model-load `except` block (~line 127), replace the print with:

```python
    except Exception as e:
        print('=' * 70)
        print(f'[BottomCam] FATAL: model load failed: {e}')
        print(f'[BottomCam] weights path: {weights}')
        print('[BottomCam] dfc_rs_26.onnx is NOT in the repo — export and '
              'deploy the downward model (deploy_model.sh), or do not launch '
              'the bottom camera. Bins/path-marker perception is DOWN.')
        print('=' * 70)
        exit_signal = True
        inference_done.set()
        return
```

- [ ] **Step 4: Verify + commit**

```bash
python3 -c "from vision import bottom_camera_node; print('import OK')"
python3 -m pytest tests/ -v
git add src/vision
git commit -m "fix: bottom camera BGRA swap, own depth topic, loud model failure (audit F12)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

(The audit's "unify the two inference threads" refactor is deliberately deferred — high churn, zero behavior delta once both are correct. Revisit post-competition.)

---

### Task 13: Depth enrichment by IoU, not list index (F19)

**Files:**
- Modify: `src/vision/vision/detector.py` (`enrich_depths` ~line 618 + its caller ~line 988), `src/vision/vision/bottom_camera_node.py` (its `enrich_depths` call ~line 319)
- Test: `tests/test_enrich_depths.py`

**Interfaces:**
- Produces: `enrich_depths(info_dicts, zed_objects, img_w, img_h)` — new signature (2 extra args). Both callers updated. `_bbox_iou(...)` module helper.

- [ ] **Step 1: Write the failing test**

`tests/test_enrich_depths.py` (fakes only — no ZED needed for the function itself, but importing detector needs pyzed, fine on this Jetson):

```python
"""Ranges must attach to detections by bbox overlap, not list order (F19) —
the ZED tracker drops unconfirmed boxes and appends persistent tracks, so
index i in object_list is NOT detection i."""
import types
from vision.detector import enrich_depths


def zed_obj(x1, y1, x2, y2, pos):
    return types.SimpleNamespace(
        bounding_box_2d=[[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
        position=pos)


def det(cx, cy, w, h):
    return {'center_x': cx, 'center_y': cy, 'bbox_width': w,
            'bbox_height': h, 'depth_m': -1.0}


def test_reordered_tracker_output_matches_by_geometry():
    # detection 0 = left box, detection 1 = right box…
    infos = [det(0.25, 0.5, 0.2, 0.4), det(0.75, 0.5, 0.2, 0.4)]
    # …but the tracker returns them REVERSED: right first (5 m), left second (2 m)
    objs = types.SimpleNamespace(object_list=[
        zed_obj(832, 216, 1088, 504, (3.0, 0.0, 4.0)),   # right → 5 m
        zed_obj(192, 216, 448, 504, (0.0, 0.0, 2.0)),    # left  → 2 m
    ])
    enrich_depths(infos, objs, img_w=1280, img_h=720)
    assert abs(infos[0]['depth_m'] - 2.0) < 1e-6
    assert abs(infos[1]['depth_m'] - 5.0) < 1e-6


def test_no_overlap_leaves_minus_one():
    infos = [det(0.1, 0.1, 0.05, 0.05)]
    objs = types.SimpleNamespace(object_list=[
        zed_obj(1000, 600, 1200, 700, (1.0, 0.0, 1.0))])
    enrich_depths(infos, objs, img_w=1280, img_h=720)
    assert infos[0]['depth_m'] == -1.0
```

- [ ] **Step 2: Run — expect TypeError (old 2-arg signature)**

`python3 -m pytest tests/test_enrich_depths.py -v`

- [ ] **Step 3: Implement**

Replace `enrich_depths` in detector.py:

```python
def _bbox_iou(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter <= 0.0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return inter / (area_a + area_b - inter)


ENRICH_IOU_MIN = 0.3


def enrich_depths(info_dicts, zed_objects, img_w, img_h):
    """Attach ZED tracked-object ranges to detections by 2D-bbox IoU.

    Index-zipping was wrong (F19): the ZED tracker reorders — it drops
    unconfirmed boxes and appends persistent tracks — so with 2+ objects in
    view the gate could inherit the marker's range. Unmatched detections keep
    depth_m = -1.0.
    """
    for obj in zed_objects.object_list:
        bb = obj.bounding_box_2d               # 4×2 pixel corners
        xs = [float(p[0]) for p in bb]
        ys = [float(p[1]) for p in bb]
        ox1, oy1, ox2, oy2 = min(xs), min(ys), max(xs), max(ys)
        pos = obj.position
        dist = float(np.sqrt(float(pos[0]) ** 2 + float(pos[1]) ** 2
                             + float(pos[2]) ** 2))
        if dist <= 0.01:
            continue
        best, best_iou = None, ENRICH_IOU_MIN
        for info in info_dicts:
            cx, cy = info['center_x'] * img_w, info['center_y'] * img_h
            w, h = info['bbox_width'] * img_w, info['bbox_height'] * img_h
            iou = _bbox_iou(ox1, oy1, ox2, oy2,
                            cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
            if iou > best_iou:
                best, best_iou = info, iou
        if best is not None:
            best['depth_m'] = dist
```

- [ ] **Step 4: Update both callers**

detector.py `run_detector` (~line 988): the frame is in `image_net` — 

```python
                with lock:
                    ih, iw = image_net.shape[:2]
                    enrich_depths(detection_infos, objects, iw, ih)
                    local_infos = list(detection_infos)
```

bottom_camera_node.py (~line 319): find its `enrich_depths(...)`-equivalent call (it may import from detector or duplicate) — `grep -n "enrich" src/vision/vision/bottom_camera_node.py` — and pass the same `iw, ih` from its frame buffer. If it duplicates the old function body, delete the duplicate and import: `from vision.detector import enrich_depths`.

- [ ] **Step 5: Run tests + commit**

```bash
python3 -m pytest tests/ -v
git add src/vision tests/test_enrich_depths.py
git commit -m "fix: match ZED ranges to detections by bbox IoU (audit F19)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 14: Multi-axis dispatch in autonomous_controller (F9)

**Files:**
- Modify: `src/control/control/autonomous_controller.py` (`_dispatch_axes`, ~line 665)

**Interfaces:**
- Consumes: `MovementCommand` 6-DOF fields; thruster `'axes'` verb (all axes applied simultaneously — `set_axes` clears untouched axes to neutral every message).
- Produces: same signature `_dispatch_axes(surge, strafe, depth, yaw)` (depth +down), so `station_keep`/`waypoint` call sites don't change.

- [ ] **Step 1: Rewrite _dispatch_axes**

```python
    def _dispatch_axes(self, surge, strafe, depth, yaw):
        """One multi-axis 'axes' setpoint per tick (F9).

        The old dominant-axis verb dispatch left stale thrust on every
        non-dominant axis (verbs only touch their own axis): once rotate_cw
        was sent, current_r stayed set while later surge_forward ticks never
        cleared it — waypoint corkscrewed, station_keep oscillated. 'axes'
        writes all four axes atomically; unsent axes go neutral.
        depth is +down (submerge-positive), matching MovementCommand.heave.
        """
        m = self._max_speed
        vals = [max(-m, min(m, float(v))) for v in (surge, strafe, depth, yaw)]
        if all(abs(v) < 0.03 for v in vals):
            self._send_stop()
            return
        msg = MovementCommand()
        msg.command = 'axes'
        msg.speed = 0.0
        msg.duration = 0.0
        msg.surge, msg.strafe, msg.heave, msg.yaw_rate = vals
        msg.pitch_rate = 0.0
        msg.roll_rate = 0.0
        self._cmd_pub.publish(msg)
```

- [ ] **Step 2: Check for other stale-axis verb users in this file**

```bash
grep -n "_send_cmd('rotate\|_send_cmd('surge\|_send_cmd('strafe\|_send_cmd('submerge\|_send_cmd('emerge" src/control/control/autonomous_controller.py
```

`_tick_heading_hold` (~line 655) sends bare rotate verbs — safe (yaw is its only axis, and the else-branch `depth_hold` clears z but NOT r… fix it): replace its body's command sends with `self._dispatch_axes(0.0, 0.0, 0.0, yaw_cmd if abs(err_yaw) > 0.05 else 0.0)`. `_tick_search` similarly — inspect and convert any multi-axis sequences to `_dispatch_axes`; leave true single-axis one-shots alone.

- [ ] **Step 3: Verify + commit**

```bash
python3 -c "from control import autonomous_controller; print('import OK')"
python3 -m pytest tests/ -v
git add src/control
git commit -m "fix: atomic multi-axis dispatch — no stale thrust on non-dominant axes (audit F9)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 15: Supervised surfacing mode replaces latched full ascent (F13)

**Files:**
- Modify: `src/control/control/autonomous_controller.py` (`_control_tick` ~line 460, new `_tick_surfacing`, `__init__`)

**Interfaces:**
- Consumes: `_dispatch_axes` (Task 14).
- Produces: mode string `'surfacing'`.

- [ ] **Step 1: Glitch filter + mode entry**

In `__init__` (near other counters): `self._over_depth_count = 0`. Constants near `MAX_DEPTH_M`:

```python
OVER_DEPTH_TRIP_TICKS = 5     # consecutive over-depth ticks before surfacing
                              # (one-tick ZED glitch must not rocket the sub up)
SURFACE_DONE_M = 0.3
SURFACE_ASCENT_EFFORT = 0.4
```

Replace the depth-safety block in `_control_tick`:

```python
            if self._depth_m > MAX_DEPTH_M:
                self._over_depth_count += 1
            else:
                self._over_depth_count = 0
            if (self._over_depth_count >= OVER_DEPTH_TRIP_TICKS
                    and self._mode != 'surfacing'):
                # F13: old code sent one emerge 0.6 with duration 0 and went
                # idle — an UNSUPERVISED full ascent until the thruster
                # watchdog, breaching the surface at speed.
                self.get_logger().error(
                    f'DEPTH SAFETY: {self._depth_m:.2f}m > {MAX_DEPTH_M}m '
                    f'for {self._over_depth_count} ticks — supervised surfacing')
                self._mode = 'surfacing'
```

and add to the mode chain:

```python
            elif self._mode == 'surfacing':
                self._tick_surfacing(dt)
```

- [ ] **Step 2: The supervised state**

```python
    def _tick_surfacing(self, dt):
        """Closed-loop moderate ascent; exits when shallow, then stops (F13)."""
        if self._depth_m <= SURFACE_DONE_M:
            self.get_logger().info(
                f'Surfaced ({self._depth_m:.2f} m) — stopping.')
            self._send_stop()
            self._mode = 'idle'
            return
        self._dispatch_axes(0.0, 0.0, -SURFACE_ASCENT_EFFORT, 0.0)
```

- [ ] **Step 3: Verify + commit**

```bash
python3 -c "from control import autonomous_controller; print('import OK')"
git add src/control
git commit -m "fix: depth safety is a supervised surfacing mode, glitch-filtered (audit F13)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 16: Localization frame fixes (F18)

**Files:**
- Modify: `src/localization/localization/localization_node.py` (`_quat_to_yaw`, `__init__` subscriptions, `_publish`)

**Interfaces:**
- Produces: `vio_source` ROS param (`'vslam'` default | `'bottom'`) — one VIO source per run; Y-up yaw extraction matching `field_common.heading_about_axis`.

- [ ] **Step 1: Y-up yaw**

```python
def _quat_to_yaw(q):
    """Heading about the WORLD-UP axis. Both VIO sources are ZED
    RIGHT_HANDED_Y_UP (vertical = Y) — the Z-up formula read a mix of true
    yaw and pitch/roll, so the fused heading swung whenever the camera
    pitched (F18). Matches field_common.heading_about_axis(axis='y')."""
    siny = 2.0 * (q.w * q.y + q.x * q.z)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)
```

- [ ] **Step 2: One VIO source per run**

In `__init__`, replace the two unconditional `Odometry` subscriptions with:

```python
        # F18: odom/bottom and vslam/odometry previously fed ONE callback —
        # two live sources interleave and the pose jumps between two origins.
        self.declare_parameter('vio_source', 'vslam')   # 'vslam' | 'bottom'
        src = str(self.get_parameter('vio_source').value).lower()
        topic = 'vslam/odometry' if src != 'bottom' else 'odom/bottom'
        self.create_subscription(Odometry, topic, self._odom_cb, 10)
        self.get_logger().info(f'VIO source: {topic} (param vio_source)')
```

- [ ] **Step 3: Document the hybrid frame where it's built**

In `_publish`, above the depth-splice:

```python
        # HYBRID FRAME (documented, F18): x/y are ZED camera-world axes from
        # the VIO source; z is Bar02/ZED depth (+up, hence -depth). Consumers
        # (autonomous_controller waypoint/station_keep) treat it as a local
        # tangent frame — do not mix with raw ZED odometry topics.
```

- [ ] **Step 4: Verify + commit**

```bash
python3 -c "from localization import localization_node; print('import OK')"
python3 -c "
import math, types
from localization.localization_node import _quat_to_yaw
# 90° about Y (Y-up yaw): q = (0, sin45, 0, cos45)
q = types.SimpleNamespace(x=0.0, y=math.sin(math.pi/4), z=0.0, w=math.cos(math.pi/4))
assert abs(_quat_to_yaw(q) - math.pi/2) < 1e-6, _quat_to_yaw(q)
print('yaw formula OK')
"
git add src/localization
git commit -m "fix: Y-up yaw extraction + single VIO source param (audit F18)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 17: Battery monitor — unknown voltage + simulate opt-in (F20)

**Files:**
- Modify: `src/mavlink_thruster_control/mavlink_thruster_control/safety_monitor_node.py` (lines 53, 67–69, 87–93, 126–133)

**Interfaces:**
- Produces: `simulate` defaults `False`; missing endpoint RAISES; `voltage_battery` of 0/65535 treated as unknown (cache untouched → existing staleness timeout reports unknown).

- [ ] **Step 1: Voltage sentinel fix**

In `_drain_sys_status`, replace the fallback block:

```python
                pct = float(msg.battery_remaining)
                if pct < 0:
                    raw_mv = msg.voltage_battery
                    # MAVLink sentinel: UINT16_MAX = "voltage unknown"; the
                    # old code turned 65535 into 65.5 V → curve clamped to
                    # 100% forever (F20). 0 = no reading. Skip both — the
                    # SYS_STATUS_TIMEOUT_S staleness path then reports NaN.
                    if raw_mv in (0, 65535):
                        continue
                    pct = self._voltage_to_pct(raw_mv / 1000.0)
```

- [ ] **Step 2: Simulate is opt-in; no endpoint is fatal**

Line 53: `self.declare_parameter('simulate', False)`. Replace the pymavlink fallback (~67):

```python
        if not HAS_MAVLINK and not self.simulate:
            raise RuntimeError(
                'safety_monitor: pymavlink missing and simulate:=false — '
                'install pymavlink or pass simulate:=true.')
```

In `_open_mavlink`, replace both silent fallbacks:

```python
        if not endpoint:
            raise RuntimeError(
                'safety_monitor: simulate:=false but no udp_endpoint or '
                "serial_port set. Use udp_endpoint:='udp:127.0.0.1:14551' "
                '(a mavlink-router/mavproxy split — NEVER the thruster\'s '
                'serial port: single-reader rule) or pass simulate:=true.')
```

and in the `except`: `raise RuntimeError(f'safety_monitor: MAVLink open failed on {endpoint}: {e}') from e`.

- [ ] **Step 3: Verify + commit**

```bash
python3 -c "
from mavlink_thruster_control.safety_monitor_node import SafetyMonitor
assert SafetyMonitor._voltage_to_pct(13.0) < 20
print('curve OK')
"
git add src/mavlink_thruster_control
git commit -m "fix: battery unknown-voltage sentinel + simulate opt-in (audit F20)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

(Leak detection stays a stub returning false — no hardware exists; the audit flags it so nobody mistakes it for coverage. Leave the stub's comment saying exactly that.)

---

### Task 18: Exception guards on control threads (F21)

**Files:**
- Modify: `field_common.py` (`RampedDriver.__init__`/`_stream_loop`, `DepthKeeper.__init__`/`_loop`, `session()` teardown)

**Interfaces:**
- Produces: `RampedDriver.dead` / `DepthKeeper.dead` flags mission scripts can poll; both loops log + send a stop + retry before dying.

- [ ] **Step 1: RampedDriver guard**

`__init__`: add `self.dead = False` and `self._stream_errors = 0`. Rename the existing `_stream_loop` body into `_stream_tick(self)` (one iteration, no `while`/`sleep` — keep the lock + send logic, return after `self.send(cmd, speed)`), then:

```python
    def _stream_loop(self):
        while True:
            try:
                self._stream_tick()
            except Exception as exc:
                # F21: a bare daemon loop dies silently on the first rclpy
                # hiccup and the thruster holds the last axes for the whole
                # watchdog window. Log, neutralise, retry; give up loudly.
                self._stream_errors += 1
                try:
                    self.get_logger().error(
                        f'streamer error ({self._stream_errors}/5): {exc}')
                    self.send('stop', 0.0)
                except Exception:
                    pass
                if self._stream_errors >= 5:
                    self.dead = True
                    return
                time.sleep(0.5)
            else:
                self._stream_errors = 0
            time.sleep(self._period)
```

- [ ] **Step 2: DepthKeeper guard**

Same shape: `self.dead = False`, `self._loop_errors = 0` in `__init__`; rename the loop body (everything inside `while not self._stop:` except the trailing sleep) to `_tick(self, period)`;

```python
    def _loop(self):
        period = 1.0 / RATE_HZ
        while not self._stop:
            try:
                self._tick(period)
            except Exception as exc:
                self._loop_errors += 1
                try:
                    self.driver.get_logger().error(
                        f'DepthKeeper tick error ({self._loop_errors}/5): {exc}')
                    self._send(0.0, 0.0, 0.0, 0.0)   # neutral, not last-cmd
                except Exception:
                    pass
                if self._loop_errors >= 5:
                    self.dead = True
                    return
            else:
                self._loop_errors = 0
            time.sleep(period)
```

- [ ] **Step 3: Teardown check**

In `session()`'s `finally`, before `driver.idle()`:

```python
        if getattr(driver, 'dead', False):
            print('⚠ streamer thread DIED mid-run (see errors above) — '
                  'motion commands may not have been streaming.')
```

- [ ] **Step 4: Verify + commit**

```bash
python3 -m pytest tests/ -v
python3 -c "import field_common; print('import OK')"
git add field_common.py
git commit -m "fix: guard streamer/keeper threads — log, neutralise, expose dead flag (audit F21)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

Mission scripts should check `keeper.dead` in long loops (`if keeper.dead: abort`) — add that to `gate_task.py`'s main legs while in there.

---

### Task 19: Dropper — package move + passive recv mode (F17)

**Files:**
- Create: `src/mavlink_thruster_control/mavlink_thruster_control/dropper_driver.py` (moved from root `dropper.py`)
- Modify: root `dropper.py` → thin shim; `run_course.py` (import survives via shim)
- Test: `tests/test_dropper_passive.py`

**Interfaces:**
- Produces: `mavlink_thruster_control.dropper_driver.Dropper(master, recv_lock=None)` — passive `master.messages` reads when `master._external_recv_reader` is set; serialized active reads otherwise. Root `dropper.py` re-exports `Dropper` + CLI unchanged. Task 20 consumes.

- [ ] **Step 1: Write the failing test**

`tests/test_dropper_passive.py`:

```python
"""Dropper must not recv_match on a master whose serial recv path is owned by
the Bar02 streamer — two blocking readers race pyserial (audit F17)."""
import time
import types
from mavlink_thruster_control.dropper_driver import Dropper


class FakeMaster:
    target_system = 1
    target_component = 1
    _external_recv_reader = True

    def __init__(self):
        self.messages = {}
        self.recv_calls = 0

    def recv_match(self, **kw):
        self.recv_calls += 1
        raise AssertionError('active recv on an externally-owned master')


def test_passive_read_uses_master_messages():
    m = FakeMaster()
    msg = types.SimpleNamespace(_timestamp=time.time() + 1.0, param_value=184)
    m.messages['PARAM_VALUE'] = msg
    d = Dropper(m)
    assert d._recv('PARAM_VALUE', timeout=0.5) is msg
    assert m.recv_calls == 0


def test_passive_read_ignores_stale_and_times_out():
    m = FakeMaster()
    m.messages['PARAM_VALUE'] = types.SimpleNamespace(
        _timestamp=time.time() - 10.0)     # stale — predates the request
    d = Dropper(m)
    assert d._recv('PARAM_VALUE', timeout=0.3) is None
    assert m.recv_calls == 0
```

- [ ] **Step 2: Run — expect ImportError**

`python3 -m pytest tests/test_dropper_passive.py -v`

- [ ] **Step 3: Move + extend the driver**

```bash
git mv dropper.py src/mavlink_thruster_control/mavlink_thruster_control/dropper_driver.py
```

In `dropper_driver.py`, add `import threading` and change:

```python
class Dropper:
    def __init__(self, master, recv_lock=None):
        self.master = master
        # Serializes ACTIVE recv_match against other readers in the same
        # process (thruster armed-check drain). Irrelevant in passive mode.
        self._recv_lock = recv_lock or threading.Lock()

    # -- internals ----------------------------------------------------------

    def _recv(self, mtype, timeout=3):
        if getattr(self.master, '_external_recv_reader', False):
            return self._recv_passive(mtype, timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                with self._recv_lock:
                    m = self.master.recv_match(type=mtype, blocking=True,
                                               timeout=1)
            except TypeError:
                continue          # pymavlink 2.4.49 _instances crash — retry
            if m:
                return m
        return None

    def _recv_passive(self, mtype, timeout=3):
        """Read from master.messages, filled by whichever thread owns recv
        (Bar02DepthSource streamer). Timestamp-gated so we only accept
        messages that arrived AFTER this call — i.e. responses to the
        command we just sent, not stale leftovers (F17)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.master.messages.get(mtype)
            if m is not None and getattr(m, '_timestamp', 0.0) >= t0:
                return m
            time.sleep(0.05)
        return None
```

In `_servo_raw`, make the drain conditional:

```python
    def _servo_raw(self):
        if not getattr(self.master, '_external_recv_reader', False):
            # drain stale queued messages so we read the value AFTER our
            # command (passive mode needs no drain — timestamp gate does it)
            try:
                with self._recv_lock:
                    while self.master.recv_match(type='SERVO_OUTPUT_RAW',
                                                 blocking=False):
                        pass
            except TypeError:
                pass
        m = self._recv('SERVO_OUTPUT_RAW', timeout=2)
        return getattr(m, f'servo{CHANNEL}_raw', None) if m else None
```

Add a comment near the SERVO_OUTPUT_RAW use: `# servo9_raw+ only exists on MAVLink 2 — on a MAVLink 1 link this verify always fails even though the servo moved (audit F26).`

- [ ] **Step 4: Root shim**

New root `dropper.py`:

```python
#!/usr/bin/env python3
"""Shim — the Dropper driver moved into the mavlink_thruster_control package
so the thruster node's dropper/drop service can import it (audit F14/F17).
CLI and `from dropper import Dropper` keep working. Requires sourced ws."""
from mavlink_thruster_control.dropper_driver import *          # noqa: F401,F403
from mavlink_thruster_control.dropper_driver import main, Dropper  # noqa: F401

if __name__ == '__main__':
    main()
```

(If `dropper_driver.py` has no `main()`, the original CLI entry lived under `if __name__ == '__main__':` — refactor that block into `def main():` in dropper_driver first.)

- [ ] **Step 5: Run tests, check consumers, commit**

```bash
python3 -m pytest tests/ -v
grep -rn "import dropper\|from dropper" --include=*.py . | grep -v attic
python3 -c "import dropper; print('shim OK:', dropper.Dropper)"
git add dropper.py src/mavlink_thruster_control tests/test_dropper_passive.py
git commit -m "refactor: dropper into package with passive-recv mode (audit F17)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 20: dropper/drop service + ReleaseMarker wiring (F14 manipulation half)

**Files:**
- Modify: `thruster_node.py` (service), `src/mavlink_thruster_control/package.xml` + `setup.py` (std_srvs dep), `src/robosub2026/src/mission_io.cpp` + `include/bt_mission/mission_io.hpp` (Trigger client), `src/robosub2026/src/manipulation_nodes.cpp` (`ReleaseMarker`), `src/robosub2026/package.xml`/`CMakeLists.txt` (std_srvs)

**Interfaces:**
- Consumes: `Dropper` (Task 19), `_mav_send_lock` (Task 9).
- Produces: `std_srvs/Trigger` service `dropper/drop`; `MissionIO::callDropper(double timeout_s = 15.0) -> bool`. Note SERVO9 boot behavior: ArduSub 4.5 re-saves `SERVO9_FUNCTION=184` every boot — `Dropper.prepare()` handles per-run setup; never reboot the FC mid-mission expecting it to stick.

- [ ] **Step 1: Service in ThrusterController**

Imports: `from std_srvs.srv import Trigger`, `from rclpy.callback_groups import ReentrantCallbackGroup`, `from mavlink_thruster_control.dropper_driver import Dropper`. In `__init__` after the timers:

```python
        # Dropper rides the same serial link (single-port rule) — expose it
        # as a service so the BT's ReleaseMarker can fire it (F14). Reentrant
        # group: the blocking drop (~5-10 s incl. prepare) must not stall the
        # 10 Hz control loop under the MultiThreadedExecutor.
        self._srv_group = ReentrantCallbackGroup()
        self._dropper = None
        self._drop_lock = threading.Lock()
        self.create_service(Trigger, 'dropper/drop', self._drop_cb,
                            callback_group=self._srv_group)
```

```python
    def _drop_cb(self, request, response):
        if self.simulate or self.master is None or not self.connected:
            response.success = False
            response.message = 'no MAVLink link'
            return response
        if not self._drop_lock.acquire(blocking=False):
            response.success = False
            response.message = 'drop already in progress'
            return response
        try:
            if self._dropper is None:
                self._dropper = Dropper(self.master,
                                        recv_lock=self._mav_recv_lock)
            ok = self._dropper.prepare() and self._dropper.drop_next()
            response.success = bool(ok)
            response.message = 'dropped' if ok else 'dropper reported failure'
        except Exception as exc:
            response.success = False
            response.message = f'drop error: {exc}'
        finally:
            self._drop_lock.release()
        return response
```

Add `self._mav_recv_lock = threading.Lock()` in `__init__` (next to `_mav_send_lock`), and wrap the ACTIVE recv paths — `_check_armed_status`'s drain loop and `_wait_ack`'s active branch — in `with self._mav_recv_lock:` so the dropper's active recvs can't race them (single-reader rule inside one process). Check the real dropper API first: `grep -n "def prepare\|def drop" src/mavlink_thruster_control/mavlink_thruster_control/dropper_driver.py` — if it exposes `drop_right()`/`drop_left()` instead of `drop_next()`, track a `self._drops_done` counter and alternate.

- [ ] **Step 2: MultiThreadedExecutor in main()**

```python
def main(args=None):
    rclpy.init(args=args)
    node = ThrusterController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
```

Dependency: add `<depend>std_srvs</depend>` to `src/mavlink_thruster_control/package.xml`.

- [ ] **Step 3: MissionIO::callDropper**

`include/bt_mission/mission_io.hpp`: add `#include <std_srvs/srv/trigger.hpp>`, member `rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr drop_client_;`, declaration `bool callDropper(double timeout_s = 15.0);`. `mission_io.cpp` ctor: `drop_client_ = node_->create_client<std_srvs::srv::Trigger>("dropper/drop");` and:

```cpp
bool MissionIO::callDropper(double timeout_s) {
  if (!drop_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "dropper/drop service unavailable");
    return false;
  }
  auto fut = drop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  if (rclcpp::spin_until_future_complete(
          node_, fut, std::chrono::duration<double>(timeout_s)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "dropper/drop timed out");
    return false;
  }
  auto res = fut.get();
  if (!res->success)
    RCLCPP_ERROR(node_->get_logger(), "dropper: %s", res->message.c_str());
  return res->success;
}
```

Add `std_srvs` to `src/robosub2026/package.xml` and `find_package(std_srvs REQUIRED)` + the ament dependency list in `CMakeLists.txt`.

- [ ] **Step 4: ReleaseMarker uses it**

```cpp
BT::NodeStatus ReleaseMarker::tick() {
  RCLCPP_INFO(lg(), "[manip] release marker via dropper/drop");
  bool ok = MissionIO::ready() && MissionIO::get().callDropper();
  if (auto bb = config().blackboard) {
    if (ok) {
      int remaining = 2;
      bb->get<int>("markers_remaining", remaining);
      bb->set("markers_remaining", std::max(0, remaining - 1));
      bb->set("marker_dropped", true);
    }
  }
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
```

- [ ] **Step 5: Build + test + commit**

```bash
colcon build --symlink-install --packages-select mavlink_thruster_control bt_mission
source install/setup.bash
python3 -m pytest tests/ -v
git add src/mavlink_thruster_control src/robosub2026
git commit -m "feat: dropper/drop Trigger service, ReleaseMarker fires real dropper (audit F14)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 21: Integrated launch + honest blackboard seeds + honest stubs (F14)

**Files:**
- Create: `src/robosub2026/launch/mission.launch.py`
- Modify: `src/robosub2026/src/bt_executor.cpp` (seeds + link-lost), `src/robosub2026/src/perception_nodes.cpp`, `src/robosub2026/src/mission_io.cpp` (+hpp: link-lost sub)

**Interfaces:**
- Produces: `ros2 launch bt_mission mission.launch.py` brings up thruster → safety → vision → localization → controller → executor (executor delayed 8 s); `MissionIO::thrusterLinkLost()`.

- [ ] **Step 1: mission.launch.py**

```python
"""Integrated competition bring-up (audit F14). The BT's nav() leaves depend
on thruster_node + autonomous_controller + detector being alive — shrub.launch
started only the executor, so 'launch the BT' quietly ran against nothing.

  ros2 launch bt_mission mission.launch.py                      # full stack
  ros2 launch bt_mission mission.launch.py safety_simulate:=true  # no UDP split
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bt_xml', default_value='robosub2026_mission.xml'),
        DeclareLaunchArgument('tree_id', default_value='MainTree'),
        DeclareLaunchArgument('coin_flip', default_value='normal'),
        DeclareLaunchArgument('role', default_value='survey_repair'),
        DeclareLaunchArgument('gate_red_side', default_value='right'),
        DeclareLaunchArgument('run_mode', default_value='semifinal'),
        DeclareLaunchArgument('style_enabled', default_value='true'),
        DeclareLaunchArgument('tick_rate_ms', default_value='50'),
        # safety_monitor must NOT share the thruster's serial port (single-
        # reader rule). Point it at a mavlink-router/mavproxy UDP split, or
        # run simulate until one exists — visibly, via this argument.
        DeclareLaunchArgument('safety_simulate', default_value='false'),
        DeclareLaunchArgument('safety_udp', default_value='udp:127.0.0.1:14551'),

        Node(package='mavlink_thruster_control', executable='thruster_node',
             name='thruster_controller', output='screen',
             parameters=[{'simulate': False, 'watchdog_timeout': 5.0,
                          'flight_mode': 'ALT_HOLD'}]),
        Node(package='mavlink_thruster_control', executable='safety_monitor_node',
             name='safety_monitor', output='screen',
             parameters=[{'simulate': LaunchConfiguration('safety_simulate'),
                          'udp_endpoint': LaunchConfiguration('safety_udp')}]),
        Node(package='vision', executable='detector',
             name='vision_node', output='screen'),
        Node(package='localization', executable='depth_node',
             name='depth_node', output='screen'),
        Node(package='control', executable='autonomous_controller',
             name='autonomous_controller', output='screen'),

        # Executor last — give the sensor/actuator layer time to arm/stream.
        TimerAction(period=8.0, actions=[
            Node(package='bt_mission', executable='bt_executor',
                 name='shrub_executor', output='screen',
                 parameters=[{
                     'bt_xml': LaunchConfiguration('bt_xml'),
                     'tree_id': LaunchConfiguration('tree_id'),
                     'coin_flip': LaunchConfiguration('coin_flip'),
                     'role': LaunchConfiguration('role'),
                     'gate_red_side': LaunchConfiguration('gate_red_side'),
                     'run_mode': LaunchConfiguration('run_mode'),
                     'style_enabled': LaunchConfiguration('style_enabled'),
                     'tick_rate_ms': LaunchConfiguration('tick_rate_ms'),
                 }]),
        ]),
    ])
```

Check `src/robosub2026/CMakeLists.txt` installs `launch/` (it installs shrub.launch.py — the same `install(DIRECTORY launch ...)` covers the new file; verify).

- [ ] **Step 2: Honest seeds**

In `bt_executor.cpp` (~line 98), flip the success-biased seeds and say why:

```cpp
  // F14: seeds below are task-completion FLAGS — seeding them true made
  // unfinished branches "succeed" (mission reported SUCCESS while dropping
  // nothing). False = unfinished work reads as FAILURE, visibly.
  bb->set("marker_in_bin", false);
  bb->set("light_off", false);
  bb->set("aligned", false);
  bb->set("torpedo_hit", false);
  bb->set("correct_basket", false);
```

(leave `plane_crossed`/`inside_bounds`/`vehicle_stable`/`orientation_reached`/`divider_verified` as-is — those are environment assumptions, not completion flags; revisit per-branch when each task leaf gets real logic.)

- [ ] **Step 3: Honest perception stubs**

In `perception_nodes.cpp`, fix the two `cond ? SUCCESS : SUCCESS` sites (grep `SUCCESS : BT::NodeStatus::SUCCESS`):

```cpp
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
```

for both `SearchSlalomPoles` and `DetectMagneticTarget`.

- [ ] **Step 4: Thruster link-lost → critical_failure**

`mission_io.hpp`: member `bool thruster_link_lost_ = false;` + sub + accessor. `mission_io.cpp` ctor:

```cpp
  link_lost_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/safety/thruster_link_lost", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        thruster_link_lost_ = m->data;
      });
```

accessor `bool MissionIO::thrusterLinkLost() { std::lock_guard<std::mutex> lk(mtx_); return thruster_link_lost_; }`. In `bt_executor.cpp`'s per-tick safety block extend:

```cpp
    bool link_lost = shrub::MissionIO::get().thrusterLinkLost();
    bool should_critical = leak || link_lost ||
                           (std::isfinite(batt) && batt < battery_critical_pct);
```

(and add `link_lost` to the RCLCPP_ERROR message).

- [ ] **Step 5: Build + launch smoke (no water, props clear — the thruster node ARMS; use bench with kill switch OUT or Pixhawk unpowered: expect a loud RuntimeError from Task 7, which proves the loud path)**

```bash
colcon build --symlink-install --packages-select bt_mission
source install/setup.bash
timeout 20 ros2 launch bt_mission mission.launch.py safety_simulate:=true; true
```

Expected: all nodes start; with no Pixhawk answering, thruster_node exits with the F6 RuntimeError (loud, correct); executor starts after 8 s.

- [ ] **Step 6: Commit**

```bash
git add src/robosub2026
git commit -m "feat: integrated mission launch, honest BT seeds/stubs, link-lost failsafe (audit F14)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 22: BT motion primitives — real YawSweep + shared rate constant (F24)

**Files:**
- Modify: `src/robosub2026/src/nav_nodes.cpp` (Rotate180 ~181, YawSweep ~193, ExecuteYawRotation ~564), `include/bt_mission/shrub_nodes.hpp` (YawSweep members)

**Interfaces:**
- Produces: `shrub::kYawRateDps = 30.0` (one owner for the open-loop yaw-rate guess, matching Python `TURN_90_SECONDS = 3.0`); YawSweep that actually sweeps right-center-left-center.

- [ ] **Step 1: One rate constant**

Top of `nav_nodes.cpp` (namespace scope):

```cpp
// Open-loop yaw rate at effort 0.25–0.4. Derived from the Python stack's
// TURN_90_SECONDS = 3.0 s ⇒ ~30°/s — HALF the 60°/s these nodes assumed
// (F24). CALIBRATE IN WATER; until the BT primitives get heading feedback
// (delegate to autonomous_controller heading_hold), every duration here is
// this guess.
constexpr double kYawRateDps = 30.0;
```

Rotate180: `double seconds = 180.0 / kYawRateDps; move("rotate_cw", 0.4, seconds); setDuration(seconds);`. ExecuteYawRotation: `double seconds = (360.0 / kYawRateDps) * count;` (was `6.0 * count` — 12 s per rotation now).

- [ ] **Step 2: YawSweep actually sweeps**

`shrub_nodes.hpp`: find the YawSweep declaration (it's a stateful node macro); convert to an explicit class if needed, adding members:

```cpp
  int sweep_phase_{0};
  double phase_s_{1.0};
  std::chrono::steady_clock::time_point phase_end_;
```

`nav_nodes.cpp`:

```cpp
// YawSweep — right sweep°, left through centre 2·sweep°, right back to
// centre. The old version yawed right for 1 s then idled out the deadline —
// no left half at all (F24).
BT::NodeStatus YawSweep::onStart() {
  double sweep = 30.0;
  getInput("sweep_deg", sweep);
  RCLCPP_INFO(lg(), "[gate] yaw sweep ±%.1f° @ ~%.0f°/s", sweep, kYawRateDps);
  phase_s_ = sweep / kYawRateDps;
  sweep_phase_ = 0;
  move("rotate_cw", 0.25, 0.0);
  phase_end_ = std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(phase_s_));
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus YawSweep::onRunning() {
  if (std::chrono::steady_clock::now() < phase_end_)
    return BT::NodeStatus::RUNNING;
  ++sweep_phase_;
  double next_s = (sweep_phase_ == 1) ? 2.0 * phase_s_ : phase_s_;
  if (sweep_phase_ == 1) {
    move("rotate_ccw", 0.25, 0.0);
  } else if (sweep_phase_ == 2) {
    move("rotate_cw", 0.25, 0.0);
  } else {
    stop();
    return BT::NodeStatus::SUCCESS;
  }
  phase_end_ = std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(next_s));
  return BT::NodeStatus::RUNNING;
}
void YawSweep::onHalted() { stop(); }
```

(Match `move()`'s real signature from the neighboring nodes — duration 0 means "until next command", which each phase supplies.)

- [ ] **Step 3: Build + commit**

```bash
colcon build --symlink-install --packages-select bt_mission
git add src/robosub2026
git commit -m "fix: BT yaw primitives use 30 deg/s calibration, YawSweep sweeps both ways (audit F24)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 23: DetectionMonitor.all() for multi-instance labels (F23)

**Files:**
- Modify: `field_common.py` (`DetectionMonitor`)
- Test: `tests/test_detection_monitor.py`

**Interfaces:**
- Produces: `DetectionMonitor.all(label, min_conf=0.5, stale_s=1.0) -> list` — every detection of that label from the LAST FRAME that contained the label (the slalom model uses one `slalom` class for all pipes; `best()` could only ever see one pipe). `best()`/`seen()`/`fresh()` unchanged.

- [ ] **Step 1: Failing test**

`tests/test_detection_monitor.py`:

```python
import rclpy
from auv_msgs.msg import ObjectDetection, ObjectDetectionArray
import field_common


def make_array(entries):
    msg = ObjectDetectionArray()
    for label, conf in entries:
        d = ObjectDetection()
        d.label = label
        d.confidence = conf
        msg.detections.append(d)
    return msg


def test_all_returns_every_instance_of_label():
    rclpy.init()
    try:
        mon = field_common.DetectionMonitor()
        mon._on_dets(make_array([('slalom', 0.9), ('slalom', 0.8),
                                 ('slalom', 0.7), ('gate', 0.6)]))
        pipes = mon.all('slalom', min_conf=0.5)
        assert len(pipes) == 3
        assert mon.best('slalom').confidence == 0.9
        assert len(mon.all('gate')) == 1
        assert mon.all('bin') == []
    finally:
        rclpy.shutdown()
```

- [ ] **Step 2: Run — expect AttributeError**

`python3 -m pytest tests/test_detection_monitor.py -v`

- [ ] **Step 3: Implement**

In `DetectionMonitor.__init__` add `self._frames = {}   # label -> (list[ObjectDetection], monotonic_time)`. Extend `_on_dets`:

```python
    def _on_dets(self, msg: ObjectDetectionArray):
        now = time.monotonic()
        frame = {}
        for det in msg.detections:
            self._latest[det.label] = (det, now)
            frame.setdefault(det.label, []).append(det)
        for label, dets in frame.items():
            self._frames[label] = (dets, now)
```

Add:

```python
    def all(self, label, min_conf=0.5, stale_s=1.0):
        """Every detection of `label` in the last frame containing one (F23).

        best() keeps ONE det per label — with single-class multi-instance
        models (slalom: one label for all pipes, middle-of-three = red) it
        can never see three pipes at once. Multi-object logic builds on this.
        """
        entry = self._frames.get(label)
        if entry is None:
            return []
        dets, t = entry
        if time.monotonic() - t > stale_s:
            return []
        return [d for d in dets if d.confidence >= min_conf]
```

- [ ] **Step 4: Run tests + commit**

```bash
python3 -m pytest tests/ -v
git add field_common.py tests/test_detection_monitor.py
git commit -m "feat: DetectionMonitor.all() keeps full frames per label (audit F23)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

Follow-up (run sheet): the project's `SlalomMonitor` lives in `slalom_task.py` on another machine (per memory) — when that file lands here, port it onto `all()`.

---

### Task 24: Mission-end surface + disarm epilogue (F11 BT half)

**Files:**
- Modify: `src/robosub2026/src/bt_executor.cpp` (after the tick loop, ~line 200)

**Interfaces:**
- Consumes: `MovementCommand 'disarm'` verb (Task 10), `MissionIO::depth()` (−1 until first DepthInfo), `sendMovement`.

- [ ] **Step 1: Replace the bare `stop()` epilogue**

Replace `if (shrub::MissionIO::ready()) shrub::MissionIO::get().stop();` with:

```cpp
  // F11 epilogue — runs on EVERY exit path (timeout, exception, MAX_TICKS,
  // completion). The old stop() left the sub ARMED holding depth: z-neutral
  // in ALT_HOLD means "stay here", and nothing else in the graph could
  // disarm. Surface (closed-loop on depth when we have it, 20 s cap), then
  // release the vehicle via the thruster node's disarm verb.
  if (shrub::MissionIO::ready() && rclcpp::ok()) {
    auto& io = shrub::MissionIO::get();
    RCLCPP_INFO(ros_node->get_logger(), "Epilogue: surfacing, then disarm");
    auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
           std::chrono::duration<double>(
               std::chrono::steady_clock::now() - t0).count() < 20.0) {
      rclcpp::spin_some(ros_node);
      double d = io.depth();
      if (d >= 0.0 && d < 0.2) break;      // surfaced (or never had depth: -1
                                           // still ascends the full 20 s —
                                           // positively buoyant + up-thrust)
      io.sendMovement("emerge", 0.4, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    io.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    io.sendMovement("disarm", 0.0, 0.0);
    RCLCPP_INFO(ros_node->get_logger(), "Vehicle released (disarm sent)");
  }
```

Wait — `d >= 0.0 && d < 0.2` skips ascent when depth unknown (−1)? No: `-1 < 0.0` fails the first clause, so unknown depth keeps ascending until the 20 s cap. Verify `MissionIO::depth()`'s "no data" sentinel by reading `mission_io.cpp` — if it returns something other than a negative number when unseeded, adjust the guard to match.

- [ ] **Step 2: Build + commit**

```bash
colcon build --symlink-install --packages-select bt_mission
git add src/robosub2026
git commit -m "fix: BT executor surfaces and disarms on every terminal path (audit F11)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 25: Legacy runners to attic/ + repo hygiene + F26 small fixes

**Files:**
- Create: `attic/README.md`
- Move: `gate_runner.py`, `check_vertical_direction.py` → `attic/`
- Modify: `.gitignore`, `src/localization/localization/vslam_node.py`, `src/robosub2026/src/mission_io.cpp`, git index (cached junk)

- [ ] **Step 1: attic**

```bash
mkdir -p attic
git mv gate_runner.py attic/
git mv check_vertical_direction.py attic/    # vertical-thrust issue RESOLVED 2026-07-12
cat > attic/README.md <<'EOF'
# attic/ — superseded, DO NOT RUN AT COMPETITION

- gate_runner.py — legacy gate mission: STABILIZE mode, own MAVLink link,
  constant-down-thrust depth, predates every Bar02/heading lesson (audit F25).
  Current mission: gate_task.py / gate_spin_pass.py via field_common.
- check_vertical_direction.py — one-off diagnostic for the MOT_6/8_DIRECTION
  incident (resolved 2026-07-12; params restored from backup).
EOF
git add attic/
```

- [ ] **Step 2: .gitignore + de-track junk**

Append to `.gitignore`:

```
__pycache__/
*.pyc
autotune_logs/
seq_log.txt
auto_yaw_00.csv
*.engine
```

```bash
git rm -r --cached --ignore-unmatch $(git ls-files | grep -E '__pycache__|\.pyc$') 
git rm --cached --ignore-unmatch seq_log.txt auto_yaw_00.csv
git rm -r --cached --ignore-unmatch autotune_logs
git mv "Task1_SurveyRepair (1).svg" docs/task1_survey_repair.svg
```

(Leave `.onnx`/`.pt` weights TRACKED — the detector's default-model resolution depends on them being on every machine; removing them from git is a team decision, not a hygiene fix.)

- [ ] **Step 3: vslam_node F26 items**

- Delete the dead quaternion-order "fallback" branch (grep `fallback` / the try/except around `orient` indexing at ~line 194): indexing `[qw,qx,qy,qz]` succeeds either way, so the except is unreachable and would publish garbage silently if it weren't. Keep ONE documented order.
- `frame_rot_z_deg`: rotates about Z in a Y-up world. Rename the parameter handling to rotate about Y (`frame_rot_y_deg`, keep reading the old param name with a deprecation warn) or, if it's unused (`grep -rn frame_rot src/ *.py` finds no setter), delete it.
- Add a `self._sdk_lock = threading.Lock()` and hold it in the grab loop's `get_position/retrieve_*` block and in any service callback touching `self.zed` (audit: unsynchronized SDK calls from two threads).

- [ ] **Step 4: mission_io.cpp stale comment**

`grep -n "NOT YET COMPILED" src/robosub2026/src/mission_io.cpp` → delete the line.

- [ ] **Step 5: Full verify + commit + push**

```bash
python3 -m pytest tests/ -v
colcon build --symlink-install
source install/setup.bash
python3 -c "import field_common, gate_task, gate_spin_pass, dropper; print('all imports OK')"
git add -A
git commit -m "chore: attic legacy runners, de-track build junk, vslam F26 fixes (audit F25, F26)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
git push origin main
```

---

## Deliberately NOT in scope (decisions, with reasons)

- **F12 full inference-thread unification** — deferred; both threads are correct after Task 12/13, and the refactor risks the working front camera days before competition.
- **F24 full feedback-based BT primitives** — the BT is not the competition path (audit: gate task runs on the Python stack). Task 22 makes the primitives honest; delegation to `autonomous_controller` feedback modes is post-competition work.
- **F18 full ENU/Z-up world conversion** — audit itself suggests the localization path "currently only misleads" and VSLAM is disabled for competition. Task 16 fixes the math and the dual-source bug; a proper TF tree is post-competition.
- **Bar02 cable re-seat / strain relief** — hardware, not code. The audit calls it "worth more than most software fixes here." Put it on the water-day checklist first.

## Verification after all tasks (bench, props clear)

1. `python3 -m pytest tests/ -v` — all green.
2. `colcon build --symlink-install` — clean.
3. `python3 gate_task.py --help && python3 gate_spin_pass.py --help` — import chain healthy.
4. Wrong-port drill: unplug Pixhawk USB, `ros2 run mavlink_thruster_control thruster_node` → must EXIT with the F6 RuntimeError, not print "SIMULATION mode".
5. `timeout 20 ros2 launch bt_mission mission.launch.py safety_simulate:=true` — every node starts, executor last.
6. Water day one: verify `ZED_CW_HEADING_SIGN` (single 30° turn), gyro signs (single style roll — a stall now FAILS loudly), DepthKeeper depth-hold quality with the new THR_DZ mapping, and diff live params against `pixhawk_params_4.5.7_backup_2026-07-08.param` before arming.
