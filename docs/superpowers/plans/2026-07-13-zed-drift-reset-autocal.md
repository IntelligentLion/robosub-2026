# ZED Drift Reset + Auto-Recalibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Accumulated ZED visual-odometry drift can be zeroed on demand (ROS service) and automatically (tracking-health watchdog), and every ZED owner initializes tracking the same self-calibrating way (gravity-as-origin, IMU fusion).

**Architecture:** Two processes own the front ZED directly via pyzed (single-owner, never both): `vision/detector.py` (`vision_node`, publishes `vslam/odometry` + detections) and `localization/vslam_node.py` (standalone VSLAM). Both get: (1) a `std_srvs/Trigger` reset service that queues a `Camera.reset_positional_tracking(sl.Transform())` executed inside the ZED grab thread (pyzed calls are not thread-safe across threads), (2) shared `TrackingHealth` debounce that auto-resets after sustained non-OK tracking, (3) `set_gravity_as_origin=True` so every (re)start levels itself from the IMU — the "auto calibrate" ask. Full sensor self-calibration only happens on camera open, so "recalibrate hard" = restart the node — `reset_zed_node.sh` already does that; we only document it.

**Tech Stack:** Python 3.10 / ROS 2 Humble / pyzed (ZED SDK) / std_srvs / pytest.

## Global Constraints

- This machine IS the vehicle (Jetson Orin). Bench verification may run ZED nodes (camera is safe) but must never arm the Pixhawk.
- Workspace must be sourced: `source /opt/ros/humble/setup.bash && source install/setup.bash`.
- `colcon build --symlink-install` — Python package edits are live after the FIRST symlink build of that package; if unsure, rebuild: `colcon build --symlink-install --packages-select localization vision`.
- ZED USB3 noise jams 2.4 GHz WiFi — bench-test ZED nodes on ethernet tether or 5 GHz.
- Only ONE process may open the ZED. Stop `vision_node` before bench-testing `vslam_zed_node` and vice versa (`./reset_zed_node.sh --no-relaunch <node>` kills cleanly).
- `numpy<2` — do not bump numpy.
- Run pytest as: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/ -v`.
- Commit after every task, conventional style, end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `src/localization/localization/tracking_health.py` (new) | pure debounce: sustained-bad-tracking → reset decision | 1 |
| `tests/test_tracking_health.py` (new) | unit tests for the debounce | 1 |
| `src/localization/localization/vslam_node.py` | gravity-as-origin init, `~/reset_tracking` service, auto-reset | 2 |
| `src/vision/vision/detector.py` | same three additions for `vision_node` | 3 |
| `field_common.py` | `reset_vslam(node, ...)` service-client helper | 4 |
| `run_course.py` | drift zeroing at task boundaries | 4 |
| `README.md` (ZED section) | when to reset vs restart (`reset_zed_node.sh`) | 5 |

---

### Task 1: TrackingHealth debounce (pure logic)

**Files:**
- Create: `src/localization/localization/tracking_health.py`
- Test: `tests/test_tracking_health.py`

**Interfaces:**
- Produces: `TrackingHealth(bad_after_s=5.0, cooldown_s=10.0)` with `update(ok: bool, now: float) -> bool` — True exactly when an auto-reset should fire now. Fires at most once per `cooldown_s`; a healthy sample re-arms immediately.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_tracking_health.py
from localization.tracking_health import TrackingHealth


def test_no_reset_while_healthy():
    h = TrackingHealth(bad_after_s=5.0)
    assert h.update(True, 0.0) is False
    assert h.update(True, 100.0) is False


def test_reset_fires_after_sustained_bad():
    h = TrackingHealth(bad_after_s=5.0)
    assert h.update(False, 0.0) is False      # just went bad
    assert h.update(False, 4.9) is False      # not sustained yet
    assert h.update(False, 5.1) is True       # fire
    assert h.update(False, 5.2) is False      # cooldown holds


def test_healthy_sample_rearms():
    h = TrackingHealth(bad_after_s=5.0, cooldown_s=10.0)
    h.update(False, 0.0)
    assert h.update(False, 6.0) is True
    assert h.update(True, 7.0) is False       # healthy again
    assert h.update(False, 8.0) is False      # bad clock restarts...
    assert h.update(False, 13.5) is True      # ...5s later fires again


def test_cooldown_blocks_rapid_refires():
    h = TrackingHealth(bad_after_s=1.0, cooldown_s=10.0)
    h.update(False, 0.0)
    assert h.update(False, 1.5) is True
    assert h.update(False, 3.0) is False      # still bad, inside cooldown
    assert h.update(False, 12.0) is True      # cooldown elapsed, still bad
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_tracking_health.py -v`
Expected: FAIL — `ModuleNotFoundError: localization.tracking_health` (or import error before first build).

- [ ] **Step 3: Implement**

```python
# src/localization/localization/tracking_health.py
"""Debounce for ZED tracking state -> auto-reset decisions.

POSITIONAL_TRACKING_STATE flaps during fast motion and bubbles; resetting
on every non-OK sample would thrash pose consumers. Reset only after the
state has been continuously bad for bad_after_s, then hold off cooldown_s
before considering another reset.
"""


class TrackingHealth:
    def __init__(self, bad_after_s=5.0, cooldown_s=10.0):
        self.bad_after_s = bad_after_s
        self.cooldown_s = cooldown_s
        self._bad_since = None
        self._last_reset = None

    def update(self, ok, now):
        """Feed one tracking sample. Returns True exactly when the caller
        should reset positional tracking now."""
        if ok:
            self._bad_since = None
            return False
        if self._bad_since is None:
            self._bad_since = now
        if now - self._bad_since < self.bad_after_s:
            return False
        if self._last_reset is not None \
                and now - self._last_reset < self.cooldown_s:
            return False
        self._last_reset = now
        self._bad_since = now          # require a fresh sustained-bad window
        return True
```

- [ ] **Step 4: Build + run tests**

```bash
colcon build --symlink-install --packages-select localization
source install/setup.bash
python3 -m pytest tests/test_tracking_health.py -v
```
Expected: 4 PASS.

- [ ] **Step 5: Commit**

```bash
git add src/localization/localization/tracking_health.py tests/test_tracking_health.py
git commit -m "feat(zed): TrackingHealth debounce for auto tracking reset"
```

---

### Task 2: vslam_node — gravity-as-origin, reset service, auto-reset

**Files:**
- Modify: `src/localization/localization/vslam_node.py` (init ~line 60-75, `_run_zed_loop` ~line 131-200)

**Interfaces:**
- Consumes: `TrackingHealth` from Task 1.
- Produces: ROS service `~/reset_tracking` (`std_srvs/srv/Trigger`) on the vslam node; pose republishes from identity after reset.

- [ ] **Step 1: Imports + service + flag in `__init__`**

Add imports at top: `from std_srvs.srv import Trigger` and `from localization.tracking_health import TrackingHealth`. In `__init__` (near the other declare_parameter calls):

```python
        self.declare_parameter('auto_reset_bad_tracking', True)
        self._auto_reset = self.get_parameter('auto_reset_bad_tracking').value
        # Reset is QUEUED here and EXECUTED in the ZED grab thread — pyzed
        # calls must all come from the thread that owns the camera.
        self._reset_requested = False
        self.create_service(Trigger, '~/reset_tracking', self._on_reset)

    def _on_reset(self, _req, resp):
        self._reset_requested = True
        resp.success = True
        resp.message = 'tracking reset queued (applied in grab loop)'
        self.get_logger().info('reset_tracking requested')
        return resp
```

- [ ] **Step 2: Self-calibrating tracking init**

In `_run_zed_loop` where `pt_params` is built (~line 145):

```python
            pt_params = sl.PositionalTrackingParameters()
            # Self-calibrating start: initial orientation comes from the
            # IMU gravity vector, so the world frame is level regardless of
            # the attitude the sub had at launch, and every reset re-levels.
            pt_params.set_gravity_as_origin = True
            if self.area_map_path:
                pt_params.area_file_path = self.area_map_path
            pt_params.enable_area_memory = self.enable_area_memory
```

- [ ] **Step 3: Execute queued/auto resets in the grab loop**

Right after the successful-`grab()` path (after `grab_failures = 0`), before `get_position`:

```python
                # Drift zeroing: manual (service) or automatic (sustained
                # bad tracking). Both funnel through the same call, made
                # from THIS thread which owns the camera.
                state = self.zed.get_position(zed_pose,
                                              sl.REFERENCE_FRAME.WORLD)
                ok = state == sl.POSITIONAL_TRACKING_STATE.OK
                auto_fire = self._auto_reset and self._health.update(
                    ok, monotonic())
                if self._reset_requested or auto_fire:
                    why = 'service' if self._reset_requested else \
                        f'auto (tracking bad >{self._health.bad_after_s:.0f}s)'
                    self._reset_requested = False
                    self.zed.reset_positional_tracking(sl.Transform())
                    self.get_logger().warn(
                        f'positional tracking RESET ({why}) — pose is now '
                        'identity, downstream consumers must re-reference')
                    continue
```

The existing `self.zed.get_position(zed_pose, ...)` call below becomes redundant — delete it (the pose captured above is current). Add `self._health = TrackingHealth()` just before the `while` loop, and `from time import monotonic` to the top imports (module already imports `sleep` from `time`).

- [ ] **Step 4: Build + bench verify (ZED plugged in, vision_node stopped)**

```bash
colcon build --symlink-install --packages-select localization && source install/setup.bash
ros2 run localization vslam_node &
ros2 service list | grep reset_tracking          # note the resolved name
ros2 topic echo /vslam/odometry --once           # pose non-zero after moving the sub
ros2 service call /<resolved>/reset_tracking std_srvs/srv/Trigger
ros2 topic echo /vslam/odometry --once           # position back near 0,0,0
```
Expected: reset returns `success: true`, pose re-zeros. Kill the node when done.

- [ ] **Step 5: Commit**

```bash
git add src/localization/localization/vslam_node.py
git commit -m "feat(zed): vslam_node reset_tracking service + gravity origin + auto-reset"
```

---

### Task 3: detector.py (vision_node) — same three additions

**Files:**
- Modify: `src/vision/vision/detector.py` (`VisionNode.__init__` ~line 492-500, `_enable_zed_features` ~line 776, grab loop in `run_detector`)

**Interfaces:**
- Produces: ROS service `~/reset_tracking` on `vision_node`. Same queued-flag pattern; the grab loop in `run_detector` executes it.

- [ ] **Step 1: Service in `VisionNode.__init__`**

```python
        from std_srvs.srv import Trigger      # local import, matches file style
        self.reset_requested = False
        self.create_service(Trigger, '~/reset_tracking', self._on_reset)

    def _on_reset(self, _req, resp):
        self.reset_requested = True
        resp.success = True
        resp.message = 'tracking reset queued (applied in grab loop)'
        return resp
```

- [ ] **Step 2: Gravity-as-origin in `_enable_zed_features`**

```python
    if has_depth:
        pt_params = sl.PositionalTrackingParameters()
        pt_params.set_gravity_as_origin = True   # level from IMU on every enable
        pt_status = zed.enable_positional_tracking(pt_params)
```
(replaces the bare `sl.PositionalTrackingParameters()` call at ~line 787).

- [ ] **Step 3: Execute the queued reset in `run_detector`'s grab loop**

In the main loop, immediately after a successful grab and before the pose/odometry publish (search `vslam/odometry` publish path, `publish` helper ~line 531):

```python
            if positional_tracking_enabled and getattr(
                    node, 'reset_requested', False):
                node.reset_requested = False
                zed.reset_positional_tracking(sl.Transform())
                print('vslam positional tracking RESET (service) — '
                      'pose is identity, consumers must re-reference')
```

No auto-reset here: the detector already has reopen-on-grab-failure recovery, and two independent auto-reset authorities on one pose stream is how you get fighting resets. Auto-reset lives only in `vslam_node` (Task 2); the mission triggers the detector's reset explicitly (Task 4).

- [ ] **Step 4: Build + bench verify** (vslam_zed_node stopped): launch detector, `ros2 service call /<resolved>/reset_tracking std_srvs/srv/Trigger`, watch `vslam/odometry` re-zero. Detector needs the GPU stack; if the bench has no engine built, verify import-only: `python3 -c "from vision import detector"` and defer the live check to the next field session.

- [ ] **Step 5: Commit**

```bash
git add src/vision/vision/detector.py
git commit -m "feat(zed): vision_node reset_tracking service + gravity origin"
```

---

### Task 4: Mission-side drift zeroing

**Files:**
- Modify: `field_common.py` (new helper near `HeadingMonitor`), `run_course.py` (task boundaries)
- Test: `tests/test_reset_vslam_helper.py` (new)

**Interfaces:**
- Produces: `field_common.reset_vslam(node, service_names=('/vision_node/reset_tracking', '/vslam_zed_node/reset_tracking'), timeout=2.0) -> bool` — fire-and-confirm against the first service that exists; safe when neither exists (returns False, mission continues).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_reset_vslam_helper.py
"""reset_vslam must not block the mission when no ZED node is up."""
import rclpy
from rclpy.node import Node

import field_common as fc


def test_reset_vslam_absent_service_returns_false():
    rclpy.init()
    try:
        node = Node('test_reset_vslam')
        assert fc.reset_vslam(node, timeout=0.3) is False
        node.destroy_node()
    finally:
        rclpy.shutdown()
```

- [ ] **Step 2: Run to verify failure** — `AttributeError: module 'field_common' has no attribute 'reset_vslam'`.

- [ ] **Step 3: Implement in `field_common.py`**

```python
def reset_vslam(node, service_names=('/vision_node/reset_tracking',
                                     '/vslam_zed_node/reset_tracking'),
                timeout=2.0):
    """Zero accumulated ZED drift before a leg that navigates on
    vslam/odometry. Tries each known reset service; first one present wins.
    Never blocks the mission: absent services -> False and move on. The
    caller's executor must be spinning (session() runs one) — the future
    completes there while we poll it here."""
    from std_srvs.srv import Trigger
    for name in service_names:
        cli = node.create_client(Trigger, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                continue
            fut = cli.call_async(Trigger.Request())
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not fut.done():
                time.sleep(0.05)
            if fut.done() and fut.result() is not None \
                    and fut.result().success:
                print(f'vslam drift reset via {name}')
                return True
        finally:
            node.destroy_client(cli)
    print('vslam reset: no reset_tracking service reachable — continuing '
          'with accumulated drift')
    return False
```

- [ ] **Step 4: Wire into `run_course.py`** — at each task boundary where a new leg starts navigating on ZED coordinates (read the file, find the leg-start points; typically right before the gate approach and before each subsequent task's search), insert `fc.reset_vslam(driver)` (any spinning node works). Do NOT reset mid-leg: consumers hold references into the old frame.

- [ ] **Step 5: Run tests + commit**

```bash
python3 -m pytest tests/test_reset_vslam_helper.py tests/ -v
git add field_common.py run_course.py tests/test_reset_vslam_helper.py
git commit -m "feat(zed): mission-side vslam drift reset at task boundaries"
```

---

### Task 5: Document reset vs restart

**Files:**
- Modify: `README.md` (ZED section)

- [ ] **Step 1: Add the decision table**

```markdown
### ZED drift / wedge decision table

| Symptom | Fix | Command |
|---|---|---|
| Pose drifted but frames flow | tracking reset (fast, in-place) | `ros2 service call /vision_node/reset_tracking std_srvs/srv/Trigger` |
| Tracking state stuck SEARCHING | auto-reset fires after 5 s (vslam_node) — or trigger manually as above | — |
| Frozen frames / CAMERA STREAM FAILED / stale pose | node restart, full open-time self-calibration | `./reset_zed_node.sh` |
| USB gone from bus | replug ZED (USB 3.0 port), then `reset_zed_node.sh` | — |

Tracking resets re-level from the IMU (`set_gravity_as_origin`) — the world
frame is re-created at the current pose; anything holding old-frame
coordinates must re-reference.
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs(zed): drift reset vs node restart decision table"
```

---

## Self-Review Notes

- Spec coverage: on-demand reset ✔ (T2/T3 services), drift auto-handling ✔ (T2 TrackingHealth), "auto calibrate" ✔ (gravity-as-origin at every enable + documented open-time self-calibration path via reset_zed_node.sh, T5), mission integration ✔ (T4).
- Thread-safety: all pyzed calls stay in the grab thread; services only flip a bool (atomic under the GIL).
- Deliberate asymmetry: auto-reset only in `vslam_node`, documented in T3 step 3 — not an omission.
- pyzed API names used: `PositionalTrackingParameters.set_gravity_as_origin`, `Camera.reset_positional_tracking(sl.Transform())`, `POSITIONAL_TRACKING_STATE.OK` — verify against the installed SDK with `python3 -c "import pyzed.sl as sl; print(hasattr(sl.PositionalTrackingParameters(), 'set_gravity_as_origin'), hasattr(sl.Camera, 'reset_positional_tracking'))"` before Task 2; if the installed SDK spells any of these differently, adapt in place.
