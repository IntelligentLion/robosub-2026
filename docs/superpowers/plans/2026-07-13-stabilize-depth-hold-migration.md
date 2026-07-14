# Stabilized-Mode Migration (MANUAL → STABILIZE/ALT_HOLD) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every script that drives the Pixhawk runs a stabilized flight mode (ALT_HOLD preferred, STABILIZE fallback) instead of MANUAL, so the autopilot's attitude + heading loops actively counter thruster asymmetry (the "forward veers hard right" symptom); MANUAL survives only where it is deliberately required (style rolls, dry bench).

**Architecture:** One shared mode layer in `depth_hold_bar02_test.py` (the de-facto shared MAVLink helper module for root scripts): a verified mode setter, an ALT_HOLD→STABILIZE fallback chooser (Bar02 is intermittent — ALT_HOLD silently bounces to MANUAL when it drops), a per-mode z-stick mapping (ALT_HOLD has a THR_DZ deadzone; STABILIZE/MANUAL are direct throttle), and a runtime mode watchdog. Scripts then migrate to that layer.

**Tech Stack:** Python 3.10 / pymavlink 2.4.49 / ArduSub 4.5.7 (custom modes: STABILIZE=0, ACRO=1, ALT_HOLD=2, MANUAL=19) / pytest.

## Global Constraints

- This machine IS the vehicle (Jetson Orin). Never arm the Pixhawk as part of executing this plan — bench verification means pytest, `--dry-run`, and `python3 -c` import checks only. Steps marked **WATER TEST** are for a human at the pool.
- Single-serial-reader rule: only one thread may `recv_match` on a MAVLink master.
- `depth_hold_bar02_test.py` is read-only w.r.t. FC params (see its `read_param` docstring) — this plan never writes a parameter.
- STABILIZE heading hold runs on gyro-only yaw (`EK3_SRC1_YAW=0` since 2026-07-09, compass hard-iron unusable). Heading drifts slowly; fine for task-length runs, not for absolute headings.
- The Pixhawk is mounted backward: vehicle-forward = autopilot `-x`. Do not change that convention in this plan.
- Style-roll scripts (`gate_spin_pass.py` / roll-pitch style moves) REQUIRE MANUAL — ALT_HOLD/STABILIZE self-level and fight the spin (confirmed 2026-07-10). Do not migrate them.
- Run pytest as: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/ -v`.
- Commit after every task, conventional style, end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `depth_hold_bar02_test.py` | shared mode layer: `FLIGHT_MODES`, `set_mode_verified`, `pick_mode_sequence`, `enter_stabilized_mode`, `z_for_mode`, `ModeWatch`; `set_alt_hold` refactored on top | 1, 2, 3 |
| `tests/test_mode_layer.py` (new) | FakeMaster + unit tests for the mode layer | 1, 2, 3 |
| `submerge_forward_10ft.py` | migrate off `set_manual`; `--manual` escape hatch; ModeWatch in `run_phase` | 4 |
| `submerge_forward.py` | same migration | 4 |
| `diagnose_forward_veer.py` | replace local `set_mode` with shared `set_mode_verified` | 4 |
| `gate_task.py`, `check_horizontal_direction.py`, `depth_hold_pix_test.py` | audit + migrate or explicitly annotate MANUAL | 5 |
| `docs/water-tests/2026-stabilize-migration.md` (new) | pool verification checklist | 6 |

**Not touched:** `field_common.py` / `thruster_node.py` already default to ALT_HOLD (`DEFAULT_FLIGHT_MODE = 'ALT_HOLD'`, `_MODE_IDS` at `thruster_node.py:57-64`). `gate_spin_pass.py` stays MANUAL by design.

---

### Task 1: Verified mode setter + mode table

**Files:**
- Modify: `depth_hold_bar02_test.py` (near `set_alt_hold`, line ~454)
- Test: `tests/test_mode_layer.py` (new)

**Interfaces:**
- Produces: `FLIGHT_MODES: dict[str, int]` (`{'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'MANUAL': 19}`), `set_mode_verified(master, name, timeout=5.0) -> bool`. `set_alt_hold(master)` keeps its exact current signature/behavior (delegates).

- [ ] **Step 1: Write the failing tests with a FakeMaster**

```python
# tests/test_mode_layer.py
"""Mode-layer unit tests. FakeMaster duck-types the two pymavlink surfaces
the mode helpers touch: master.mav.set_mode_send and master.recv_match."""

import depth_hold_bar02_test as dh


class FakeMsg:
    def __init__(self, mtype, **fields):
        self._mtype = mtype
        self.__dict__.update(fields)

    def get_type(self):
        return self._mtype


class FakeMav:
    def __init__(self):
        self.sent_modes = []

    def set_mode_send(self, sys, flags, mode):
        self.sent_modes.append(mode)


class FakeMaster:
    """recv_match pops scripted messages in order; None when empty."""

    def __init__(self, script):
        self.mav = FakeMav()
        self.target_system = 1
        self.target_component = 1
        self._script = list(script)

    def recv_match(self, type=None, blocking=False, timeout=None):
        while self._script:
            msg = self._script.pop(0)
            if type is None or msg.get_type() in type:
                return msg
        return None


def test_set_mode_verified_success():
    m = FakeMaster([
        FakeMsg('COMMAND_ACK', result=0, command=11),
        FakeMsg('HEARTBEAT', custom_mode=0),
    ])
    assert dh.set_mode_verified(m, 'STABILIZE', timeout=0.5) is True
    assert m.mav.sent_modes == [0]


def test_set_mode_verified_bounced_back():
    # FC acks but heartbeat never shows the mode (Bar02-gone behavior)
    m = FakeMaster([
        FakeMsg('COMMAND_ACK', result=0, command=11),
        FakeMsg('HEARTBEAT', custom_mode=19),
        FakeMsg('HEARTBEAT', custom_mode=19),
    ])
    assert dh.set_mode_verified(m, 'ALT_HOLD', timeout=0.5) is False


def test_flight_modes_table():
    assert dh.FLIGHT_MODES == {'STABILIZE': 0, 'ACRO': 1,
                               'ALT_HOLD': 2, 'MANUAL': 19}
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_mode_layer.py -v`
Expected: FAIL — `AttributeError: module ... has no attribute 'set_mode_verified'` / `FLIGHT_MODES`.

- [ ] **Step 3: Implement in `depth_hold_bar02_test.py`**

Insert above `set_alt_hold` (keep `ALT_HOLD_MODE = 2` constant for back-compat):

```python
# ArduSub custom_mode ids — one table for every root script. Matches
# thruster_node._MODE_IDS; keep the two in sync.
FLIGHT_MODES = {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'MANUAL': 19}


def set_mode_verified(master, name, timeout=5.0):
    """Command flight mode `name` and verify via heartbeat custom_mode.
    The ACK alone proves nothing — ArduSub can silently bounce back to
    MANUAL (e.g. ALT_HOLD requested while the Bar02 is off the I2C bus).
    Returns True only once a heartbeat reports the requested mode."""
    mode_num = FLIGHT_MODES[name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num)
    ack = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=3)
    print(f'{name} ACK: result={ack.result}' if ack
          else f'No ACK for set_mode {name} — verifying via heartbeat…')
    hb = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        hb = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == mode_num:
            print(f'Mode verified: {name} active.')
            return True
    print(f'MODE VERIFY FAILED: autopilot not in {name} (last custom_mode='
          f'{hb.custom_mode if hb else "none received"}).')
    return False
```

Then refactor `set_alt_hold` body to:

```python
def set_alt_hold(master):
    """ALT_HOLD with heartbeat verification (see set_mode_verified). Kept
    for back-compat; prints the Bar02 hint on failure."""
    if set_mode_verified(master, 'ALT_HOLD'):
        return True
    print('Depth hold would not work — check the Bar02 (mode 19 = MANUAL '
          'forced because the depth sensor is gone).')
    return False
```

Note: `set_mode_verified` uses `time.monotonic()`; the FakeMaster script just runs dry before the deadline, so a short `timeout=0.5` keeps tests fast.

- [ ] **Step 4: Run tests**

Run: `python3 -m pytest tests/test_mode_layer.py -v`
Expected: 3 PASS. Also run the full suite: `python3 -m pytest tests/ -v` — no regressions.

- [ ] **Step 5: Commit**

```bash
git add depth_hold_bar02_test.py tests/test_mode_layer.py
git commit -m "feat(modes): shared verified mode setter + FLIGHT_MODES table"
```

---

### Task 2: Fallback chooser + per-mode z mapping

**Files:**
- Modify: `depth_hold_bar02_test.py`
- Test: `tests/test_mode_layer.py`

**Interfaces:**
- Consumes: `set_mode_verified`, `vertical_z(effort, direction)`, `NEUTRAL_Z`, `clamp`.
- Produces: `pick_mode_sequence(want_depth_hold: bool) -> list[str]` (pure), `enter_stabilized_mode(master, want_depth_hold=True) -> str | None` (returns the mode actually entered), `z_for_mode(mode_name, effort, direction) -> int`.

- [ ] **Step 1: Write the failing tests**

```python
def test_pick_mode_sequence():
    assert dh.pick_mode_sequence(True) == ['ALT_HOLD', 'STABILIZE']
    assert dh.pick_mode_sequence(False) == ['STABILIZE']


def test_enter_stabilized_mode_falls_back():
    # ALT_HOLD bounces (heartbeats stay 19), STABILIZE verifies.
    m = FakeMaster([
        FakeMsg('COMMAND_ACK', result=0, command=11),
        FakeMsg('HEARTBEAT', custom_mode=19),
        FakeMsg('COMMAND_ACK', result=0, command=11),
        FakeMsg('HEARTBEAT', custom_mode=0),
    ])
    assert dh.enter_stabilized_mode(m, timeout=0.5) == 'STABILIZE'
    assert m.mav.sent_modes == [2, 0]


def test_enter_stabilized_mode_total_failure():
    m = FakeMaster([])
    assert dh.enter_stabilized_mode(m, timeout=0.2) is None


def test_z_for_mode_alt_hold_clears_deadzone():
    # ALT_HOLD: must jump past THR_DZ; identical to vertical_z
    assert dh.z_for_mode('ALT_HOLD', 0.0, -1) == dh.vertical_z(0.0, -1)
    assert abs(dh.z_for_mode('ALT_HOLD', 0.2, -1) - dh.NEUTRAL_Z) \
        > dh.THROTTLE_DZ


def test_z_for_mode_direct_modes_linear():
    # STABILIZE/MANUAL: direct throttle, no deadzone jump
    assert dh.z_for_mode('STABILIZE', 0.5, +1) == dh.NEUTRAL_Z + 250
    assert dh.z_for_mode('MANUAL', 1.0, -1) == dh.NEUTRAL_Z - 500
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_mode_layer.py -v` — new tests FAIL (missing attributes).

- [ ] **Step 3: Implement**

```python
def pick_mode_sequence(want_depth_hold):
    """Ordered mode candidates. ALT_HOLD only makes the list when the run
    wants autopilot depth hold — it needs a healthy Bar02 and bounces to
    MANUAL without one. STABILIZE needs no baro: gyro attitude + heading
    hold, which is exactly what counters a weak-thruster veer."""
    return ['ALT_HOLD', 'STABILIZE'] if want_depth_hold else ['STABILIZE']


def enter_stabilized_mode(master, want_depth_hold=True, timeout=5.0):
    """Try each candidate mode in order; return the name of the one the
    heartbeat confirmed, or None. Callers MUST use z_for_mode() with the
    returned name — the z-stick means different things per mode."""
    for name in pick_mode_sequence(want_depth_hold):
        if set_mode_verified(master, name, timeout=timeout):
            return name
        print(f'{name} not accepted — trying next candidate…')
    print('No stabilized mode accepted (ALT_HOLD needs the Bar02; '
          'STABILIZE needs a healthy AHRS). NOT falling back to MANUAL '
          'automatically — that is a deliberate, prompted choice.')
    return None


def z_for_mode(mode_name, effort, direction):
    """Map (effort 0..1, direction -1 descend/+1 ascend) to a z stick for
    the given mode. ALT_HOLD interprets z as a climb-rate stick with a
    THR_DZ deadzone (use vertical_z's deadzone-clearing map). STABILIZE and
    MANUAL pass z straight to the mixer — linear, no deadzone jump."""
    if mode_name == 'ALT_HOLD':
        return vertical_z(effort, direction)
    return NEUTRAL_Z + direction * round(clamp(effort, 0.0, 1.0) * 500)
```

- [ ] **Step 4: Run tests** — all PASS, full suite green.

- [ ] **Step 5: Commit**

```bash
git add depth_hold_bar02_test.py tests/test_mode_layer.py
git commit -m "feat(modes): ALT_HOLD->STABILIZE fallback + per-mode z mapping"
```

---

### Task 3: Runtime mode watchdog

**Files:**
- Modify: `depth_hold_bar02_test.py` (`ModeWatch` class; wire `HEARTBEAT` into `drain_depth`'s type list)
- Test: `tests/test_mode_layer.py`

**Interfaces:**
- Produces: `ModeWatch(expected_name)` with `.update(custom_mode) -> bool` (False = FC left the expected mode → caller must abort or re-enter) and `.bounces: int`. `drain_depth(...)` gains a 4th return element `custom_mode` (None when no heartbeat buffered) — **breaking change**, update both existing callers (`depth_hold_bar02_test.main`, `submerge_forward_10ft.run_phase`) in this task.

- [ ] **Step 1: Write the failing tests**

```python
def test_mode_watch_ok_and_bounce():
    w = dh.ModeWatch('ALT_HOLD')
    assert w.update(2) is True
    assert w.update(2) is True
    assert w.update(19) is False        # Bar02 gone -> FC forced MANUAL
    assert w.bounces == 1


def test_mode_watch_ignores_none():
    w = dh.ModeWatch('STABILIZE')
    assert w.update(None) is True       # no heartbeat this tick: no verdict
    assert w.bounces == 0
```

- [ ] **Step 2: Run to verify failure** — `AttributeError: ... 'ModeWatch'`.

- [ ] **Step 3: Implement**

```python
class ModeWatch:
    """Detect the FC silently leaving the commanded mode mid-run — the
    'mode 19 spam' failure: Bar02 drops off I2C, ArduSub forces MANUAL,
    depth hold vanishes and the sub sinks while the script keeps sending
    sticks. Feed every heartbeat custom_mode; False means act NOW."""

    def __init__(self, expected_name):
        self.expected_name = expected_name
        self.expected_id = FLIGHT_MODES[expected_name]
        self.bounces = 0
        self.last_seen = None

    def update(self, custom_mode):
        if custom_mode is None:
            return True
        self.last_seen = custom_mode
        if custom_mode == self.expected_id:
            return True
        self.bounces += 1
        print(f'MODE WATCH: FC left {self.expected_name} '
              f'(custom_mode={custom_mode}) — bounce #{self.bounces}.')
        return False
```

In `drain_depth` (line ~736) add `'HEARTBEAT'` to the `recv_match` type list, capture `custom_mode`, and return `depth, yaw, pwm, custom_mode`. Update the two existing call sites to unpack 4 values (search: `drain_depth(`).

- [ ] **Step 4: Run full suite** — PASS; `python3 -c "import submerge_forward_10ft"` imports clean.

- [ ] **Step 5: Commit**

```bash
git add depth_hold_bar02_test.py submerge_forward_10ft.py tests/test_mode_layer.py
git commit -m "feat(modes): ModeWatch + heartbeat in drain_depth (4-tuple)"
```

---

### Task 4: Migrate the surge/depth scripts

**Files:**
- Modify: `submerge_forward_10ft.py` (delete local `set_manual`, line ~55-73), `submerge_forward.py` (same pattern), `diagnose_forward_veer.py` (delete local `set_mode`, use `dh.set_mode_verified`)

**Interfaces:**
- Consumes: `dh.enter_stabilized_mode`, `dh.z_for_mode`, `dh.ModeWatch`, 4-tuple `dh.drain_depth`.

- [ ] **Step 1: `submerge_forward_10ft.py` — replace mode entry**

Delete `set_manual` and `MANUAL_MODE`. Add `--manual` flag (explicit escape hatch for A/B tests). In `main()` replace `if not set_manual(master): return 1` with:

```python
    if args.manual:
        print('EXPLICIT --manual: no attitude/heading assist, open-loop veer.')
        mode = 'MANUAL' if dh.set_mode_verified(master, 'MANUAL') else None
    else:
        mode = dh.enter_stabilized_mode(master, want_depth_hold=True)
    if mode is None:
        return 1
    args.mode = mode
```

In `depth_z` swap `dh.vertical_z(...)` for `dh.z_for_mode(args.mode, effort, direction)` (thread `args` through — it already is). In `run_phase`, unpack the 4-tuple and feed a `ModeWatch`:

```python
        depth_m, _yaw, pwm, cmode = dh.drain_depth(
            master, surface_hpa, args.water_density, args.ptype)
        if not mode_watch.update(cmode):
            print(f'ABORT [{label}]: FC left {mode_watch.expected_name} — '
                  'neutral + disarm (never drive on in a surprise mode).')
            return False
```

(construct `mode_watch = dh.ModeWatch(args.mode)` in `main`, pass into `run_phase`). Update the docstring: the script is no longer "NO IMU dependency" by default — say STABILIZE/ALT_HOLD by default, `--manual` restores the old behavior.

- [ ] **Step 2: `submerge_forward.py` — same migration** (same code pattern; copy the block, don't reference).

- [ ] **Step 3: `diagnose_forward_veer.py`** — delete its local `set_mode()` and call `dh.set_mode_verified(master, 'MANUAL')` / `dh.set_mode_verified(master, 'STABILIZE')` in the wet phase (it MUST keep explicit MANUAL for the raw-veer measurement — that's the experiment).

- [ ] **Step 4: Verify** — `python3 submerge_forward_10ft.py --dry-run` (connects + preflight only, never arms; needs the Pixhawk plugged in, else expect the clean "no heartbeat" exit), `python3 -m pytest tests/ -v` green, `python3 diagnose_forward_veer.py --help` clean.

- [ ] **Step 5: Commit**

```bash
git add submerge_forward_10ft.py submerge_forward.py diagnose_forward_veer.py
git commit -m "feat(modes): surge scripts default to stabilized modes, --manual escape hatch"
```

---

### Task 5: Audit + migrate the remaining Pixhawk scripts

**Files:**
- Modify (as audit dictates): `gate_task.py`, `check_horizontal_direction.py`, `depth_hold_pix_test.py`, `src/prequalification/prequalification/prequalification_node.py`, `src/mavlink_thruster_control/tools/move_forward.py`

- [ ] **Step 1: Audit every mode-touching file**

Run: `grep -rn "set_mode_send\|MANUAL_MODE\|custom_mode\s*=\s*19\|'MANUAL'" --include=*.py . | grep -v venv | grep -v install | grep -v build`
For each hit classify: (a) migrate to `enter_stabilized_mode`, (b) deliberate MANUAL — add a one-line comment `# MANUAL required: <reason>` citing why (style rolls fight self-leveling; dry bench; raw-veer measurement), (c) already stabilized.

- [ ] **Step 2: Apply the classification** — migrations use exactly the Task 4 pattern (fallback + `z_for_mode` + `ModeWatch`). `prequalification_node.py` and `move_forward.py` go through `ThrusterController` — confirm neither passes `flight_mode='MANUAL'`; if one does, remove the override so the ALT_HOLD default applies.

- [ ] **Step 3: Verify** — full pytest suite; `python3 -c "import gate_task"` style import checks for each touched root script; `colcon build --symlink-install --packages-select prequalification mavlink_thruster_control && source install/setup.bash` if either package was touched.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat(modes): finish MANUAL->stabilized migration, annotate deliberate MANUAL"
```

---

### Task 6: Water-test checklist (document only — humans execute)

**Files:**
- Create: `docs/water-tests/2026-stabilize-migration.md`

- [ ] **Step 1: Write the checklist**

```markdown
# Stabilize-migration water test (run top to bottom, tether + kill switch)

1. Baseline veer (MANUAL): `python3 diagnose_forward_veer.py --wet --yes`
   — record MANUAL mean yaw rate (°/s).
2. Stabilized veer: `python3 diagnose_forward_veer.py --wet --stabilize --yes`
   — STABILIZE yaw rate should be < 1/3 of MANUAL's. If not, thruster
   imbalance exceeds the controller authority — do the equalization plan
   first.
3. Depth + surge: `python3 submerge_forward_10ft.py --depth 2` — expect
   ALT_HOLD entry (or STABILIZE fallback printed), straight track, and an
   ABORT (not a sink) if the Bar02 is unplugged mid-run (pull it — really).
4. Mode-bounce drill: start step 3, unplug Bar02 at depth. PASS = ModeWatch
   abort within 2 s, neutral + disarm, sub floats up.
5. Style regression: `gate_spin_pass.py` still rolls freely (MANUAL kept).
```

- [ ] **Step 2: Commit**

```bash
git add docs/water-tests/2026-stabilize-migration.md
git commit -m "docs: stabilize-migration water-test checklist"
```

---

## Self-Review Notes

- Spec coverage: mode table ✔ (T1), fallback for intermittent Bar02 ✔ (T2), deadzone difference ✔ (T2), runtime bounce detection ✔ (T3), script migration ✔ (T4-5), deliberate-MANUAL exceptions preserved ✔ (T5), water verification ✔ (T6).
- `drain_depth` 4-tuple is the one breaking interface change; both known callers updated in the same task (T3) so no task boundary leaves the tree broken.
- Type consistency: `enter_stabilized_mode` returns `str|None`; every consumer checks `None` before arming.
