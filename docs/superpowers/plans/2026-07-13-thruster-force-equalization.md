# Thruster Force Equalization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make equal commands produce equal thrust on all 8 thrusters by derating the STRONGER motors down to the weakest one's output (per-motor `SERVOn_MIN/MAX` narrowing), driven by measured per-motor current from `diagnose_forward_veer.py --sweep` — and keep the non-skippable preflight gate consistent with the applied trim.

**Architecture:** ArduSub does all mixing from MANUAL_CONTROL sticks, so per-motor compensation cannot live in Python control code; the only per-output knob is the PWM span `[SERVOn_MIN, SERVOn_MAX]` around 1500 µs (existing `motor_trim.py` mechanism). This plan upgrades `motor_trim.py` from "one weak motor, uniform derate of the rest" to "per-motor factors computed from measured currents", persists the applied trim to `thruster_trim.json`, and teaches `depth_hold_bar02_test.verify_thruster_params` (the hard preflight every dive script runs) to expect the trimmed endpoints instead of aborting on them. The measure → apply → re-measure loop closes with the diagnostic sweep.

**Trade-off (unchanged from motor_trim.py):** derating costs top-end thrust ≈ (1−factor) on derated motors. This is a software band-aid; the real fix is hardware (prop, debris, bearing, ESC bidirectional throttle calibration — a thruster weak in only ONE direction is an ESC calibration problem, not a trim problem).

**Tech Stack:** Python 3.10 / pymavlink 2.4.49 / ArduSub 4.5.7 / pytest.

## Global Constraints

- This machine IS the vehicle. Never arm the Pixhawk executing this plan. `motor_trim.py` param WRITES are also field-only: plan execution builds and tests the code paths with fakes; a human runs the tool at the pool (**WATER TEST** steps).
- The preflight gate stays NON-SKIPPABLE. This plan changes what the gate EXPECTS (trim-aware), never whether it runs. A live param matching neither the backup nor the recorded trim must still hard-abort.
- Order dependency: run `diagnose_forward_veer.py --sweep` IN WATER first (props loaded) — dry current numbers are noise. Sweep output is this plan's input.
- `pixhawk_params_4.5.7_backup_2026-07-08.param` stays the untouched baseline; the trim delta lives ONLY in `thruster_trim.json` (repo root, committed), so "what changed vs backup" is always one file.
- Motor↔output mapping is identity on this frame (`SERVO{n}_FUNCTION = Motor n`, verified by the preflight) but `motor_trim.py` still resolves it via `motor_output_map()` — keep that.
- Stop `thruster_node` before running any tool on the Pixhawk serial (single owner).
- Run pytest as: `cd ~/robosub2026/robosub-2026 && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest tests/ -v`.
- Commit after every task, conventional style, end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

---

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `motor_trim.py` | + `factors_from_currents`, `endpoints_for_factor`, `--currents`, `--factors`, trim-record write; `--reset` clears record | 1, 2 |
| `thruster_trim.json` (new, written by tool) | the one persisted trim record | 2 |
| `tests/test_trim_math.py` (new) | factor math + endpoints + record round-trip | 1, 2 |
| `depth_hold_bar02_test.py` | trim-aware preflight expectations | 3 |
| `tests/test_preflight_trim.py` (new) | expectation-merge unit tests | 3 |
| `docs/water-tests/2026-thruster-equalization.md` (new) | measure→apply→verify field procedure | 4 |

---

### Task 1: Factor math (pure, TDD)

**Files:**
- Modify: `motor_trim.py` (new pure functions near the constants)
- Test: `tests/test_trim_math.py`

**Interfaces:**
- Produces: `factors_from_currents(currents: dict[int, float|None], floor=0.7) -> dict[int, float]` and `endpoints_for_factor(factor: float) -> tuple[int, int]`. Existing constants consumed: `NEUTRAL_US=1500`, `HALF_SPAN_US=400`.

Factor model: thrust rises faster than linearly with current on T200-class thrusters, so matching thrust needs less PWM reduction than the raw current ratio — `factor_m = sqrt(I_weakest / I_m)` is the conservative first cut (never over-derates). It's a starting point; the field loop (Task 4) iterates against re-measured currents.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_trim_math.py
import json

import pytest

import motor_trim as mtim


def test_factors_balanced_group_all_full():
    f = mtim.factors_from_currents({1: 3.0, 2: 3.0, 3: 3.0, 4: 3.0})
    assert f == {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0}


def test_factors_strong_motor_derated_sqrt():
    f = mtim.factors_from_currents({1: 3.0, 2: 3.0, 3: 4.32, 4: 3.0})
    assert f[1] == 1.0 and f[2] == 1.0 and f[4] == 1.0
    assert f[3] == pytest.approx((3.0 / 4.32) ** 0.5, abs=1e-6)


def test_factors_floor_clamps():
    # 4x the weakest current would want factor 0.5 -> clamped to floor
    f = mtim.factors_from_currents({1: 1.0, 2: 4.0}, floor=0.7)
    assert f[2] == 0.7


def test_factors_skips_missing_and_nonpositive():
    f = mtim.factors_from_currents({1: 3.0, 2: None, 3: 0.0, 4: 3.3})
    assert set(f) == {1, 4}


def test_factors_empty_raises():
    with pytest.raises(ValueError):
        mtim.factors_from_currents({1: None})


def test_endpoints_for_factor():
    assert mtim.endpoints_for_factor(1.0) == (1100, 1900)
    assert mtim.endpoints_for_factor(0.85) == (1160, 1840)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest tests/test_trim_math.py -v`
Expected: FAIL — `AttributeError: ... 'factors_from_currents'`.

- [ ] **Step 3: Implement in `motor_trim.py`** (below the constants; add `import math` and `import json` up top)

```python
def factors_from_currents(currents, floor=0.7):
    """Per-motor derate factors from measured same-PWM current draw
    (diagnose_forward_veer.py --sweep, net amps above idle, IN WATER).

    The weakest motor keeps factor 1.0 (full span); every stronger motor is
    derated by sqrt(I_weak / I_m) — thrust grows superlinearly with
    current, so the sqrt under-derates rather than over-derates; iterate
    with a re-sweep. Motors with missing/non-positive current are omitted
    (unmeasured — leave their span alone rather than guess)."""
    valid = {m: a for m, a in currents.items() if a is not None and a > 0.0}
    if not valid:
        raise ValueError('no positive current measurements')
    weakest = min(valid.values())
    return {m: max(floor, min(1.0, math.sqrt(weakest / a)))
            for m, a in sorted(valid.items())}


def endpoints_for_factor(factor):
    """Derate factor -> (SERVOn_MIN, SERVOn_MAX) around 1500 us neutral."""
    half = int(round(HALF_SPAN_US * factor))
    return NEUTRAL_US - half, NEUTRAL_US + half
```

- [ ] **Step 4: Run tests** — 6 PASS; full suite green (`motor_trim` has a `__main__` guard, import-safe).

- [ ] **Step 5: Commit**

```bash
git add motor_trim.py tests/test_trim_math.py
git commit -m "feat(trim): per-motor factor math from measured currents"
```

---

### Task 2: motor_trim.py apply paths + persisted trim record

**Files:**
- Modify: `motor_trim.py` (args, apply, reset)
- Test: `tests/test_trim_math.py`

**Interfaces:**
- Produces: CLI `--currents "1:3.2,2:3.1,3:4.0,4:3.3"` (compute+confirm+apply), `--factors "3:0.88,4:0.95"` (direct apply), both writing `thruster_trim.json`; `--reset` restores full span AND deletes the record. Record schema (consumed by Task 3):

```json
{
  "applied_utc": "2026-07-14T02:11:00Z",
  "source": "currents",
  "source_currents": {"1": 3.2, "2": 3.1, "3": 4.0, "4": 3.3},
  "factors": {"3": 0.88, "4": 0.95},
  "endpoints": {"3": [1148, 1852], "4": [1120, 1880]}
}
```
(`factors`/`endpoints` list ONLY motors not at full span; untouched motors are implicitly 1100–1900.)

- Produces (pure, for tests): `parse_motor_map('1:3.2,2:3.1') -> dict[int, float]`, `build_trim_record(factors, source, source_currents=None, now_utc=None) -> dict`.

- [ ] **Step 1: Write the failing tests**

```python
def test_parse_motor_map():
    assert mtim.parse_motor_map('1:3.2, 3:4.0') == {1: 3.2, 3: 4.0}


def test_parse_motor_map_rejects_bad_motor():
    with pytest.raises(ValueError):
        mtim.parse_motor_map('9:1.0')


def test_build_trim_record_only_derated_motors():
    rec = mtim.build_trim_record(
        {1: 1.0, 2: 1.0, 3: 0.88}, source='currents',
        source_currents={1: 3.0, 2: 3.0, 3: 4.0},
        now_utc='2026-07-14T00:00:00Z')
    assert rec['factors'] == {'3': 0.88}
    assert rec['endpoints'] == {'3': list(mtim.endpoints_for_factor(0.88))}
    assert rec['applied_utc'] == '2026-07-14T00:00:00Z'
    assert '1' not in rec['factors']
```

- [ ] **Step 2: Run to verify failure** — missing attributes.

- [ ] **Step 3: Implement**

```python
TRIM_FILE = 'thruster_trim.json'


def parse_motor_map(text):
    """'1:3.2,3:4.0' -> {1: 3.2, 3: 4.0}; motors must be 1..8."""
    out = {}
    for part in text.split(','):
        m_str, _, v_str = part.strip().partition(':')
        m = int(m_str)
        if not 1 <= m <= NUM_MOTORS:
            raise ValueError(f'motor {m} out of range 1..{NUM_MOTORS}')
        out[m] = float(v_str)
    return out


def build_trim_record(factors, source, source_currents=None, now_utc=None):
    """Record ONLY derated motors; full-span motors stay implicit so the
    preflight's default expectation (1100-1900) covers them."""
    derated = {str(m): round(f, 4) for m, f in sorted(factors.items())
               if f < 1.0}
    return {
        'applied_utc': now_utc or time.strftime(
            '%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'source': source,
        'source_currents': ({str(m): c for m, c in
                             sorted(source_currents.items())}
                            if source_currents else None),
        'factors': derated,
        'endpoints': {m: list(endpoints_for_factor(f))
                      for m, f in derated.items()},
    }
```

CLI wiring in `main()`: add `ap.add_argument('--currents')` and `ap.add_argument('--factors')` (mutually exclusive with `--weak`); path:

```python
    if args.currents or args.factors:
        if args.currents:
            currents = parse_motor_map(args.currents)
            factors = factors_from_currents(currents)
            source = 'currents'
        else:
            factors = parse_motor_map(args.factors)
            source = 'factors'
            currents = None
        print('Computed derate factors (weakest = 1.0):')
        for m, f in sorted(factors.items()):
            lo, hi = endpoints_for_factor(f)
            print(f'  motor {m}: {f:.3f} -> {lo}-{hi}')
        if input('Apply to the FC now? [yes/NO] ').strip().lower() != 'yes':
            print('Aborted, nothing written.')
            return
        ok = True
        for m, f in sorted(factors.items()):
            out = mapping[m]
            lo, hi = endpoints_for_factor(f)
            for name, val in ((f'SERVO{out}_MIN', lo), (f'SERVO{out}_MAX', hi)):
                if set_param(master, name, val):
                    print(f'  motor {m} (output {out}): {name} = {val}')
                else:
                    print(f'  motor {m} (output {out}): FAILED {name}')
                    ok = False
        rec = build_trim_record(factors, source, source_currents=currents)
        with open(TRIM_FILE, 'w') as f:
            json.dump(rec, f, indent=2)
        print(f'Trim record written: {TRIM_FILE} — the preflight gate now '
              'expects these endpoints. Commit this file.')
```

`--reset` path additionally:

```python
        import os
        if os.path.exists(TRIM_FILE):
            os.remove(TRIM_FILE)
            print(f'{TRIM_FILE} removed — preflight expects full span again.')
```

Keep `--weak/--factor` working (legacy path) but make it ALSO write the record via `build_trim_record` — otherwise legacy use desyncs the gate.

- [ ] **Step 4: Run tests + `python3 motor_trim.py --help`** — PASS / clean help.

- [ ] **Step 5: Commit**

```bash
git add motor_trim.py tests/test_trim_math.py
git commit -m "feat(trim): --currents/--factors apply + thruster_trim.json record"
```

---

### Task 3: Trim-aware preflight gate

**Files:**
- Modify: `depth_hold_bar02_test.py` (`EXPECT_SERVO_MIN/MAX` use, `verify_thruster_params` ~line 553-590)
- Test: `tests/test_preflight_trim.py`

**Interfaces:**
- Produces: `expected_endpoints(trim_record: dict | None) -> dict[int, tuple[float, float]]` (pure) and `load_trim_record(path='thruster_trim.json') -> dict | None`. `verify_thruster_params` compares `SERVO{m}_MIN/MAX` against the per-motor expectation and PRINTS when a trim record is active. Everything else about the gate (non-skippable, all checks) unchanged.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_preflight_trim.py
import json

import depth_hold_bar02_test as dh


def test_expected_endpoints_no_record_full_span():
    exp = dh.expected_endpoints(None)
    assert exp[1] == (1100.0, 1900.0)
    assert set(exp) == set(range(1, 9))


def test_expected_endpoints_with_record():
    rec = {'endpoints': {'3': [1148, 1852]}}
    exp = dh.expected_endpoints(rec)
    assert exp[3] == (1148.0, 1852.0)
    assert exp[4] == (1100.0, 1900.0)     # untouched motor: full span


def test_load_trim_record_roundtrip(tmp_path):
    p = tmp_path / 'thruster_trim.json'
    p.write_text(json.dumps({'endpoints': {'2': [1160, 1840]}}))
    rec = dh.load_trim_record(str(p))
    assert rec['endpoints']['2'] == [1160, 1840]
    assert dh.load_trim_record(str(tmp_path / 'missing.json')) is None
```

- [ ] **Step 2: Run to verify failure** — missing attributes.

- [ ] **Step 3: Implement** (next to the `EXPECT_*` constants)

```python
def load_trim_record(path='thruster_trim.json'):
    """The motor_trim.py record of deliberately narrowed endpoints, or
    None. Malformed file -> treated as None with a loud warning (the gate
    then expects full span, so an applied-but-corrupt trim FAILS the
    preflight — fail toward not diving)."""
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except (json.JSONDecodeError, OSError) as e:
        print(f'WARNING: {path} unreadable ({e}) — ignoring trim record.')
        return None


def expected_endpoints(trim_record):
    """Per-motor (MIN, MAX) the preflight should expect: full span unless
    the trim record narrows that motor."""
    exp = {m: (EXPECT_SERVO_MIN, EXPECT_SERVO_MAX) for m in range(1, 9)}
    if trim_record:
        for m_str, (lo, hi) in trim_record.get('endpoints', {}).items():
            exp[int(m_str)] = (float(lo), float(hi))
    return exp
```

In `verify_thruster_params`, before the loop:

```python
    trim = load_trim_record()
    endpoints = expected_endpoints(trim)
    if trim:
        print(f'  trim record active (applied {trim.get("applied_utc")}) — '
              f'derated motors: {sorted(trim.get("factors", {}))}')
```

and in the per-motor `checks` tuple replace the two endpoint lines:

```python
            (f'SERVO{m}_MIN', endpoints[m][0]),
            (f'SERVO{m}_MAX', endpoints[m][1]),
```

Add `import json` to the module imports. Update the abort message to mention both sources: `'...does NOT match the known-good backup (+ thruster_trim.json if present).'`

- [ ] **Step 4: Run tests** — new tests PASS; **full suite** PASS (mode-layer/preflight tests from other plans must still pass — the gate's behavior without a record is bit-identical).

- [ ] **Step 5: Commit**

```bash
git add depth_hold_bar02_test.py tests/test_preflight_trim.py
git commit -m "feat(preflight): gate accepts recorded thruster trim endpoints"
```

---

### Task 4: Field procedure (document only — humans execute)

**Files:**
- Create: `docs/water-tests/2026-thruster-equalization.md`

- [ ] **Step 1: Write it**

```markdown
# Thruster equalization loop (sub IN WATER, thruster_node stopped)

0. Hardware first: pull each horizontal prop, check debris/damage; a motor
   weak in ONE direction only = ESC bidirectional calibration, not trim.
1. Measure: `python3 diagnose_forward_veer.py --sweep --both --yes`
   -> note net amps per motor at 60% (and 40%).
2. Compute+apply: `python3 motor_trim.py --currents "1:<A>,2:<A>,3:<A>,4:<A>"`
   (paste the 60% net amps). Confirm. thruster_trim.json is written —
   `git add thruster_trim.json && git commit`.
3. Re-measure: rerun step 1. PASS = every horizontal within ±10% of the
   group median. Off? Nudge single factors:
   `python3 motor_trim.py --factors "3:0.84"` and repeat.
4. Prove it: `python3 diagnose_forward_veer.py --wet --yes` — MANUAL-mode
   yaw rate should drop to near the verticals' noise floor (<2°/s).
5. Preflight regression: `python3 submerge_forward_10ft.py --dry-run`
   — gate must PASS with the trim active and print the trim-record line.
6. Undo path: `python3 motor_trim.py --reset` (restores 1100-1900,
   deletes thruster_trim.json).

Ceiling check: factors < ~0.8 mean you're giving up >20% of that motor's
thrust — fix the hardware instead of trimming deeper.
```

- [ ] **Step 2: Commit**

```bash
git add docs/water-tests/2026-thruster-equalization.md
git commit -m "docs: thruster equalization field procedure"
```

---

## Self-Review Notes

- Spec coverage: "make stronger thrusters weaker to match the weakest" ✔ (T1 factor model + T2 apply), measurement source ✔ (diagnose sweep, constraints + T4), persistence ✔ (T2 record), the critical preflight interaction ✔ (T3 — without it every dive script would hard-abort after trimming), field loop + undo ✔ (T4).
- Failure directions checked: corrupt trim file → gate expects full span → trimmed FC fails preflight → no dive (safe). Legacy `--weak` path also writes the record (T2) so it can't desync the gate.
- Type consistency: record keys are strings (JSON), converted at the boundary (`expected_endpoints`, tests cover both); `endpoints_for_factor` returns ints, gate compares with 0.5 tolerance as before.
