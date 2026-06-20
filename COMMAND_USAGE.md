# Command Usage — Field-Test Toolkit & Depth-Hold Flags

Usage and parameter breakdown for the most recent session's changes:

- Isolated-action + stage water-test toolkit (`act_*.py`, `stage_*.py`, `field_common.py`)
- `ThrusterController` `flight_mode` parameter (default `ALT_HOLD`)
- Prequal `depth_hold_source` parameter (`baro` | `zed`)
- ZED reconnect resilience (`detector.py`, `vslam_node.py`) — operational notes

> ⚠ **SAFETY** — every `act_*`/`stage_*` tool and the `*_test.py` scripts ARM the
> Pixhawk and drive the REAL thrusters. Stop `thruster_node` first (it is the
> single owner of the serial port). Clear the props, run on a tether, keep the
> kill switch reachable. `Ctrl+C` → stop + disarm.

Source the workspace before any command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 1. Shared movement flags (`field_common.add_move_args`)

Every `act_*`/`stage_*` movement tool exposes the same tuning knobs. Per-tool
defaults differ (noted below); the base defaults:

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--speed` | float | `0.3` | target effort `0–1` (clamped) |
| `--ramp-up` | float | `1.0` | seconds to ramp `0 → speed` |
| `--ramp-down` | float | `0.5` | seconds to ramp `speed → 0` |
| `--duration` | float | `3.0` | seconds to hold at target speed |
| `--pause` | float | `2.0` | depth-hold pause after the action |
| `--yes` | flag | off | skip the "Props clear? type go" confirm prompt |

Ramping is linear at 10 Hz (`RATE_HZ`), so thrusters spin up/down smoothly
instead of stepping to full power. Tools run the production
`ThrusterController` in-process (arm, **ALT_HOLD** by default, heartbeat,
watchdog) and guarantee stop + disarm on exit.

---

## 2. Isolated actions (`act_*.py`)

Single maneuver → pause holding depth.

### `act_forward.py` — surge forward
```bash
python3 act_forward.py --speed 0.4 --ramp-up 1.5 --duration 3 --pause 2
```
Shared flags only. Defaults: speed `0.3`, duration `3.0`.

### `act_turn_left.py` — yaw CCW
```bash
python3 act_turn_left.py --speed 0.3 --ramp-up 1.0 --duration 6 --pause 2
```
Shared flags only. **Default `--duration` is `6.0`** — tune it to land ~90°.

### `act_strafe_left.py` / `act_strafe_right.py` — lateral strafe
```bash
python3 act_strafe_left.py  --speed 0.35 --ramp-up 1.0 --duration 3 --pause 2
python3 act_strafe_right.py --speed 0.35 --ramp-up 1.0 --duration 3 --pause 2
```
Shared flags only. Defaults: speed `0.3`, duration `3.0`.

### `act_center_gate.py` — closed-loop yaw to centre on the gate
Spawns the TensorRT detector in-process; yaws until the gate's normalised
centre-x is within `--tol` or `--timeout` expires. Yaw effort is proportional
to centring error, clamped to `[--min-speed, --speed]`. Needs the ZED connected.
```bash
python3 act_center_gate.py --speed 0.3 --gain 0.6 --tol 0.08
```

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--label` | str | `gate` | detection label to centre on |
| `--gain` | float | `0.6` | yaw effort per unit centre-x error |
| `--min-speed` | float | `0.4` | min yaw effort while correcting |
| `--tol` | float | `0.08` | `|centre-x − 0.5|` under which centred |
| `--conf` | float | `0.5` | min detection confidence |
| `--timeout` | float | `20.0` | give up centring after N seconds |
| _shared_ | | speed `0.3`, duration `0.0`, pause `2.0` | |

### `act_coords.py` — read ZED pose (no arming, safe)
Spawns the ZED vslam node, prints WORLD-frame x,y,z from `vslam/odometry`.
Does **not** arm or drive — pure readout. ZED Y_UP frame → **Y is vertical (depth)**.
```bash
python3 act_coords.py             # print at --rate until Ctrl+C
python3 act_coords.py --once      # print one fix and exit
python3 act_coords.py --external  # subscribe to a vslam node you launch
```

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--once` | flag | off | print first fix then exit |
| `--external` | flag | off | don't spawn vslam; subscribe to an existing one |
| `--rate` | float | `2.0` | print rate (Hz) |

---

## 3. Stages (`stage_*.py`)

Multi-step rehearsals of the prequal run.

### `stage_gate.py` — submerge (timed) → through the gate
No vision; descent is purely timed. Rehearse the motion profile / tune
speeds before adding detection.
```bash
python3 stage_gate.py --submerge-speed 0.4 --submerge-duration 4 \
                      --speed 0.4 --duration 5
```

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--submerge-speed` | float | `0.4` | descent effort |
| `--submerge-duration` | float | `4.0` | seconds to descend (timed, no depth sensor) |
| `--speed` / `--duration` | float | `0.4` / `5.0` | forward gate transit |
| _shared_ | | ramp-up/down, pause | |

### `stage_gate_detect.py` — submerge until gate detected → through the gate
Spawns the detector; descends until the gate label is seen on
`vision/detections` or `--gate-timeout` expires, then drives the timed transit.
```bash
python3 stage_gate_detect.py --submerge-speed 0.4 --gate-timeout 25 \
                             --speed 0.4 --duration 5
```

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--label` | str | `gate` | detection label |
| `--submerge-speed` | float | `0.4` | descent effort |
| `--gate-timeout` | float | `25.0` | descend until detected OR N seconds |
| `--conf` | float | `0.5` | min detection confidence |
| `--speed` / `--duration` | float | `0.4` / `5.0` | forward gate transit |

### `stage_marker.py` — submerge (timed) → around the marker
Timed descent, then the open-loop around-marker maneuver
(strafe right → forward → turn left → forward → turn left → forward).
```bash
python3 stage_marker.py --submerge-duration 4 --speed 0.35 \
                        --leg-duration 3 --turn-duration 6
```

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--submerge-speed` | float | `0.4` | descent effort |
| `--submerge-duration` | float | `4.0` | seconds to descend (timed) |
| `--speed` | float | `0.35` | effort for every maneuver leg |
| `--leg-duration` | float | `3.0` | seconds per straight/strafe leg |
| `--turn-duration` | float | `6.0` | seconds per ~90° turn |

### `stage_marker_detect.py` — submerge until marker detected → around the marker
```bash
python3 stage_marker_detect.py --submerge-speed 0.4 --marker-timeout 25 \
                               --speed 0.35 --leg-duration 3 --turn-duration 6
```

| Flag | Type | Default | Meaning |
|------|------|---------|---------|
| `--label` | str | `marker` | detection label |
| `--submerge-speed` | float | `0.4` | descent effort |
| `--marker-timeout` | float | `25.0` | descend until detected OR N seconds |
| `--conf` | float | `0.5` | min detection confidence |
| `--speed` | float | `0.35` | effort for every maneuver leg |
| `--leg-duration` / `--turn-duration` | float | `3.0` / `6.0` | maneuver leg / turn seconds |

---

## 4. `ThrusterController` flight mode

`flight_mode` selects the autopilot mode the driver sets on arm and re-asserts
in the armed-status watchdog.

- **`ALT_HOLD`** (default, ArduSub custom_mode `2`) — autopilot holds depth on
  the **barometer** while horizontal axes stay `manual_control`. Use for normal
  runs (field toolkit, prequal, production driver, `move_forward.py`).
- **`MANUAL`** (custom_mode `19`) — no autopilot depth hold. Use only when an
  external closed loop is the sole depth authority, or on a dry bench with no
  depth sensor (ALT_HOLD may refuse to arm dry).
- `STABILIZE` (`0`) also recognised. Unknown name → falls back to `MANUAL`.

Selection precedence: explicit kwarg → ROS `flight_mode` param → default `ALT_HOLD`.

```python
ThrusterController()                      # ALT_HOLD (default)
ThrusterController(flight_mode='MANUAL')  # external loop owns depth / dry bench
```
```bash
ros2 run mavlink_thruster_control thruster_node --ros-args -p flight_mode:=MANUAL
```

Caller modes set this session:
- ALT_HOLD: field toolkit, prequal, production driver, `move_forward.py`
- MANUAL: `depth_hold_test.py`, `submerge_test.py` (ZED P-controller is sole
  depth authority), `dry_test.py` (no water/depth sensor)

---

## 5. Prequal `depth_hold_source` (`baro` | `zed`)

Config: `src/prequalification/config/prequalification.yaml` → `depth_hold_source`
(default `baro`). Resolves the ALT_HOLD-vs-ZED depth-controller conflict.

| Value | Behaviour | Pair with thruster mode |
|-------|-----------|-------------------------|
| `baro` (default) | Pixhawk **ALT_HOLD** holds depth on the baro; prequal just holds neutral vertical | `flight_mode=ALT_HOLD` |
| `zed` | prequal runs the closed-loop **P-controller** off `sub_depth` (ZED) | `flight_mode=MANUAL` (else it fights ALT_HOLD) |

Unknown value → falls back to `baro`.

```bash
# baro depth hold (default)
ros2 run prequalification prequalification_node \
  --ros-args --params-file src/prequalification/config/prequalification.yaml

# ZED closed-loop depth hold — also set thruster MANUAL
ros2 run prequalification prequalification_node \
  --ros-args -p depth_hold_source:=zed
ros2 run mavlink_thruster_control thruster_node \
  --ros-args -p flight_mode:=MANUAL
```

---

## 6. ZED reconnect resilience (operational note)

`detector.py` and `vslam_node.py` now open the camera with retry + backoff,
tear down and reopen on sustained mid-run grab failures, and handle
SIGINT/SIGTERM/**SIGHUP** to release the camera cleanly.

Practical effect: a busy / briefly-unplugged ZED recovers on rerun without a
power cycle, and an SSH drop (SIGHUP) no longer leaves the camera locked — which
previously caused "camera stream failed" on the next run. No new flags; just
rerun the node.
