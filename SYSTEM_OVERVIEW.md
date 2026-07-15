# RoboSub 2026 — Full System Overview & API Reference

> **Purpose of this file:** complete context handoff. Anyone (or any AI assistant)
> reading this should be able to make deep, correct changes and troubleshoot this
> repo without prior exposure. Last updated **2026-07-14** on branch `heading-lock`.

---

## Table of contents

1. [What this is](#1-what-this-is)
2. [Hardware & environment](#2-hardware--environment)
3. [Repository layout](#3-repository-layout)
4. [System architecture](#4-system-architecture)
5. [The control stack (new API, current)](#5-the-control-stack-new-api-current)
   - [`thruster_node` — MAVLink gateway](#51-thruster_node--the-mavlink-gateway)
   - [`motion_node` — THE movement node](#52-motion_node--the-centralized-movement-node)
   - [Pure controllers under motion_node](#53-pure-controllers)
   - [`Auv` — the operator Python API](#54-the-auv-python-api-controlapi)
   - [`orientation_node`](#55-orientation_node-imu-package)
   - [`rviz_visualizer` & pose sources](#56-rviz_visualizer--pose-sources)
   - [`forward_hold_mission`](#57-forward_hold_mission)
   - [`autonomous_controller` (legacy closed-loop)](#58-autonomous_controller-older-closed-loop-path)
6. [Message & service definitions (`auv_msgs`)](#6-message--service-definitions-auv_msgs)
7. [Vision](#7-vision)
8. [Mission planning — SHRUB v4 behavior tree](#8-mission-planning--shrub-v4-behavior-tree)
9. [Localization & other packages](#9-localization--other-packages)
10. [Launch files & how to run things](#10-launch-files--how-to-run-things)
11. [Root-level scripts](#11-root-level-scripts)
12. [Build, test, and tuning workflow](#12-build-test-and-tuning-workflow)
13. [Topic/service/param quick-reference tables](#13-quick-reference-tables)
14. [Hard-won gotchas & troubleshooting](#14-hard-won-gotchas--troubleshooting)
15. [The attic (archived legacy toolkit)](#15-the-attic-archived-legacy-toolkit)

---

## 1. What this is

Autonomous underwater vehicle (AUV) software for the **RoboSub 2026** competition
(Team IntelligentLion). ROS 2 **Humble** workspace on an NVIDIA **Jetson** (Tegra,
L4T kernel 5.15.148). The mission planner is **SHRUB v4** ("Software for Handling
and Regulating Underwater Behavior") — a BehaviorTree.CPP v4 tree — sitting on top
of a Python control/vision stack that talks MAVLink to a **Pixhawk running
ArduSub** (4.5.x, params backed up in `pixhawk_params_4.5.7_backup_2026-07-08.param`).

Two generations of control code coexist:

- **The current "new ROS control API"** (branch `heading-lock`, commit
  `15e45647` migrated to it): `motion_node` + `control.api.Auv` + `thruster_node`
  as gateway. **All new work happens here.**
- **The legacy field toolkit** (`field_common.py`, `act_*.py`, `depth_hold*.py`,
  `motor_test.py`, etc.) — archived to `attic/` on 2026-07-14. Check `attic/`
  before assuming a script is gone; do not resurrect them into the live tree.

Season constraints: **no hydrophones** (pinger tasks removed; torpedo/octagon are
vision-only), **no DVL**, **VSLAM intentionally disabled for competition** (see
`src/localization/launch/vslam_localization_launch.py` for rationale).

## 2. Hardware & environment

| Item | Detail |
|---|---|
| Compute | NVIDIA Jetson (aarch64, Tegra). GPU inference via native TensorRT + `cuda-python` 12.6 (~37 fps). `numpy<2` required in the vision env. |
| Flight controller | Pixhawk, **ArduSub** (~4.5.7 params backup; 4.7.0-beta7 fixed a Bar02 scaling bug). Serial `/dev/ttyACM0` @ 115200. |
| Frame | Vectored 8-thruster BlueROV-style: motors 1–4 horizontal, 5–8 vertical. Known-good `MOT_x_DIRECTION`/`SERVOx_*` values live in `thruster_params.py` (copied from the .param backup). |
| Depth sensor | **Bar02** external pressure sensor on I2C → MAVLink `SCALED_PRESSURE2` (occasionally 3). **Connection is intermittent** — see gotchas. The FMU's internal baro (`SCALED_PRESSURE`) is sealed in the hull and is NEVER a depth source. |
| Cameras | Two **ZED 2i**: front (serial **31166146**, model `ffc_rs_26`) and bottom (serial **30758628**, model `dfc_rs_26`). ZED USB3 noise **jams 2.4 GHz WiFi** — tether SSH on 5 GHz or ethernet. |
| IMU | Pixhawk IMU is the yaw source for the heading lock (`pixhawk/imu/data` → `orientation_node` → `imu/rpy`). ZED IMU also exists. `EK3_SRC1_YAW=0` (gyro-only) since 2026-07-09 — hard-iron distortion made compass yaw chase (~1.5°/s uncommanded turn in ALT_HOLD). |
| Battery | 2× Blue Robotics 14.8 V / 10 Ah LiPo in parallel (4S, 20 Ah). Voltage→% curve in `safety_monitor_node`. |
| Actuators | Marker dropper on SERVO9 (`SERVO9_FUNCTION`: ArduSub 4.5 re-saves 184 every boot; set to 0 per-run, don't reboot; `DO_SET_ACTUATOR` unsupported). Manipulation drivers are **not yet implemented** (see `src/robosub2026/MANIPULATION_DRIVERS.md`). |
| Python env | `venv/` at repo root for the vision stack (`setup_vision_env.sh`). ROS nodes use system Python. `py_trees` needed for `bt_coinflip.py`. |

## 3. Repository layout

```
robosub-2026/
├── src/                          ROS 2 workspace sources
│   ├── control/                  ★ the new control API (motion_node, Auv, launch files)
│   ├── mavlink_thruster_control/ ★ thruster_node (MAVLink gateway), safety_monitor_node
│   ├── auv_msgs/                 custom messages + SetFlightMode service
│   ├── imu/                      orientation_node (+ diagnostics/marker viz nodes)
│   ├── pix_imu/                  pixhawk_imu_bridge (dry-bench ONLY — see gotchas)
│   ├── vision/                   detector (front ZED), bottom_camera_node, models
│   ├── localization/             localization_node, depth_node, drift_correction_node
│   ├── prequalification/         standalone prequal mission state machine
│   ├── robosub2026/              ★ SHRUB v4 BT (package `bt_mission`, exe `bt_executor`)
│   ├── BehaviorTree.ROS2         vendored BT↔ROS2 integration
│   ├── zed-ros2-wrapper/, zed-ros2-examples/   vendored ZED ROS packages
│   ├── run_stack.sh              legacy full-stack launcher (autonomous_controller path)
│   └── test_movement.sh
├── tests/                        ★ offline pytest suite (~153 tests, no hardware)
├── attic/                        archived pre-new-API toolkit (read-only reference)
├── docs/                         CENTERING.md, testing-submerge-hold.md, rules txt, audit
├── bt_coinflip.py                ★ coin-flip task tree (py_trees, uses Auv)
├── auto_tune_pid.py              ★ heading-PID auto-tuner (sim + live modes)
├── movement_test.py              minimal Auv exercise script
├── depth_and_forward.py          minimal Auv exercise script
├── build_engine.py / convert_to_onnx.py / deploy_model.sh   model deployment
├── setup_vision_env.sh / reset_zed_node.sh
├── pixhawk_params_4.5.7_backup_2026-07-08.param   ★ ground truth for Pixhawk params
├── pytest.ini                    pytest config (tests/ dir)
├── JETSON_SETUP.md, COMMAND_USAGE.md (legacy toolkit doc), README.md (SHRUB overview)
└── *.log, log_archive/, rosbag2_*   field-test artifacts
```

★ = the parts you will touch most.

## 4. System architecture

```
                          ┌────────────────────────────────────────────┐
                          │ Mission layer (pick ONE driver at a time)  │
                          │  • bt_mission/bt_executor (SHRUB v4 C++)   │
                          │  • bt_coinflip.py (py_trees, uses Auv)     │
                          │  • forward_hold_mission (one-shot run)     │
                          │  • your script using control.api.Auv       │
                          └───────────────┬────────────────────────────┘
                                          │ motion/cmd, motion/submerge,
                                          │ motion/heading   (intent topics)
                                          ▼
   vision/detections ───►  ┌──────────────────────────────┐
   (front detector)        │ motion_node  (control pkg)   │  SOLE publisher of
   imu/rpy ──────────────► │  SubmergeController          │  movement_command.
   (orientation_node)      │   └─ DepthController         │  Self-inhibits if any
   pixhawk/depth ────────► │  HeadingLock (PID on yaw)    │  other publisher on
   pixhawk/mode, armed ──► │  MotionController (mixer)    │  that topic exists.
                           └──────────────┬───────────────┘
                                          │ movement_command (MovementCommand)
                                          ▼
                           ┌──────────────────────────────┐
                           │ thruster_node                │  SOLE serial owner,
                           │ (mavlink_thruster_control)   │  sole pymavlink user.
                           │  MANUAL_CONTROL @10Hz, arm,  │  services: set_mode,
                           │  heartbeat, watchdog,        │  preflight, disarm
                           │  Bar02→depth, mode/armed pub │
                           └──────────────┬───────────────┘
                                          │ MAVLink over /dev/ttyACM0
                                          ▼
                                   Pixhawk / ArduSub  (ALT_HOLD holds depth,
                                                       self-levels roll/pitch)
```

**Division of labour — the guiding rule** (ArduSub keeps loops it's already good at):

| Axis | Owner |
|---|---|
| depth | **ArduSub ALT_HOLD** (on the Bar02). No custom depth PID. The dive itself (getting *to* depth) is `DepthController`'s only job. |
| roll, pitch | **ArduSub** (ALT_HOLD self-levels). Never commanded by the stack. |
| heading (yaw) | **Us** — `HeadingLock` PID in motion_node. ArduSub has no yaw-heading-hold mode; this is the one genuine gap. |
| surge, strafe | The operator/mission. The only axes clients touch. |

**Two single-owner invariants, enforced in code:**

1. **One serial reader/writer**: `thruster_node` is the only process that opens
   the Pixhawk port and the only one importing pymavlink at runtime. Two readers
   on one port → `device reports readiness to read but returned no data` stall,
   both die. Never run `pix_imu/pixhawk_imu_bridge` alongside `thruster_node`.
2. **One `movement_command` publisher**: `motion_node` counts publishers on the
   topic every 2 s and **inhibits itself** (stops, refuses submerge) if it isn't
   alone. `autonomous_controller`, `prequalification_node` and the attic scripts
   also publish that topic — stop them before running motion_node, or vice versa.

## 5. The control stack (new API, current)

### 5.1 `thruster_node` — the MAVLink gateway

`src/mavlink_thruster_control/mavlink_thruster_control/thruster_node.py` (~1140 lines).
Run: `ros2 run mavlink_thruster_control thruster_node`.

Translates `MovementCommand` into MAVLink `MANUAL_CONTROL` at **10 Hz**, sends a
1 Hz heartbeat (prevents GCS failsafe disarm), auto-detects the serial port
(configured port first, then scans `/dev/ttyACM*`, `/dev/ttyUSB*`), runs in
**simulation mode** if no hardware (or `simulate:=true`), auto-reconnects and
re-arms on serial errors/unexpected disarms.

**Parameters**

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | tried first, then auto-scan |
| `baud_rate` | 115200 | |
| `simulate` | false | no hardware; node still runs so stack is testable |
| `watchdog_timeout` | 4 s | no new command → auto-stop (neutral axes) |
| `disarm_watchdog_timeout` | 60 s | continued drought → disarm |
| `flight_mode` | `ALT_HOLD` | mode requested at startup/arm. MANUAL only for ZED-depth/dry work |
| `water_density` | 1000.0 | kg/m³ for Bar02 depth conversion (pool = fresh) |

**Subscribes:** `movement_command` (MovementCommand), plus internal.
**Publishes:** `pixhawk/imu/data` (sensor_msgs/Imu), `pixhawk/depth` (Float32,
metres +down, **NaN when no external baro**), `pixhawk/mode` (String — the mode
the vehicle is *actually* in, read back from HEARTBEAT), `pixhawk/armed` (Bool).

**Services:**

- `pixhawk/set_mode` (`auv_msgs/SetFlightMode`) — sends set_mode then **waits for
  HEARTBEAT.custom_mode to read back** the requested mode. `success=false` +
  ArduSub's STATUSTEXT reason (e.g. `"Depth sensor is not connected."`) if it
  didn't take. Callers must treat failure as a hard abort.
- `pixhawk/preflight` (`std_srvs/Trigger`) — **read-only** comparison of live
  `MOT_x_DIRECTION` / `SERVOx_{FUNCTION,REVERSED,TRIM,MIN,MAX}` for all 8 motors
  against the known-good values in `thruster_params.py`. **Fails closed** (an
  unreadable param is a failure). No bypass flag, by design — flipped directions
  have twice caused a forward command to spin the sub / an up-thrust on submerge.
- `pixhawk/disarm` (`std_srvs/Trigger`).

Depth pipeline (`pressure.py`, pure functions): picks `SCALED_PRESSURE2`→`3`
(never falls back to hull baro — no depth is a safe abort, wrong depth is not),
latches the surface reference as the **median** of surface samples, sanity-checks
it's 900–1100 hPa, then `depth = (P - P_surface)*100 / (ρ·g)`. Negative depth is
not clamped (a persistently negative depth exposes a bad surface latch).

`mavlink_compat.install_add_message_guard()` **must** run before any
`mavlink_connection()` — pymavlink 2.4.49 bug where a MAVLink1 packet poisons the
message cache and a later MAVLink2 packet of the same type raises TypeError,
silently killing the whole receive path (symptom: node connects, looks healthy,
never sees another message / "SIMULATION mode"). Copies of the guard exist in
`attic/field_common.py` and `pix_imu/pixhawk_imu_bridge.py`.

#### `safety_monitor_node` (same package)

Publishes `/safety/battery_pct` (Float32 0–100, NaN unknown) and
`/safety/leak_detected` (Bool) at 2 Hz. **Defaults to `simulate:=true`** (100 %,
no leak) so the BT's CriticalFailure branch only fires for real on the sub.
Battery: MAVLink `SYS_STATUS.battery_remaining`, falling back to a voltage→%
curve for the actual pack when ArduSub reports −1 (16.4 V→100 %, 14.8 V→50 %,
14.0 V→20 % trip, 12.0 V→0 %). Leak: `_read_leak_gpio()` stub (no hardware yet).
Params: `simulate`, `serial_port`, `baud_rate`, `udp_endpoint` (consume a
mavproxy/mavlink-router forward instead of owning a port), `leak_gpio_chip/line`.
On real hardware either give it a UDP forward or make it the single MAVLink
owner — never a second reader on thruster_node's port.

### 5.2 `motion_node` — THE centralized movement node

`src/control/control/motion_node.py`. Run:
`ros2 run control motion_node` (normally via `submerge_hold.launch.py`).

Sole publisher of `movement_command`. Everything that wants the sub to move goes
through its intent topics. Replaces the old `heading_lock_node` (retired).

**Subscribes**

| Topic | Type | Meaning |
|---|---|---|
| `imu/rpy` (param `yaw_topic`) | Vector3Stamped | `vector.z` = yaw, radians, REP-103 CCW+ |
| `pixhawk/depth` | Float32 | metres +down, NaN when unavailable |
| `pixhawk/mode` | String | actual vehicle mode |
| `pixhawk/armed` | Bool | |
| `motion/cmd` | MovementCommand | **operator intent: surge/strafe (+ deliberate yaw_rate)**. heave/roll/pitch ignored with a warning. `command='stop'` zeroes intent. `yaw_rate != 0` = deliberate turn (lock stands down, re-captures each tick so release holds the NEW heading); `yaw_rate == 0` = lock steers. |
| `motion/submerge` | Float32 | target depth m. **> 0** starts the submerge sequence; **≤ 0** aborts + releases + acknowledges a latched duty abort. |
| `motion/heading` | Float32 | **absolute** heading target (rad, REP-103 CCW+). Honoured only while in HOLD; a deliberate slew for the lock (used by `auto_tune_pid.py --live` step response). |

**Publishes:** `movement_command` (`axes` while active, `stop` on abort/idle),
`submerge/state` (String: `idle|preflight|mode_set|arming|diving|hold|failed: <reason>`),
and debug Float32 topics for rqt_plot/RViz:
`heading/{current,target,error,yaw_correction}`, `depth/{current,target}`,
`motion/{forward_cmd,vertical_cmd}`.

**Service clients:** `pixhawk/preflight`, `pixhawk/set_mode` (request/poll
pattern — never blocks inside the timer callback; blocking on a service future
in a single-threaded executor deadlocks).

**Parameters** (all in `PARAM_DEFAULTS`; all live-tunable via `ros2 param set`
**except** `control_rate_hz` and `yaw_topic`, which are rejected live because
they only apply at construction):

| Param | Default | Meaning |
|---|---|---|
| `target_depth` | 2.0 | default dive target (launch arg) |
| `dive_speed` | 0.3 | descent heave magnitude — the descent rate lives HERE, not in the Auv API |
| `depth_tolerance_m` | 0.15 | dive "arrived" tolerance |
| `min_heave` | 0.12 | floor on heave so a small dive_speed doesn't land inside ArduSub's throttle deadzone (±THR_DZ ≈ ±0.1) and become no command at all |
| `depth_timeout` | 30.0 | dive must reach target within this |
| `phase_timeout_s` | 15.0 | per-phase timeout for preflight/mode/arm |
| `heading_kp/ki/kd` | 1.2 / 0.0 / 0.3 | heading-lock PID gains |
| `i_limit` | 0.3 | PID integral clamp |
| `max_yaw_correction` | 0.4 | clamp on lock's yaw output (SAFETY: 0 or negative would invert the clamp into constant full-authority spin) |
| `max_forward_speed` | 0.6 | surge clamp in the mixer |
| `stale_timeout_s` | 0.5 | yaw sample older than this = stale |
| `grace_s` | 1.0 | blind-forward grace after yaw goes stale, then stop |
| `depth_stale_timeout_s` | 2.0 | depth older than this = lost → stop movement |
| `stale_window_s` / `stale_duty_abort` | 3.0 / 0.5 | duty-cycle abort: if > 50 % of ticks in the trailing window had stale yaw, abort and **latch** (catches a source that keeps dropping *under* stale_timeout_s — brownout pattern — which never trips the plain grace abort). Latched until acknowledged with `motion/submerge ≤ 0`. |
| `control_rate_hz` | 20.0 | restart-only |
| `yaw_topic` | `imu/rpy` | restart-only |

`PARAM_BOUNDS` validates every live set — whole batch rejected on first
violation (no partial application). The bounds are **safety limits, not taste**;
the block comment in `motion_node.py` is the canonical rationale (e.g.
`stale_duty_abort` has a *strict* upper bound < 1.0 because the test is
`fraction > threshold` and fraction ≤ 1.0, so exactly 1.0 makes the abort
unreachable).

**Safety responses (all in `_tick`, deliberately different per loss):**

| Loss | Response |
|---|---|
| preflight mismatch | abort before arming, dry |
| ALT_HOLD refused | abort **on the surface**, zero thrust commanded |
| yaw stale | correction → 0 immediately (never steer blind); forward continues `grace_s`, then stop. Depth hold untouched. |
| yaw degraded (duty) | stop + unlock, **latched** until operator ack |
| depth stale/NaN | stop movement; stay in ALT_HOLD (autopilot may still hold). Never dive. |
| mode leaves ALT_HOLD | depth hold is GONE → stop movement |
| tick exception | stop + unlock |

### 5.3 Pure controllers

All ROS-free, time-injected (no sleeps/clock reads), pinned by `tests/`:

- **`submerge.py` — `SubmergeController`**: state machine
  `IDLE → PREFLIGHT → MODE_SET(ALT_HOLD) → ARMING → DIVING → HOLD`, any failure
  → `FAILED` with reason. Two load-bearing orderings: preflight **before**
  anything (flipped MOT direction check must happen dry) and ALT_HOLD
  **confirmed before the first centimetre of heave** (ArduSub silently refuses
  ALT_HOLD with a dead depth sensor — diving first would mean descending with no
  depth hold). ARMING *waits* for armed; **arming itself belongs to
  thruster_node** (single arming authority). Side effects go through the
  `Effects` protocol (request/result split, poll-friendly). On reaching depth,
  captures the current yaw into the heading lock; reaching depth with no yaw
  source is a **failure**, not HOLD.
- **`depth_controller.py` — `DepthController`**: performs the DIVE only; it is
  *not* a depth PID (ALT_HOLD holds depth better than we could). Commands
  constant `dive_speed` heave (floored at `min_heave`, sign-forgiving) until
  within `tolerance_m`, then returns 0 heave forever (AT_DEPTH latches — no
  tug-of-war with ALT_HOLD on overshoot). Dropout → 0 heave (`NO_DEPTH_DATA`);
  timeout keeps running through dropouts so a dead sensor can't hold the dive
  open.
- **`heading_lock.py` — `HeadingLock`**: capture yaw at start, PID it straight.
  Sign contract (pinned by `tests/test_heading_lock.py`): input yaw REP-103
  CCW+; output yaw_rate MovementCommand CW+; `error = wrap(current − target)`
  and the convention flip is folded into the error sign, so PID output IS the
  yaw_rate command. `set_target()` = deliberate absolute slew (PID reset for a
  clean step; no-op unless LOCKED/STALE_GRACE). Stale: correction 0 now, forward
  rides `grace_s`, then ABORTED (latches until `stop()`); recovery within grace
  resumes against the ORIGINAL target.
- **`motion.py` — `MotionController`**: the axis mixer and the authority
  enforcement point. Operator gets surge/strafe; yaw comes from the lock UNLESS
  `operator_yaw` is not None (any number **including 0.0** overrides — an
  explicit command must not be fought by the lock); heave only from the dive;
  roll/pitch have **no field** so they can't be commanded by accident. All axes
  NaN-scrubbed and clamped to [-1, 1] (a NaN reaching MANUAL_CONTROL is
  undefined behaviour at the autopilot).
- **`pid.py` — `PID`**: shared PID; output clamp `limit`, integral clamp
  `i_limit`, derivative clamp ±10, non-finite input/output → reset + 0.
  `set_gains()` supports live tuning.

### 5.4 The `Auv` Python API (`control.api`)

The operator façade — used identically by mission scripts, BT nodes, and
interactive pool work. **Holds no MAVLink and no control state**: publishes
intent to motion_node, watches `submerge/state`. Blocking calls; pumps rclpy
internally. Speeds are normalized 0.0–1.0.

```python
from control.api import Auv, SubmergeError

with Auv() as auv:                      # owns its own rclpy node ('auv_api')
    auv.submerge_to_depth(2.0)          # blocks until 'hold'; raises SubmergeError
    auv.move_forward(speed=0.4, duration=10)   # depth+heading+attitude auto-held
    auv.turn(yaw_rate=1.0, degrees=90)  # closed-loop off imu/rpy; sign = direction
    auv.turn(yaw_rate=-0.3, duration=6) # open-loop timed alternative
    auv.move_left(0.35, 3); auv.move_right(0.35, 3); auv.move_backward(0.3, 2)
    auv.stop()                          # stop translating; depth+heading hold stay on
    auv.surface()                       # RELEASES the dive (motion/submerge 0) —
                                        # does NOT ascend; sub stays in ALT_HOLD at depth.
                                        # Disarm or command ascent to actually come up.
```

Details that matter:

- `Auv(node=...)` can share an existing rclpy node (e.g. inside a py_trees tick).
- `submerge_to_depth(target_depth, timeout=60)` first **waits for motion_node to
  be subscribed** to `motion/submerge` (a Float32 published into the void is
  silently lost), then publishes and polls `submerge/state` until `hold` /
  `failed: …` / timeout. Descent rate is motion_node's `dive_speed` param, not
  an argument here.
- `move_*` **re-publishes axes every 0.1 s** (motion_node holds last intent; the
  stream keeps it honest on drops) and raises `SubmergeError` mid-move if state
  goes `failed`.
- `turn(degrees=...)` unwraps ±π rollover per step and counts progress **in the
  commanded direction** (jitter cancels instead of inflating). Needs `imu/rpy`.
- `state` property: latest `submerge/state` string.
- Context manager guarantees `stop()` + node cleanup even on exception.

### 5.5 `orientation_node` (imu package)

`ros2 run imu orientation_node`. Subscribes an Imu topic (param `imu_topic`,
default the ZED; the launch files point it at `/pixhawk/imu/data`), averages the
first `calib_samples` quaternions as a startup reference, publishes orientation
RELATIVE to it (sub reads level/identity at launch, no cross-run accumulation).
Publishes **`imu/rpy` (Vector3Stamped — the yaw source for motion_node)** and,
when `publish_tf:=true`, the odom→base_link TF. Service
`/imu/reset_orientation` (Trigger) re-zeroes live. In `submerge_hold.launch.py`
its `publish_tf` is **false** because `rviz_visualizer` owns odom→base_link
there (it has the dead-reckoned translation; two broadcasters on one TF edge
fight). Also in the package: `diagnostics_node`, `marker_node` (viz helpers),
`imu_math.py` (pure quaternion helpers, tested).

### 5.6 `rviz_visualizer` & pose sources

`src/control/control/rviz_visualizer.py` — **subscribe-only**; reads the debug
topics motion_node already publishes, never publishes `movement_command`. Omit
freely. Publishes `viz/markers` (heading arrows: blue=current, yellow=desired,
red=error arc, green=surge, magenta=yaw correction, cyan=heave; depth plane;
text), `viz/path`, and TF map→odom→base_link. `viz/target_waypoint`
(PointStamped) is an input hook nothing publishes yet.

**Position honesty** (`pose_source.py`): a Bar02 + IMU **cannot measure XY**.
Default `pose_source: pixhawk_imu` = `DeadReckonPose`, integrating **commanded**
velocity through measured heading (`surge_scale=0.5`, `strafe_scale=0.4` m/s per
unit — guesses). Path drawn **orange** + "POSITION ESTIMATED" text; it shows the
*shape* ("did the lock keep us straight?"), not location. Depth is measured and
exact. `pose_source: zed` = `ZedOdomPose` relaying `vslam/odometry` (green,
measured). A DVL/EKF drops in as a third `PoseSource` with no controller change.
**Station-keeping is not implemented and not possible with these sensors.**

### 5.7 `forward_hold_mission`

`ros2 run control forward_hold_mission` / `move_forward_depth_hold.launch.py`.
The one-command run: dive, hold, drive forward. Owns no control law — gates on
the stack being *genuinely* alive, then drives `Auv`. Readiness gates (each
timeout → nonzero exit → launch shuts the whole stack down): (1) pixhawk topic
publishers exist, (2) a real MAVLink mode string arrived, (3) preflight+set_mode
services ready, (4) **5 consecutive finite `pixhawk/depth` samples** (~0.5 s —
thruster_node publishes NaN without depth, so a ticking topic proves nothing and
the intermittent Bar02 loves one lucky sample), (5) motion_node subscribed to
`motion/submerge`. Params: `target_depth` 2.0, `forward_speed` 0.4,
`forward_duration` 10, `startup_timeout` 30, `dive_timeout` 60,
`surface_on_finish` true.

### 5.8 `autonomous_controller` (older closed-loop path)

`src/control/control/autonomous_controller.py` — the pre-motion_node
closed-loop controller, still used by `src/run_stack.sh` + SHRUB's
`NavigationCommand` path. Subscribes `navigation_command`, `vision/detections`,
`localization/pose`, `depth/sub_depth`; **publishes `movement_command`
directly** — so it and motion_node are mutually exclusive (sole-publisher
guard). Modes: `idle`, `station_keep`, `track_object` (vision centering +
approach), `search` (rotate until found → track), `waypoint`, `heading_hold`.
Centering logic lives in `centering.py`: `TargetState` (frame-agnostic snapshot,
image-space + metric from ZED `position.z` slant range), `TargetTracker` (EMA +
coast through dropped frames), `CenteringPolicy` per task (`GatePolicy`
implemented; others default-stubbed). See `docs/CENTERING.md`.

## 6. Message & service definitions (`auv_msgs`)

### `MovementCommand.msg` — the low-level thruster interface

```
string  command      # verb, or 'axes', or 'stop'
float32 speed        # 0.0–1.0 (verb mode)
float32 duration     # seconds (0 = until next command; verb mode)
float32 surge        # -1..1  +forward      (axes mode)
float32 strafe       # -1..1  +right
float32 heave        # -1..1  +DOWN  (0 = hold depth in ALT_HOLD)
float32 yaw_rate     # -1..1  +clockwise
float32 pitch_rate   # -1..1  +nose-up   ┐ MAVLink2 MANUAL_CONTROL extensions;
float32 roll_rate    # -1..1  +right-down┘ need FRAME_CONFIG=Vectored-6DOF, else ignored
```

Verbs: `submerge, emerge, surge_forward, surge_backward, strafe_left,
strafe_right, rotate_cw, rotate_ccw, stop, depth_hold`. Verb mode is the
open-loop/scripted path; `axes` is the simultaneous 4-axis setpoint used by
motion_node and track_object centering.

**Sign conventions (memorize):** +surge forward, +strafe right, **+heave DOWN**,
+yaw_rate **clockwise** (MovementCommand) vs **imu/rpy yaw CCW+** (REP-103).
HeadingLock folds the flip into its error sign.

### `NavigationCommand.msg` — high-level goals for autonomous_controller

`mode` (`idle|station_keep|track_object|search|waypoint|heading_hold`),
`target_label`, `target_x/y/z` (m, z +down), `target_yaw` (rad), `speed` (0–1),
`approach_dist` (m, 0 = default).

### Others

- `ObjectDetection.msg`: `label`, `confidence`, `position` (geometry_msgs/Point —
  **`position.z` is ZED slant range in metres**), `bbox_width`, `bbox_height`
  (normalized). `ObjectDetectionArray.msg`: `detections[]`.
- `DepthInfo.msg`: `stamp`, `sub_depth_m`, `stop_distance_m`.
- `BehaviorStatus.msg`: `stamp`, `action_name`, `status`, `reason`.
- `SetFlightMode.srv`: request `mode` (`MANUAL|STABILIZE|ALT_HOLD|ACRO`) →
  `success` (true only if custom_mode **read back**), `reason` (ArduSub
  STATUSTEXT on failure).

## 7. Vision

Two ZED 2i cameras, two nodes, both in `src/vision/vision/`:

### `detector` (front camera) — `ros2 run vision detector`

TensorRT (native, GPU, ~37 fps) with ONNX-runtime CPU fallback. Defaults resolve
`ffc_rs_26.onnx` / `.engine` next to the module (symlinked into the build dir) —
**no `--onnx` flag needed** for the current model.

CLI (after `--`): `--weights/--onnx` (default ffc_rs_26), `--classes`, `--svo`
(playback), `--serial` (default **31166146** = front), `--img_size` 416,
`--conf_thres` 0.4, `--device` cuda, `--zed_fps` 60 (mission_stack halves to 30),
`--view` (OpenCV window), `--save_frames DIR` (diagnose channel/letterbox
issues), plus `--twod_only`-style knobs on the bottom node.

Publishes: `vision/detections` (ObjectDetectionArray), `depth/sub_depth`
(Float32 — from ZED), `vslam/odometry` (Odometry — ZED positional tracking).
ZED SDK does custom-box 3D object tracking, so each detection carries a real
slant range in `position.z`.

**Front model labels (`ffc_rs_26`)**: there is **no `gate` class** — the gate is
identified by role images hung on it: `compass`, `hammer_and_wrench` (Survey
side) / `buoy`, `sos` (Rescue side). Slalom uses **one label `slalom`** for all
pipes (middle-of-3 = red; `SlalomMonitor` in the old task handled multiplicity —
`fc.DetectionMonitor` keeps only 1 det/label). The SHRUB tree's label vocabulary
(section 8) is broader than what the model emits; unmatched `Detect_*` nodes
return SUCCESS (smoke-test friendly) — check MIGRATION.md.

**ZED frame gotchas**: frames are **BGRA** (a 2026-07-10 fix ended a
channel-swap + letterbox squash that silently wrecked accuracy; `--save_frames`
diagnoses). Both `--conf_thres` and `--conf` existed on old tools — both matter.

### `bottom_camera` — `ros2 run vision bottom_camera`

Serial **30758628**. Detects floor path markers with the `dfc_rs_26` model.
Publishes `vision/path_markers` (ObjectDetectionArray), and — only when running
full 3D — `odom/bottom` (ZED VIO) and `depth/sub_depth`. In
`mission_stack.launch.py` it runs **2D-only** (`--twod_only`): no depth engine,
no VIO, ~half the GPU — and it removes the `depth/sub_depth` double-publisher,
leaving the front camera sole publisher. `bottom_twod:=false` restores VIO only
if tegrastats shows headroom.

### Model deployment

New `.pt` → `convert_to_onnx.py` → copy via `deploy_model.sh` → rebuild the
TensorRT engine **on-device** with `trtexec` (`build_engine.py`) → **rebuild the
vision package** (engines/onnx resolve in the build dir). Env: `numpy<2`,
cuda-python 12.6; `setup_vision_env.sh` builds the venv. `test_vision.py`,
`test_pipeline.py` at root are ad-hoc checks.

## 8. Mission planning — SHRUB v4 behavior tree

`src/robosub2026/` — package **`bt_mission`**, executable **`bt_executor`**,
C++17, BehaviorTree.CPP **v4** + `behaviortree_ros2`. The 2026 "Restore and
Recovery" tree is `bt_xml/robosub2026_mission.xml` (Groot2 format, root
`MainTree`, ~50 subtrees, 39 conditions, ~130 action node types).
**Read `src/robosub2026/MIGRATION.md` — it is the canonical contract doc** for
the blackboard, label vocabulary, and known gaps. Summary:

- `include/bt_mission/shrub_nodes.hpp` — every node declared via macros
  (`SHRUB_SYNC`, `SHRUB_COND`, `SHRUB_STATEFUL`); stateful actions share a
  `TimedAction` deadline base.
- `src/safety_nodes.cpp` (39 conditions), `nav_nodes.cpp` (init + movement,
  drives `MovementCommand` or `NavigationCommand` via MissionIO),
  `perception_nodes.cpp` (`Detect_*`/`Search*` wrap
  `MissionIO::bestDetection(label, conf)`), `manipulation_nodes.cpp` (stubs
  updating blackboard counters), `task_logic_nodes.cpp`
  (**`registerAllNodes()` — the single place new node types are registered**),
  `bt_executor.cpp` (seeds blackboard, pushes live depth each 50 ms tick),
  `mission_io.cpp` (`shrub::MissionIO` process-wide singleton).

```cpp
MissionIO::get().sendMovement("surge_forward", 0.4, 2.0);        // open-loop
MissionIO::get().sendNav("track_object", "gate", 0.4, 1.0);      // closed-loop
MissionIO::get().sendNav("heading_hold", "", 0.3, 0.0, 1.57);    // target_yaw
shrub::Detection d;
MissionIO::get().bestDetection("gate", 0.5, d);
```

- Launch: `ros2 launch bt_mission shrub.launch.py` with args `coin_flip`
  (`normal`), `role` (`survey_repair`), `gate_red_side` (`right`),
  `style_enabled` (`true`).
- Blackboard keys (full table in MIGRATION.md): counters
  `markers_remaining`(2), `torpedoes_remaining`(2), `objects_delivered`(0);
  flags `marker_in_bin`, `torpedo_hit`, `inside_octagon`, `critical_failure`,
  `mission_complete`…; live `depth`, `battery_pct`, `leak_detected` (via
  safety_monitor → MissionIO; battery below `battery_critical_pct` (15 %) or
  leak trips `critical_failure` → `GlobalRecovery`/`SurfaceSafely`).
- Tree label vocabulary: `gate, role_sign, survey_repair, search_rescue,
  orange_path, slalom_pole, slalom_gap, path_marker, pipeline, fire_bin,
  blood_bin, bin1, bin2, marker, magnetic_target, target_board, large_opening,
  small_opening, octagon, basket, repair_object, medical_object`. Labels with no
  real publisher → `Detect_*` returns SUCCESS so the tree still ticks.
- **Known gaps** (MIGRATION.md): Roll90/Pitch90 only log (MovementCommand now
  HAS roll/pitch axes — wiring them is open work); manipulation drivers are
  stubs (`MANIPULATION_DRIVERS.md` has the designs — small nodes behind
  `std_srvs/Trigger`); `altitude_m` has no publisher (no DVL/altimeter);
  battery/leak real-hardware wiring documented but defaults simulate.
- **Competition rules context** (docs/, README): time bonus needs touch-a-buoy +
  (marker in bin OR torpedo through opening) + surface inside floating
  structure; 100 pts/minute remaining. Style: 90° increments, reversals don't
  score, roll/pitch > yaw. `bt_coinflip.py` defaults roll:720, pitch:360,
  yaw:720.

## 9. Localization & other packages

- **`localization_node`** — fuses `odom/bottom` (ZED VIO), `localization/correction`
  (PoseStamped from path markers), `depth/info` → publishes `localization/pose`
  (PoseStamped, 10 Hz). Sanity clamps: |position| ≤ 50 m, offsets ≤ 10 m.
- **`depth_node`** — subscribes `vision/detections` + `depth/sub_depth` →
  publishes `depth/info` (DepthInfo, 10 Hz) with the mission stop distance
  (`--stop_distance`, default 1.5 m).
- **`drift_correction_node`** — path-marker-based drift corrections.
- **VSLAM is disabled for competition** — `src/localization/launch/vslam_localization_launch.py`
  documents why (unreliable localization → reactive centering beats a bad map).
- **`prequalification`** — self-contained prequal state machine
  (`prequalification_node`): 14-step vision-triggered run (submerge until gate
  seen → duck under top bar → through gate → around the vertical marker → back
  through gate → resurface), every state has timed/spatial fallbacks so it never
  stalls. Publishes `movement_command` **directly** (mutually exclusive with
  motion_node). `prequal_dry_test_node` for bench tests. Had a
  `depth_hold_source` param (`baro|zed`).
- **`pix_imu/pixhawk_imu_bridge`** — standalone pymavlink→`/pixhawk/imu/data`
  bridge for **dry-bench IMU work only, with thruster_node NOT running** (port
  conflict otherwise). Has its own copy of the pymavlink guard.
- **`imu`** — see 5.5.
- **BehaviorTree.ROS2**, **zed-ros2-wrapper/examples** — vendored dependencies.

## 10. Launch files & how to run things

### The current stack (new API)

```bash
# dive + hold + drive stack (thruster_node, orientation_node, motion_node, viz):
ros2 launch control submerge_hold.launch.py            # args: viz, rviz, serial_port,
                                                       #   simulate, target_depth, pose_source
# everything incl. BOTH cameras:
ros2 launch control mission_stack.launch.py            # + front_fps, bottom_twod, enable_bottom
# then drive it:
python3 bt_coinflip.py --depth-ft 3 --yes              # coin-flip task
# or raw:
ros2 topic pub --once /motion/submerge std_msgs/Float32 '{data: 2.0}'
# or one-shot mission:
ros2 launch control move_forward_depth_hold.launch.py  # forward_hold_mission on top
```

### Legacy full stack (autonomous_controller + SHRUB path)

```bash
./src/run_stack.sh /path/model.pt [conf_thres] [img_size] [device] [stop_dist]
./src/run_stack.sh --onnx /path/model.onnx ...
```

Starts thruster_node → safety_monitor → localization_node →
autonomous_controller → bt_executor → detector → depth_node. Falls back to
simulate mode with no serial device. Note this path and `submerge_hold` are
**mutually exclusive** (movement_command single-publisher).

RViz config: `src/control/rviz/submerge_hold.rviz` (`rviz:=true`).

## 11. Root-level scripts

| Script | What |
|---|---|
| `bt_coinflip.py` | Task 0 tree on **py_trees** (`pip install py_trees`). Submerge (via Auv) → turn LEFT until any gate role-image label (`compass/hammer_and_wrench/buoy/sos`) seen on `vision/detections` → stop pointing at gate. Bring stack + detector up FIRST. Flags: `--depth-ft`, `--yes`, style rotation defaults roll:720/pitch:360/yaw:720. |
| `auto_tune_pid.py` | Heading-PID auto-tuner. `--sim` (default, safe, offline plant) or live: steps `motion/heading`, measures rise/overshoot/settle/SSE off `imu/rpy`, Hooke-Jeeves pattern search over `heading_kp/ki/kd/i_limit` via `ros2 param set`. Reads but never sets `max_yaw_correction`. Sim gains are ballpark only. |
| `movement_test.py`, `depth_and_forward.py` | Minimal Auv exercise scripts (submerge, forward, turns, strafes). |
| `convert_to_onnx.py`, `build_engine.py`, `deploy_model.sh` | Model pipeline (section 7). |
| `setup_vision_env.sh`, `reset_zed_node.sh` | Env/ZED helpers. |
| `test_vision.py`, `test_pipeline.py` | Ad-hoc vision checks (not the pytest suite). |

## 12. Build, test, and tuning workflow

```bash
cd ~/robosub2026/robosub-2026
source /opt/ros/humble/setup.bash
colcon build --symlink-install          # ALWAYS --symlink-install (see gotchas)
source install/setup.bash
python -m pytest tests/ -q              # expect ~153 passed, ~35 s, no hardware
```

**Staged test plan** (`docs/testing-submerge-hold.md` — follow it, don't skip):

1. **Offline pytest** — `tests/test_e2e_submerge.py` runs a live motion_node
   against a simulated vehicle that veers right under thrust (the 2026-07-13
   symptom) and asserts the whole feature.
2. **Simulate mode** — full stack with `simulate:=true`, no hardware.
3. **Dry with Pixhawk** — preflight, mode readback, props off/clear.
4. **Pool.**

Test files pin specific contracts: `test_heading_lock.py` (sign convention),
`test_submerge.py` (ordering + zero heave on every failure path),
`test_motion.py` (axis authority), `test_motion_node.py` (loss paths,
sole-publisher guard, param bounds), `test_gateway*.py` (mode readback,
hull-baro rejection, preflight fails closed), `test_pressure.py`,
`test_pose_source.py`, `test_thruster_params.py`, `test_msg_compat.py`,
`test_imu_math.py`, `test_pid.py`, `test_depth_controller.py`.

**Live tuning at the pool:**

```bash
ros2 param set /motion_node heading_kp 1.6
rqt_plot /heading/error /heading/yaw_correction /motion/forward_cmd
ros2 topic echo /submerge/state
```

## 13. Quick-reference tables

### Topics

| Topic | Type | Publisher → Consumers |
|---|---|---|
| `movement_command` | MovementCommand | motion_node (or autonomous_controller/prequal — ONE at a time) → thruster_node |
| `motion/cmd` | MovementCommand | Auv / missions → motion_node (surge/strafe/deliberate yaw) |
| `motion/submerge` | Float32 | Auv / missions → motion_node (>0 dive, ≤0 abort+ack) |
| `motion/heading` | Float32 | auto_tune_pid / missions → motion_node (absolute yaw slew, HOLD only) |
| `submerge/state` | String | motion_node → Auv / operators |
| `pixhawk/depth` | Float32 | thruster_node (m +down, NaN=none) → motion_node |
| `pixhawk/mode`, `pixhawk/armed` | String, Bool | thruster_node → motion_node, gates |
| `pixhawk/imu/data` | Imu | thruster_node (or pix_imu bridge, never both) → orientation_node |
| `imu/rpy` | Vector3Stamped | orientation_node → motion_node, Auv.turn, tuner |
| `vision/detections` | ObjectDetectionArray | detector (front) → BT, controllers, depth_node |
| `vision/path_markers` | ObjectDetectionArray | bottom_camera → drift correction |
| `depth/sub_depth` | Float32 | front detector (sole in 2D-only bottom config) |
| `vslam/odometry` | Odometry | front detector → pose_source zed |
| `odom/bottom` | Odometry | bottom_camera (full-3D only) → localization_node |
| `localization/pose` | PoseStamped | localization_node → autonomous_controller |
| `depth/info` | DepthInfo | depth_node → mission |
| `navigation_command` | NavigationCommand | SHRUB MissionIO → autonomous_controller |
| `/safety/battery_pct`, `/safety/leak_detected` | Float32, Bool | safety_monitor → MissionIO |
| `heading/*`, `depth/{current,target}`, `motion/{forward,vertical}_cmd` | Float32 | motion_node debug → rqt_plot/rviz_visualizer |
| `viz/markers`, `viz/path` | MarkerArray, Path | rviz_visualizer |

### Services

| Service | Type | Server |
|---|---|---|
| `pixhawk/set_mode` | auv_msgs/SetFlightMode | thruster_node (readback-confirmed) |
| `pixhawk/preflight` | std_srvs/Trigger | thruster_node (read-only param audit, fails closed) |
| `pixhawk/disarm` | std_srvs/Trigger | thruster_node |
| `/imu/reset_orientation` | std_srvs/Trigger | orientation_node |

### Executables

`control`: `motion_node`, `autonomous_controller`, `forward_hold_mission`,
`rviz_visualizer` · `mavlink_thruster_control`: `thruster_node`,
`safety_monitor_node` (+ `tools/move_forward.py`, `tools/reverse_thrusters.py`)
· `vision`: `detector`, `bottom_camera`, `behavior_status_listener` · `imu`:
`orientation_node`, `diagnostics_node`, `marker_node` · `localization`:
`localization_node`, `depth_node`, `drift_correction_node` · `prequalification`:
`prequalification_node`, `prequalification_dry_test` · `bt_mission`: `bt_executor`
· `pix_imu`: `pixhawk_imu_bridge`.

## 14. Hard-won gotchas & troubleshooting

Each of these cost real pool time. Check here FIRST.

| Symptom | Cause / fix |
|---|---|
| "My code change didn't take effect" | Stale `install/` from a non-symlink build. Always `colcon build --symlink-install`. The single most recurring trap. |
| "mode 19" spam, ArduSub forces MANUAL, no depth hold, sub sinks | **Bar02 dropped off I2C** (intermittent — happened again 2026-07-09). Reseat the cable. ArduSub refuses ALT_HOLD without it. (Bar02-as-30BA scaling misdetect fixed by ArduSub 4.7.0-beta7.) Note: deliberate mode-19 (MANUAL) during style rolls is expected — ALT_HOLD fights rolls, style maneuvers use MANUAL + gyro-rate feedback. |
| `device reports readiness to read but returned no data`, node dies | Two readers on one serial port. Only thruster_node may open it. Don't run pix_imu bridge, attic scripts, or a second MAVLink consumer alongside. |
| Node connects then never sees another MAVLink message ("SIMULATION mode" symptom) | pymavlink 2.4.49 `add_message` `_instances=None` TypeError. Guard must be installed before connecting (`mavlink_compat.py`). Any new standalone pymavlink script needs it too. |
| `recv_match` gets nothing despite traffic | Passing a **tuple** as `type=` silently matches nothing — use a str/list, or match `get_type()` manually. |
| Dive commanded, nothing happens, no error | Heave inside ArduSub's throttle deadzone: z within ±THR_DZ (100/1000) of neutral = no-op in ALT_HOLD. `min_heave` (0.12) floors this. `PILOT_SPEED_DN` caps descent rate. |
| Forward command spins the sub / submerge rolls or up-thrusts | Flipped `MOT_x_DIRECTION` / `SERVOx_REVERSED` — twice caused by ad-hoc scripts leaving param writes (2026-07-12/13, MOT_5/6/8 incidents). The preflight service gates every dive on the `.param` backup values; **never write these params at runtime**. `DO_MOTOR_TEST` is NOT a valid direction oracle (bypasses the mixer) — verify with a real MANUAL_CONTROL z-stick. |
| Sub slowly turns (~1.5°/s) in ALT_HOLD, EKF blind to rotation | Hard-iron compass distortion. `EK3_SRC1_YAW=0` (gyro-only) since 2026-07-09. Diff live params against the `.param` backup before deep debugging. |
| Oscillation after a commanded turn | Heading-lock target not re-captured after the turn — fixed 2026-07-14: motion_node re-`start()`s the lock every steering tick, so releasing yaw holds the NEW heading. If it recurs, look at `_tick`'s `_op_yaw is not None` branch. |
| SSH drops when the detector starts | ZED USB3 EMI jams 2.4 GHz WiFi. Use 5 GHz hotspot or ethernet tether. |
| Detections garbage / low confidence | ZED frames are **BGRA**; check channel order and letterboxing (`--save_frames`). Rebuild the TensorRT engine on-device after any model change; rebuild vision so build-dir symlinks resolve. |
| Both camera nodes grab the same camera | Open by **serial** (front 31166146, bottom 30758628) — mission_stack does; ad-hoc scripts must too. |
| Motor test cuts out / cools down 10 s | `DO_MOTOR_TEST` needs `ORDER_BOARD`(2) in p6, 0-based motor index in p1, and a 5 Hz re-send stream (500 ms lapse → 10 s cooldown). `attic/motor_test.py` handles it. |
| Dropper fires at boot / won't fire | ArduSub 4.5 re-saves `SERVO9_FUNCTION=184` every boot. Set 0 per-run without rebooting; `DO_SET_ACTUATOR` unsupported. |
| Closed-loop script commands chatter | Single-writer rule inside a controller: idle the driver + raw `send()` per tick; calling `move_*` helpers per tick = double writer (legacy field_common lesson; motion_node architecture exists to prevent this class). |
| `submerge refused — duty-cycle abort latched` | Yaw source measurably degraded (brownout pattern). Deliberate latch: acknowledge by publishing `motion/submerge` ≤ 0, fix the source, re-dive. |
| motion_node INHIBITED | Another `movement_command` publisher (autonomous_controller, prequal node, attic script). Stop it; motion_node re-enables itself. |
| `ros2 param set` refused | You hit a `PARAM_BOUNDS` safety bound or a restart-only param — the rejection message names the bound. Do not widen bounds casually; read the rationale comment in `motion_node.py`. |
| Motor trim questions | Trim was RESET 2026-07-12 (thruster 6 replaced); all motors full range. `attic/motor_trim.py`. |
| Veer-right under forward thrust | The original 2026-07-13 symptom; heading lock is the fix. Diagnostics: `attic/diagnose_forward_veer.py` (`--sweep` in water, `--wet --stabilize`); order: sweep → trim → re-sweep. |

**Historical context:** a 26-finding codebase audit (2026-07-12) lives at
`docs/robosub-audit-2026-07-12.html` with a 25-task remediation plan at
`docs/superpowers/plans/2026-07-12-audit-remediation.md`.

## 15. The attic (archived legacy toolkit)

`attic/` holds every pre-new-API root script (moved 2026-07-14): the
`field_common.py` framework, `act_*` isolated actions, `stage_*` rehearsals,
`depth_hold*`/`motor_*`/`dropper*`/`submerge_forward*` tools, `tune_pid.py`,
`gate_task.py`/`gate_runner.py`/`run_course.py`, diagnostics, and their docs
(`COMMAND_USAGE.md` at root documents them). They ARM the Pixhawk in-process and
publish `movement_command` directly — **incompatible with a running
motion_node/thruster_node**. Use them only as reference for proven MAVLink
sequences (motor test dance, trim, preflight origins); do not run them against
the current stack. Their tested logic was ported: pressure math →
`pressure.py`, preflight → `thruster_params.py`, PID → `control/pid.py`,
heading lock → `heading_lock.py`.

---

*Cross-checks when editing: sign conventions (§6), single-owner invariants (§4),
param-bounds rationale (`motion_node.py`), and MIGRATION.md's blackboard table.
Run `python -m pytest tests/ -q` after any control change — the suite is fast
and pins exactly the contracts that have bitten before.*
