# Submerge-and-Hold — design

**Date:** 2026-07-14
**Status:** approved, ready for implementation plan
**Supersedes (partially):** `2026-07-14-heading-lock-design.md` — `heading_lock_node`
is retired into `motion_node` by this design; the pure logic in
`control/heading_lock.py` is kept verbatim.

## Goal

One command submerges the sub to a target depth and then holds it there —
depth, heading and attitude maintained automatically — while the operator
issues only forward/lateral movement.

```python
from control.api import Auv

auv = Auv()
auv.submerge_to_depth(target_depth=2.0, dive_speed=0.3)  # blocks; on exit: at depth, ALT_HOLD, heading captured
auv.move_forward(speed=0.4, duration=10)                 # depth + heading + attitude held automatically
auv.stop()
```

## Principles

1. **ArduSub owns what ArduSub is good at.** Depth and attitude are held by the
   Pixhawk's own controllers in ALT_HOLD. No custom depth PID, no custom
   roll/pitch stabilisation. The single genuine gap is *heading* hold — ArduSub
   has no yaw-heading-hold mode — so that, and only that, is closed in software.
2. **One process touches the serial port.** `thruster_node` already owns
   `/dev/ttyACM0` and is the only MAVLink writer. It becomes the gateway: sole
   reader too. Nothing else in the stack imports `pymavlink`.
3. **One node publishes `movement_command`.** Today `heading_lock_node`,
   `autonomous_controller` and the standalone scripts all publish it; any two
   running at once fight over the thrusters. `motion_node` becomes the sole
   publisher and refuses to start if anything else is on the topic.
4. **Control logic is pure Python.** The four controllers hold no ROS handles
   and no MAVLink. They are unit-tested without a robot.
5. **Visualisation is subscribe-only.** It reads published topics and never
   feeds back into control, so it can be omitted from a competition launch with
   no behavioural change.

## Architecture

```
                    ┌──────────────────────────────────────────┐
  Pixhawk ─serial───┤ thruster_node   (MAVLink gateway)        │
                    │  sole reader, sole writer                │
                    └──┬────────────────────────────────────▲──┘
      publishes ───────┤                                    │
        pixhawk/imu/data   (sensor_msgs/Imu)                │ subscribes
        pixhawk/depth      (std_msgs/Float32, m, +down)     │   movement_command
        pixhawk/mode       (std_msgs/String)                │ serves
        pixhawk/armed      (std_msgs/Bool)                  │   pixhawk/set_mode (srv)
                          │                                 │
                          ▼                                 │
                    ┌──────────────────────────────────────────┐
                    │ motion_node   (centralized movement node)│
                    │   SubmergeController  (phase sequencer)  │
                    │   DepthController     (dive only)        │
                    │   HeadingController   (yaw P/PID)        │
                    │   MotionController    (axis mixer)       │
                    └──┬───────────────────────────────────▲───┘
     debug topics ─────┤                                   │ subscribes
       heading/{current,target,error,yaw_correction}       │   motion/cmd
       motion/{forward_cmd,vertical_cmd}                   │   motion/submerge
       depth/{current,target}                              │
       submerge/state                                      │
                          │                                 │
                          ▼                          control/api.py  (Auv façade)
                    ┌──────────────────────┐
                    │ rviz_visualizer      │  subscribe-only: MarkerArray, Path, TF
                    └──────────────────────┘
```

## Layer 1 — `thruster_node` as MAVLink gateway

`src/mavlink_thruster_control/mavlink_thruster_control/thruster_node.py`

### Reader

One dedicated reader thread replaces the current opportunistic drain inside
`_check_armed_status`. It `recv_match`es `ATTITUDE`, `RAW_IMU`,
`SCALED_PRESSURE2`, `HEARTBEAT`, `STATUSTEXT` and fans them out to publishers.

The existing `_external_recv_reader` escape hatch (added because
`field_common.Bar02DepthSource` also read the port) is no longer needed *within
the ROS stack* and stays only for the standalone-script path.

`pymavlink` 2.4.49's `add_message` crash guard (`_safe_add_message`) must be
installed before the connection is created — it already exists in
`field_common.py` and `pixhawk_imu_bridge.py` and moves into one shared module
rather than being copied a third time.

### New publishers

| Topic | Type | Rate | Contents |
|---|---|---|---|
| `pixhawk/imu/data` | `sensor_msgs/Imu` | 50 Hz | `ATTITUDE` quaternion + body rates, `RAW_IMU` accel. Same contract `pixhawk_imu_bridge` already publishes, so `imu/orientation_node` consumes it unchanged. |
| `pixhawk/depth` | `std_msgs/Float32` | 10 Hz | metres below surface, positive down. `NaN` while no valid pressure. |
| `pixhawk/mode` | `std_msgs/String` | 1 Hz + on change | live `HEARTBEAT.custom_mode` decoded (`MANUAL`/`STABILIZE`/`ALT_HOLD`/…), **not** the requested mode. |
| `pixhawk/armed` | `std_msgs/Bool` | 1 Hz + on change | `HEARTBEAT.base_mode & SAFETY_ARMED`. |

### Depth derivation

Bar02, via `SCALED_PRESSURE2`. Startup sequence, ported from the proven code in
`depth_and_forward.py` into a shared `mavlink_thruster_control/pressure.py`:

1. `detect_pressure_source()` — prefer `SCALED_PRESSURE2`/`3` over instance-0
   `SCALED_PRESSURE` (which is the FMU baro reading *hull air*, not water).
2. `latch_surface()` — median of 10 samples at the surface → `p0`.
3. `surface_sane(p0)` — reject an implausible latch.
4. `depth_m = (press_abs - p0) * 100.0 / (rho * g)`, `rho = 1000`, `g = 9.80665`.

If no external pressure source appears, the node publishes `NaN` depth and logs
an error. It does **not** fall back to the hull baro — a wrong depth is worse
than no depth.

### `pixhawk/set_mode` service

New `auv_msgs/srv/SetFlightMode.srv`:

```
string mode        # MANUAL | STABILIZE | ALT_HOLD | ACRO
---
bool success
string reason      # ArduSub STATUSTEXT on failure, else ''
```

The handler sends `set_mode_send`, then waits up to `mode_ack_timeout_s`
(default 3.0) for `HEARTBEAT.custom_mode` to **read back** the requested mode.
On timeout it returns `success=false` with the most recent `STATUSTEXT` (e.g.
`"Depth sensor is not connected."`), which is exactly how a dead Bar02 announces
itself. A topic could not carry this, which is why this is a service: the spec
requires "if the requested mode cannot be entered: abort safely, notify the
caller".

`set_flight_mode()` continues to update `flight_mode_name`/`flight_mode_id`, so
the existing 0.2 Hz mode-reassert watchdog re-asserts the *new* mode and does
not fight the caller.

### Unchanged

The `movement_command` subscription, the axis primitives, the arm/disarm logic,
the watchdog and the reconnect path are untouched. The `MovementCommand`
contract does not change.

## Layer 2 — pure control logic

`src/control/control/`. No ROS, no MAVLink, no I/O. Each is unit-tested.

### `HeadingController` — `heading_lock.py` (exists, unchanged)

Already implements capture-yaw-on-start, PID on `wrap(current − target)`,
clamp to `max_yaw_authority`, and the stale contract
(`LOCKED / STALE_GRACE / ABORTED`). Its sign convention is pinned by
`tests/test_heading_lock.py` and is not revisited.

Only change: `motion_node` constructs it instead of `heading_lock_node`.

### `DepthController` — `depth_controller.py` (new)

**Not a PID.** ArduSub holds depth; this only performs the *dive*.

```python
class DiveState(Enum):
    IDLE, DIVING, AT_DEPTH, TIMEOUT, NO_DEPTH_DATA

class DepthController:
    def __init__(self, tolerance_m=0.15, min_heave=0.12,
                 timeout_s=30.0, stale_timeout_s=1.0): ...
    def start(self, target_depth_m, dive_speed): ...
    def update(self, depth_m, now_s) -> (heave, DiveState): ...
    def stop(self): ...
```

* `depth_m is None` (stale/`NaN`) → heave 0, `NO_DEPTH_DATA`. Never dive blind.
* `target − current <= tolerance_m` → heave 0, `AT_DEPTH` (latches).
* elapsed > `timeout_s` → heave 0, `TIMEOUT`.
* otherwise heave = `dive_speed`, magnitude floored at `min_heave`.

`min_heave` exists because of the ALT_HOLD throttle deadzone: ArduSub treats a
`z` within ±`THR_DZ` (100 of 1000) of neutral as *no command at all*, so a heave
of 0.05 does nothing while looking like it should. Flooring the magnitude at
0.12 keeps a small commanded dive_speed from silently becoming a no-op.

Descent rate in ALT_HOLD is additionally capped by ArduSub's `PILOT_SPEED_DN`;
this is documented, not worked around.

### `SubmergeController` — `submerge.py` (new)

Phase sequencer. Pure: side effects go through an injected `Effects` protocol
(`preflight() -> (ok, reason)`, `set_mode(name) -> (ok, reason)`,
`wait_armed(timeout) -> bool`, `depth() -> Optional[float]`, `yaw() -> Optional[float]`)
so it is testable with a fake.

```
IDLE → PREFLIGHT → MODE_SET(ALT_HOLD) → ARMED? → DIVE → CAPTURE_HEADING → HOLD
                                                                           ↓
                            (any step fails) ────────────────────────→ FAILED
```

* **PREFLIGHT** — the non-skippable `MOT_*_DIRECTION` / `SERVO*_REVERSED`
  check from the 2026-07-13 incident. A flipped *horizontal* thruster turns a
  forward command into a spin; a flipped vertical makes a submerge command
  climb. Verified against the `.param` backup before any dive.
* **MODE_SET** — calls the `pixhawk/set_mode` service and requires the
  **read-back**, not the send. `thruster_node` already sets ALT_HOLD and arms
  during `_connect_mavlink`, but "the gateway asked for ALT_HOLD" is not the
  same claim as "the vehicle is in ALT_HOLD" — with a dead Bar02 ArduSub
  refuses the mode and silently stays in whatever it was in. This step is what
  turns that silent refusal into an abort.
* **ARMED?** — *waits for* `pixhawk/armed`; it does not arm. Arming (and
  re-arming) is `thruster_node`'s job and stays there — two arming authorities
  would be exactly the overlap this design exists to remove. If the vehicle is
  not armed within the timeout, abort.
* **Ordering matters:** ALT_HOLD is confirmed *before* any heave is commanded,
  so the failure mode of a dead depth sensor is "sub sits at the surface", never
  "sub descends with no depth hold and no way to stop".
* **DIVE** — delegates to `DepthController`.
* **CAPTURE_HEADING** — `HeadingController.start(current_yaw, base_speed=0)` the
  moment `AT_DEPTH` latches. This is the `desired_heading = current_heading`
  step from the brief.
* **HOLD** — heave released to 0 permanently. ALT_HOLD owns depth and
  roll/pitch from here on.

### `MotionController` — `motion.py` (new)

The axis mixer, and the enforcement point for "only the forward channel comes
from the user".

```python
def mix(self, operator_surge, operator_strafe, yaw_correction, heave,
        operator_yaw=None) -> Axes
```

Yaw comes from `HeadingController` unless the operator *explicitly* requests a
new heading (`operator_yaw is not None`), in which case the manual value passes
through and the heading lock re-captures on release. Heave is 0 in HOLD. Roll
and pitch are always 0 — ArduSub self-levels them, and commanding them would
fight its attitude controller.

## Layer 3 — `motion_node`

`src/control/control/motion_node.py`. The centralized movement node and the
**sole publisher of `movement_command`**.

### Isolation guard

On startup, and periodically, it calls `count_publishers('movement_command')`.
If anyone else is publishing, it logs an error and refuses to command. This is
the mechanism that makes "nothing overlaps" enforceable rather than aspirational.

### Subscribes

| Topic | Type | Purpose |
|---|---|---|
| `imu/rpy` | `geometry_msgs/Vector3Stamped` | yaw. Topic name is the `yaw_topic` param, so the source (Pixhawk vs ZED) is a launch-time choice. |
| `pixhawk/depth` | `std_msgs/Float32` | depth for the dive and for the loss-of-depth abort. |
| `pixhawk/mode` | `std_msgs/String` | if the vehicle leaves ALT_HOLD unexpectedly, depth hold is gone → stop. |
| `motion/cmd` | `auv_msgs/MovementCommand` | operator surge/strafe/duration. Reuses the existing message; `heave` and `roll/pitch` fields are ignored on this topic and a warning is logged if non-zero. |
| `motion/submerge` | `std_msgs/Float32` | target depth in metres; `<= 0` aborts the dive. |

### Publishes

`movement_command` (`axes`, every tick while active; `stop` on abort/idle), plus
the debug topics below.

### Parameters

| Param | Default | Meaning |
|---|---|---|
| `target_depth` | 2.0 | m below surface |
| `dive_speed` | 0.3 | normalized heave, floored by `min_heave` |
| `forward_speed` | 0.4 | default surge for `move_forward` |
| `heading_kp` / `heading_ki` / `heading_kd` | 1.2 / 0.0 / 0.3 | as `heading_lock_node` today |
| `max_yaw_correction` | 0.4 | clamp on the yaw output |
| `control_rate_hz` | 20.0 | tick rate |
| `depth_timeout` | 30.0 | dive abort |
| `depth_tolerance_m` | 0.15 | `AT_DEPTH` band |
| `yaw_topic` | `imu/rpy` | |
| `stale_timeout_s`, `grace_s`, `stale_window_s`, `stale_duty_abort` | as `heading_lock_node` | inherited stale contract |

The param validator from `heading_lock_node` (`PARAM_DEFAULTS` / `PARAM_BOUNDS` /
whole-batch validation, restart-only rejection) moves across intact. Those bounds
are safety limits — a `max_yaw_correction` of 0 inverts the clamp into constant
full authority.

### Safety

| Loss | Response |
|---|---|
| **Heading data lost** | Yaw correction → 0 immediately (never steer blind). Forward continues for `grace_s`, then stop. Depth hold is unaffected — heave stays 0, ALT_HOLD is untouched. Error logged. Degraded-source duty-cycle abort (`stale_duty_abort`) carries over from `heading_lock_node` and latches until acknowledged. |
| **Depth data lost** | Stop all movement (`surge = strafe = 0`). Remain in ALT_HOLD if the autopilot still holds it. Publish `submerge/state = NO_DEPTH_DATA` and log an error. Do **not** attempt a dive. |
| **Mode lost** (`pixhawk/mode != ALT_HOLD` after HOLD) | Depth hold is gone. Stop movement, log an error, attempt one re-set via the service, and if that fails surface the failure to the caller. |
| **Mode cannot be entered** | `SubmergeController` → `FAILED` with the ArduSub reason, *before* arming. `Auv.submerge_to_depth` raises. |
| **Tick exception** | Stop + unlock, as `heading_lock_node` does today. |

## Layer 4 — `control/api.py`

The encapsulation layer: one façade, used identically by mission scripts,
BehaviorTree action nodes, and interactive pool operation. It publishes to
`motion_node` and subscribes to `submerge/state`; it holds no MAVLink and no
control state.

```python
class Auv:
    def __init__(self, node=None): ...
    def submerge_to_depth(self, target_depth, dive_speed=0.3, timeout=None) -> None
        """Block until AT_DEPTH. Raises SubmergeError(reason) on FAILED/TIMEOUT."""
    def move_forward(self, speed, duration) -> None
        """Block for `duration`. Depth/heading/attitude held automatically."""
    def move_lateral(self, speed, duration) -> None
    def set_heading(self, yaw_rad) -> None
        """Intentional heading change; re-captures the lock at the new value."""
    def stop(self) -> None
    def surface(self) -> None
```

Speeds are normalized `0.0–1.0`, matching `MovementCommand`. The brief's
`dive_speed=-300` / `speed=400` are raw MAVLink units; the API uses the
project's existing normalized convention instead of introducing a second one.

## Layer 5 — `rviz_visualizer.py`

`src/control/control/rviz_visualizer.py`. Subscribe-only, launched separately,
omitted in production. It never publishes `movement_command`.

### Pose source

Position is the honest problem: **Bar02 + Pixhawk IMU cannot determine XY
position.** No DVL, no GPS, no USBL. One interface, selected by the
`pose_source` param:

```python
class PoseSource(ABC):
    def pose(self) -> Optional[PoseStamped]: ...
    def is_estimate(self) -> bool: ...
```

* `pixhawk_imu` (default) — `DeadReckonPose`: integrates the *commanded* surge/
  strafe through the Pixhawk yaw, takes z straight from `pixhawk/depth`.
  `is_estimate() → True`. Rendered in a distinct colour and labelled
  **"ESTIMATED (dead-reckoned — no XY sensor)"** in the text marker. It drifts,
  and the display says so.
* `zed` — `ZedOdomPose`: relays the existing `vslam/odometry`.
  `is_estimate() → False`.

A DVL or EKF later becomes a third implementation with no controller change.

### Markers (`visualization_msgs/MarkerArray` on `viz/markers`)

| Marker | Content |
|---|---|
| ARROW | current heading, from `base_link` |
| ARROW | desired heading (`HeadingController.target_yaw`) |
| LINE_STRIP | arc between the two, width scaled by `heading_error` |
| TEXT_VIEW_FACING | current depth, target depth, current heading, desired heading, heading error, yaw correction, forward command, flight mode, pose-estimate warning |
| PLANE (thin CUBE) | the target-depth plane, so the depth error is visible spatially |
| SPHERE | target waypoint. **Nothing publishes a waypoint in this scope** — the marker subscribes to `viz/target_waypoint` (`geometry_msgs/PointStamped`) and simply renders nothing until some future node populates it. It is a hook, not a feature. |

Plus `nav_msgs/Path` on `viz/path` (trailing N poses) and TF `map → odom →
base_link`. `imu/orientation_node` already publishes `base_link → imu_link` and
is reused.

### Debug topics (`rqt_plot`, no RViz needed)

`heading/current`, `heading/target`, `heading/error`, `heading/yaw_correction`,
`motion/forward_cmd`, `motion/vertical_cmd`, `depth/current`, `depth/target`
(all `std_msgs/Float32`), `pixhawk/mode` (`String`), `submerge/state` (`String`).

## Testing

| Level | What |
|---|---|
| Unit (`tests/`) | `DepthController` — tolerance latch, timeout, stale → `NO_DEPTH_DATA`, `min_heave` deadzone floor. `SubmergeController` — full happy path, and a `FAILED` for each of preflight/mode/arm/dive-timeout via a fake `Effects`. `MotionController` — operator yaw override, heave-zero-in-HOLD, roll/pitch always 0. `HeadingController` — existing tests unchanged. |
| Node | `motion_node` with fake topic inputs: verifies it publishes `stop` on depth loss, zeroes yaw on heading loss, and refuses to start when a second `movement_command` publisher exists. `thruster_node` in `simulate=True` for the gateway publishers and the `set_mode` failure path. |
| Message compat | Extend `tests/test_msg_compat.py` for `SetFlightMode.srv`. |
| In-water | Bench: preflight gate + `set_mode` failure with the Bar02 unplugged. Pool: dive to 2 m, confirm hold, then `move_forward` and confirm the veer-right symptom is gone with RViz + `rqt_plot` recording. |

## Explicitly out of scope

* **Station-keeping / XY drift rejection.** "Minimize drift when no
  translational commands are given" requires XY position feedback. With Bar02 +
  IMU only, it is not achievable, and ArduSub has no position-hold mode without
  GPS or a DVL. What the vehicle actually does with zero command: holds depth,
  holds heading, stays level, and **drifts with the current in XY**. Closing
  this needs the ZED pose source driving a position loop, and it is a follow-on
  project.
* A custom depth PID. ArduSub's is used.
* Custom roll/pitch stabilisation. ALT_HOLD self-levels.
* Migrating the standalone scripts (`depth_and_forward.py`, `field_common.py`)
  onto the gateway. They keep their own MAVLink and remain mutually exclusive
  with the ROS stack, as they are today.

## Migration

* `heading_lock_node` is deleted; its entry point is removed from
  `src/control/setup.py`. `control/heading_lock.py` and its tests survive intact
  inside `motion_node`.
* `pix_imu/pixhawk_imu_bridge` remains as the dry-bench / IMU-only path. It must
  never run alongside `thruster_node` — both open the serial port, and two
  readers on one port produce the "device reports readiness to read but returned
  no data" stall. The launch files enforce this: `pix_imu_viz.launch.py` (bridge,
  no thruster node) versus the new `submerge_hold.launch.py` (thruster gateway,
  no bridge). The mutual exclusion is documented at the top of both.
