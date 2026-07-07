# Vision Centering Framework

*How the sub centers itself on a target (gate, torpedo hole, bin, …) and knows
when / where to stop.*

This documents the task-aware, simultaneous multi-axis centering system that
lives in `src/control/control/`. It is the AUV's answer to OSU's
"stereo → depth → pos → align" pipeline (`riptide_perception`'s `tensor_detector`

+ `riptide_mapping`), adapted to this stack.

---

## TL;DR — what changed

| Layer | File | Change |
| --- | --- | --- |
| **Message** | `src/auv_msgs/msg/MovementCommand.msg` | Added a native 4-axis mode: `command="axes"` + `surge/strafe/heave/yaw_rate`. Backward compatible (verb commands unchanged). |
| **Thruster** | `src/mavlink_thruster_control/.../thruster_node.py` | `axes` command applies all four MAVLink axes simultaneously every tick (was one primitive per tick). |
| **Framework** | `src/control/control/centering.py` *(new)* | `TargetState`, `TargetTracker` (filter + coast), `CenteringPolicy`/`GatePolicy`, `policy_for()`, `centering_errors()`. ROS-free, unit-testable. |
| **Controller** | `src/control/control/autonomous_controller.py` | `_tick_track_object` rewritten: policy-driven, simultaneous 4-axis, filtered target, convergence confirmation. |
| **BT gate** | `src/robosub2026/src/nav_nodes.cpp` | `AlignGateCenter` now requires centered **and** at standoff range (was centering-only). |

---

## Architecture

```
                       vision/detections (ObjectDetectionArray)
                       position.x/y = norm image centre, position.z = range_m
                                       │
                                       ▼
   ┌─────────────────────────────────────────────────────────────┐
   │ autonomous_controller  (track_object mode, 10 Hz)            │
   │                                                              │
   │  detection ─▶ TargetTracker ─▶ TargetState (filtered+coast)  │
   │                                  │                            │
   │                  CenteringPolicy (gate/torpedo/…) ◀──────────┤
   │                                  │                            │
   │             centering_errors() ──▶ body-frame errors          │
   │                                  │                            │
   │        PIDs (yaw/strafe/depth) + shape_surge() ──▶ 4-axis     │
   │                                  │                            │
   │             _dispatch_setpoint(surge,strafe,heave,yaw) ──┐    │
   └──────────────────────────────────────────────────────────┼────┘
                            MovementCommand(command="axes")   ▼
                       ┌──────────────────────────────────────────┐
                       │ thruster_node  → manual_control(x,y,z,r)   │
                       └────────────────────────────────────────────┘
```

### The three layers (`centering.py`)

1. **`TargetState`** — frame-agnostic snapshot of where the target is. Carries
   image-space (`cx`, `cy`, `bbox`) AND metric (`range_m` + derived
   `lateral_m`/`vertical_m`/`forward_m`) fields, plus a `frame` tag
   (`'camera'` today; `'world'` when a mapping node feeds it later). **This tag
   is the seam for a future OSU `riptide_mapping`-style world-frame map** — when
   localization is reliable, publish `TargetState(frame='world')` and the same
   policies + controller work unchanged.

2. **`TargetTracker`** — per-label EMA smoothing + a "coast" window. A single
   jittery or dropped frame no longer spikes the controller: the last good
   state is smoothed and held for `coast_s` seconds before expiring. **This is
   the robustness layer on top of reactive per-frame centering.**

3. **`CenteringPolicy`** — per-task config: desired standoff, lateral/vertical
   offsets, tolerances, active axes, convergence confirmation.
   `GatePolicy` is implemented; torpedo/bin have sensible stubs via
   `policy_for(label)` so the controller works for every label today.

### Why reactive (not a world map) today?

VSLAM is intentionally disabled for competition
(`src/localization/launch/vslam_localization_launch.py`). A world-frame object
map built on unreliable localization would be **less** robust than reactive
per-frame centering. The `TargetState.frame` tag is the seam: ship the mapping
layer later without rewriting the controller.

### Distance source

Each ZED camera already solves the 3D pose of every detected object (ZED SDK
custom-box object tracking in `vision/detector.py`) and publishes the slant
range as `ObjectDetection.position.z`. **That is OSU's "stereo → depth →
distance" step — already done.** The controller uses that range for the metric
standoff + stop condition, and falls back to bounding-box size when range is
unavailable (2D-only / depth disabled).

Metric lateral/vertical offsets are derived from image bearing + range + camera
FOV (pinhole unprojection) in `TargetState.metric_offsets()` — this is
**convention-independent** (does not depend on the ZED's configured
`sl.COORDINATE_SYSTEM`), so it is correct regardless of coordinate settings.

---

## The control law (`_tick_track_object`)

Each 10 Hz tick, per the active `CenteringPolicy`:

1. Feed the freshest detection into the `TargetTracker` (coasts through drops).
2. Read the filtered `TargetState`; if expired, **hold** (depth-hold).
3. Compute body-frame errors via `centering_errors()`:
   + **Metric mode** (range available): `yaw_err` = bearing to desired lateral
     pos; `strafe_err` = lateral offset; `depth_err` = vertical offset;
     `surge_remaining` = `range − standoff`.
   + **Fallback mode** (no range): drive from normalized `cx/cy`; bbox-width as
     the range proxy.
4. Run PIDs on yaw/strafe/depth + `shape_surge()` for forward approach.
5. Dispatch **one** simultaneous `axes` setpoint (all four converge together —
   not one axis per tick like before).

**When to stop (convergence):** policy tolerances held for `converge_ticks`
consecutive ticks → dispatch a zero setpoint (hold). The BT node's own check
then returns SUCCESS. For the gate, that's "centered within ±0.06 cx/cy (or
±0.15 m) AND within ±0.30 m of the 1.5 m standoff, for 3 ticks."

**Where to stop:** the policy's `standoff_m` (gate: 1.5 m, overridable by the
`approach_dist` field of `NavigationCommand`) and `lateral_offset_m` /
`vertical_offset_m` (0 = on the centerline; set ± for a gate-side pass).

---

## ROS parameters (autonomous_controller)

Tunable at launch / runtime with `ros2 param`:

| Param | Default | Meaning |
| --- | --- | --- |
| `hfov_deg` | `110.0` | Forward ZED 2i horizontal FOV (HD720). For metric lateral offset. |
| `vfov_deg` | `70.0` | Vertical FOV. For metric vertical offset. |
| `ema_alpha` | `0.3` | Weight on the new sample in the target tracker (1.0 = no smoothing). |
| `coast_s` | `0.6` | Seconds to keep acting on the last good detection through dropouts. |

Set the FOV to match the actual camera resolution in use (read it from
`zed.get_camera_information()` if unsure).

---

## Adding a new task policy

This is the main extension point. To specialise, say, the torpedo task:

1. Add a `TorpedoPolicy(CenteringPolicy)` dataclass in `centering.py` with the
   task's standoff, tolerances, and active axes (e.g. tight `tol_lateral_m`,
   `use_surge` for a firing standoff, a small `standoff_m`).
2. Wire it in `policy_for()` for the labels `large_opening` / `small_opening`
   (currently stubs).
3. (Optional) Add a convergence-check BT node that reads `MissionIO::Detection`
   range/bbox like `AlignGateCenter` does.

No controller changes needed — the control law is task-agnostic and reads
everything from the policy. For a top-down task (bins via the bottom camera),
the geometry differs (heave = altitude, surge unused) — handle by setting
`use_surge=False` and driving `lateral/vertical` from the bottom camera's
`vision/path_markers`, or add a `frame='world'`/bottom-camera policy variant.

---

## Build

The `MovementCommand.msg` change requires rebuilding the message package, then
the Python + C++ that use it:

```bash
cd ~/robosub-2026
source /opt/ros/humble/setup.bash
colcon build --packages-select auv_msgs
colcon build --packages-select control mavlink_thruster_control
colcon build --packages-select bt_mission     # C++ — verify on the Jetson
source install/setup.bash
```

> The C++ BT layer (`bt_mission`) is not compiled in this workspace yet
> (see `src/robosub2026/MIGRATION.md`). Verify `nav_nodes.cpp` builds there.

---

## Testing

End-to-end (pool / dry): run the detector + thruster + controller, then send a
high-level goal:

```bash
# terminal 1 — thruster driver (owns the serial port)
ros2 run mavlink_thruster_control thruster_controller

# terminal 2 — vision (front ZED)
ros2 run vision vision_node --ros-args -p ...

# terminal 3 — the controller
ros2 run control autonomous_controller

# terminal 4 — send a track_object goal on the gate
python3 test_centering.py --label gate --speed 0.3 --approach 1.5
# (Ctrl+C sends idle)
```

`test_centering.py` publishes `NavigationCommand(mode=track_object, …)` at 1 Hz
so the controller stays in centering mode. Watch `movement_command` (or the
thruster's MAVLink TX log) for the streaming `axes` setpoints and the
controller's "found/switching" logs.

Tune live with `ros2 param set /autonomous_controller ema_alpha 0.2` etc.

---

## Known gaps / next steps

+ **`TorpedoPolicy` / `BinPolicy`** — stubs only; flesh out with task-specific
  tolerances + standoffs (see "Adding a new task policy").
+ **World-frame map** — `TargetState.frame='world'` seam is in place; ship a
  `riptide_mapping`-style node when localization is reliable enough.
+ **Metric 3D pose in the message** — today range is the slant distance; a
  future enhancement is to also publish the ZED's per-axis `obj.position` in
  `ObjectDetection` so metric offsets come straight from the camera (not
  derived via FOV). The controller already degrades gracefully; this just
  removes the FOV dependency.
+ **`station_keep` / `waypoint`** — still use the dominant-axis verb dispatch;
  they could migrate to the simultaneous `axes` setpoint too (the PIDs are
  already 4-axis).
