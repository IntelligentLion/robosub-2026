# SHRUB v4 migration — status and runbook

**Status (2026-05-30):** the v4 BT (`bt_mission/bt_executor`) is the only
planner. The full 2026 "Restore and Recovery" mission tree at
`bt_xml/robosub2026_mission.xml` loads cleanly, every node referenced by the
XML is registered, and the tree ticks end-to-end without hardware (most
actions log + return SUCCESS so the tree flows). Real hardware/perception
coverage is partial — see "Known gaps" below.

Legacy v3 (`src/mission/`, `bt_runner`) has been **deleted**. The duplicate
`src/zed2i_vslam(2)/`, the vendored ZED SDK examples
(`src/vision/vision/zed-sdk-master/`), and the empty `src/test/`,
`src/examples/`, `src/zed-ros2-{examples,wrapper}/` stubs are also gone.

**No hydrophones this season.** Pinger-related actions (`VerifyHydrophones`,
`DetectPinger`, `NavigateToPinger`, `DetectOctagonPinger`) have been removed
from the tree, the node declarations, and the registration. Torpedo and
octagon tasks enter via vision-only search.

## v3 → v4: the season-over-season migration (2025 → 2026)

| | **SHRUB v3 (last season)** | **SHRUB v4 (this season, 2026)** |
|---|---|---|
| Package | `src/mission/` (`mission`) | `src/robosub2026/` (`bt_mission`) |
| Executable | `bt_runner` | `bt_executor` |
| BT engine | BehaviorTree.CPP **v3** | BehaviorTree.CPP **v4** (+ `behaviortree_ros2`) |
| Structure | one ~2000-line `main.cpp` | modular node files + declarative XML |
| Mission | last season's course | 2026 **"Restore and Recovery"** course |

## What this branch does

- **`bt_xml/robosub2026_mission.xml`** — the canonical Groot2-formatted tree.
  Root: `MainTree`. ~50 subtrees, 39 conditions, ~130 action node types.
- **`include/bt_mission/shrub_nodes.hpp`** — declares every node referenced by
  the XML using compact macros (`SHRUB_SYNC`, `SHRUB_COND`, `SHRUB_STATEFUL`).
  Stateful actions share a `TimedAction` base for deadline tracking.
- **`src/safety_nodes.cpp`** — all 39 conditions. Read blackboard with safe
  defaults, or live `MissionIO` state where applicable
  (`GateDetected`, `LocalizationStable/Lost`, `ValidDropAltitude`).
- **`src/nav_nodes.cpp`** — initialization + every navigation / movement
  action. Stateful actions drive `MovementCommand` (`thruster_node`) or
  `NavigationCommand` (`autonomous_controller`) via `MissionIO`.
- **`src/perception_nodes.cpp`** — `Detect_*` / `Search*` / `Identify*` /
  `Estimate*` wrappers around `MissionIO::bestDetection(label, conf)`.
- **`src/manipulation_nodes.cpp`** — actuator stubs that update the
  blackboard counters (`markers_remaining`, `torpedoes_remaining`,
  `objects_delivered`) so downstream BT branches advance correctly.
- **`src/task_logic_nodes.cpp`** — `CalculateRotationCount` +
  `registerAllNodes(factory, ros_node)`. The single place new node types are
  registered.
- **`src/bt_executor.cpp`** — `tree_id` default `MainTree`; seeds the
  blackboard with `coin_flip`, `role`, `gate_red_side`, `style_enabled`,
  counters, and `task_start_time`. Pushes live depth in each tick.
- **`launch/shrub.launch.py`** — parameters for `coin_flip`, `role`,
  `gate_red_side`, `style_enabled` (exposed on the command line).
- **`src/run_stack.sh`** — switched from `mission/bt_runner` to
  `bt_mission/bt_executor`.

## I/O layer (unchanged interface, slightly extended)

`shrub::MissionIO` (`mission_io.hpp/cpp`) — process-wide singleton, created
once in `bt_executor`. Use from any node:

```cpp
#include <bt_mission/mission_io.hpp>
using shrub::MissionIO;

// open-loop primitive (auto-stops after `duration`):
MissionIO::get().sendMovement("surge_forward", /*speed*/0.4, /*duration*/2.0);

// closed-loop hand-off to autonomous_controller:
MissionIO::get().sendNav("track_object", /*label*/"gate", /*speed*/0.4, /*approach*/1.0);
// extended overload also accepts target_yaw/x/y/z for heading_hold / waypoint:
MissionIO::get().sendNav("heading_hold", "", 0.3, 0.0, /*target_yaw_rad*/1.57);

// read vision in onRunning():
shrub::Detection d;
if (MissionIO::get().bestDetection("gate", 0.5, d)) { /* ... */ }
```

## Blackboard contract

Seeded by `bt_executor` at startup. Anything not listed is undefined and
falls back to a node-local default.

| key | type | seed | who writes | who reads |
|---|---|---|---|---|
| `coin_flip` | string | param `normal` | executor | `CoinflipNormal`, `CoinflipBackward` |
| `role` | string | param `survey_repair` | executor + `SetRole*` actions | `RoleSurveyRepair`, `RoleSearchRescue`, role-gated branches |
| `gate_red_side` | string | param `right` | executor | `GateRedRight`, `GateRedLeft` |
| `style_enabled` | bool | param `true` | executor | `StyleModeEnabled` |
| `markers_remaining` | int | 2 | `ReleaseMarker` | `MarkersRemaining` |
| `torpedoes_remaining` | int | 2 | `LaunchTorpedo` | `TorpedoesRemaining` |
| `objects_delivered` | int | 0 | `ReleaseObject` | `OneItemInBasket`, `TwoItemsInBasket`, `CalculateRotationCount` |
| `marker_in_bin` | bool | true | `ObserveMarker`, `RetryMarkerDrop` | `MarkerInBin` |
| `light_off` | bool | true | `ActivateTool` | `LightOff` |
| `aligned` | bool | true | `DetectLargeOpening`, `DetectSmallOpening` | `AlignmentConfirmed` |
| `torpedo_hit` | bool | true | `LaunchTorpedo` | `TorpedoHit` |
| `inside_octagon` | bool | false | `EstimateOctagonCenter` | `InsideOctagon` |
| `correct_basket` | bool | true | (perception when wired) | `CorrectBasket` |
| `object_delivered` | bool | false | `ReleaseObject` | `ObjectDelivered` |
| `depth` | double | 0 → live | `bt_executor` from MissionIO::depth() | `ValidDropAltitude` (via `altitude_m`) |
| `altitude_m` | double | 1.0 | (future altimeter) | `ValidDropAltitude` |
| `task_start_time` | double | startup time | executor | `TaskTimeout` |
| `gate_search_start` | double | startup time | executor / search nodes | `GateSearchTimeout` |
| `mission_complete` | bool | false | `MissionComplete` (TODO: wire) | `MissionFinished` |
| `obstacle_detected` | bool | false | (future obstacle module) | `ObstacleDetected`, `NoCollisionRisk` |
| `depth_unstable` | bool | false | (future depth monitor) | `DepthUnstable` |
| `critical_failure` | bool | false | (set by safety logic) | `CriticalFailure` |
| `battery_pct` | double | 100 | **TODO: Pixhawk SYS_STATUS** | (reserved — no battery condition in current tree) |
| `leak_detected` | bool | false | **TODO: leak GPIO** | (reserved — no leak condition in current tree) |

## Vision label vocabulary used by the tree

The perception nodes call `MissionIO::bestDetection(label, conf, out)` with
these labels. The detector must emit `ObjectDetection.label` values that
match; otherwise the relevant `Detect_*` returns FAILURE.

```
gate, role_sign, survey_repair, search_rescue,
orange_path, slalom_pole, slalom_gap, path_marker,
pipeline, fire_bin, blood_bin, bin1, bin2,
marker, magnetic_target,
target_board, large_opening, small_opening,
octagon, basket, repair_object, medical_object
```

When a label has no real publisher yet, the corresponding `Detect_*` returns
SUCCESS so the tree can still tick — this is the "smoke-test friendly"
behavior.

## Known gaps / next work

1. **Roll/Pitch primitives** — `Roll90` / `Pitch90` log a warning today
   because `MovementCommand` has no roll/pitch axis. Add them (or send a
   compound thruster mix from `thruster_node`) for full style-point support.
2. **Battery + leak publishers** — `battery_pct` / `leak_detected` keep
   their seeded defaults. Add a hardware monitor (Pixhawk `SYS_STATUS`
   battery, leak GPIO) and push onto the blackboard each tick (mirror the
   `depth` pattern in `bt_executor`).
3. **Manipulation drivers** — `ReleaseMarker`, `LaunchTorpedo`,
   `ActivateTool`, `ReleaseObject` log + update counters but do not call
   real ROS services. Replace each `TODO: <driver>` comment when the driver
   lands.
4. **Altitude** — `ValidDropAltitude` reads a blackboard `altitude_m` that
   nothing publishes. Wire a DVL/sonar altimeter when available.
5. **TaskTimeout granularity** — currently uses the mission start time as
   `task_start_time`. For per-subtree timeouts, reset `task_start_time`
   when entering each task subtree.

## Definition of done

- `colcon build --packages-select auv_msgs bt_mission` clean ✅ (done)
- Tree validates with `xmllint --noout` ✅ (done)
- `ros2 run bt_mission bt_executor` ticks end-to-end without hardware,
  exercising every node ✅ (verified on the Jetson)
- Pool-verified gate transit + ≥1 task scored
