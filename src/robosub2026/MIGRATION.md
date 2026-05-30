# SHRUB v4 migration — porting the working mission logic into the BT nodes

**Decision (2026-05):** `bt_mission` (this package, SHRUB v4, BehaviorTree.CPP v4)
is the canonical mission planner going forward. The legacy `src/mission/`
package (`main.cpp`, BehaviorTree.CPP **v3**, executable `bt_runner`) remains the
*only fully working* brain until this migration is complete, so **`run_stack.sh`
still launches `bt_runner`** on purpose. Do not repoint it at `bt_executor`
until the nodes below are ported and pool-verified.

## Why a migration is needed

The v4 node bodies were stubs (`return SUCCESS;`) that never touched ROS. The
working logic lives in `src/mission/src/main.cpp` (~2000 lines): a set of
publishers/subscribers plus ~30 BT action nodes that drive the Python autonomy
stack via topics and close the loop on vision.

## What is already done (this commit)

- **`include/bt_mission/mission_io.hpp` + `src/mission_io.cpp`** — the shared ROS
  I/O layer (`shrub::MissionIO`). Singleton created in `bt_executor`. It:
  - publishes `auv_msgs/MovementCommand` → `movement_command` (consumed by
    `mavlink_thruster_control/thruster_node`)
  - publishes `auv_msgs/NavigationCommand` → `navigation_command` (consumed by
    `control/autonomous_controller`)
  - subscribes to `vision/detections`, `depth/info`, `localization/pose` and
    caches them with thread-safe getters (`depth()`, `detections()`,
    `bestDetection()`, `pose()`).
- **`bt_executor.cpp`** — calls `MissionIO::init(ros_node)` and injects live
  `depth` onto the blackboard each tick, so the SafetyMonitor reads a real
  sensor.
- **`EmergencySurface`** — now commands `emerge` via `MissionIO` instead of only
  logging.
- Build wiring: `auv_msgs` added to `CMakeLists.txt` + `package.xml`.

> ⚠️ None of the C++ in this commit has been compiled in the dev environment.
> First step for whoever picks this up:
> `colcon build --packages-select auv_msgs bt_mission` on the Jetson and fix any
> compile errors before continuing.

## The I/O pattern to use in every node

```cpp
#include <bt_mission/mission_io.hpp>
using shrub::MissionIO;

// Command movement (open-loop; thruster_node auto-stops after `duration`s):
MissionIO::get().sendMovement("surge_forward", /*speed*/0.4, /*duration*/2.0);

// Or hand a whole behavior to the autonomous_controller (closed-loop):
MissionIO::get().sendNav("track_object", /*label*/"gate", /*speed*/0.4);

// Read vision feedback inside onRunning():
shrub::Detection d;
if (MissionIO::get().bestDetection("gate", 0.5, d)) {
  double err_x = d.cx - 0.5;   // horizontal centering error
  // ... return RUNNING until centered, then SUCCESS
}
```

Prefer `sendNav(...)` for anything that needs a control loop (centering,
approaching, station-keeping) — `autonomous_controller` already implements those
modes well. Use `sendMovement(...)` only for short open-loop primitives
(submerge a fixed time, fire-and-forget turns, the emergency abort).

## Node-by-node port map  (legacy `main.cpp` → v4 node)

| v4 node (this pkg) | Source of truth in legacy `main.cpp` | Suggested wiring |
|---|---|---|
| `Submerge`, `AscendTo` | `Submerge` | `sendMovement("submerge"/"emerge", s, t)` then SUCCESS |
| `Turn`, `Face_direction` | `TurnRight90`, heading nodes | `sendNav("heading_hold", "", 0, 0)` w/ target_yaw, or open-loop rotate |
| `Move_through_gate` | `Move_with_style_through_gate` | `sendMovement("surge_forward", ...)` for a fixed push |
| `Navigate_forward`, `Navigate_on_heading` | `Move_until_the_other_end_of_the_path` | `sendNav("waypoint"/"heading_hold", ...)` |
| `Navigate_to_bearing` | pinger logic | `sendNav` once acoustic bearing is available |
| `Detect_gate`, `Detect_buoy`, `Detect_bin_below`, `Detect_object`, `Detect_task_board`, `Detect_opening`, `Detect_path_marker`, `Detect_slalom_pipes`, `Detect_vertical_marker`, `Detect_float_area_below` | `VisionSubscriber` + `Detect_*` classes | `bestDetection(label, conf, d)` → SUCCESS/RUNNING; write result to BB |
| `Detect_animal_on_gate`, `Detect_animal_image` | `Detect_preferred_animal_left_of_center` | same; set `animal`/`side`/`dir` outputs |
| `Align_to`, `Align_above`, `Align_to_opening`, `Align_to_basket`, `Center_beneath` | `Center_*` nodes | `sendNav("track_object", label)` and watch `bestDetection` centering error |
| `Approach_and_touch` | buoy touch logic | `sendNav("track_object", "buoy", ...)` with small approach_dist |
| `Surface_in_float_area`, `Confirm_overhead` | `SurfaceInOctagon`, `ConfirmOverhead` | confirm via `Detect_float_area_below` then `sendMovement("emerge")` |
| `Grab_object`/`Release_object` | `Grab_trash_with_claw` | **needs a gripper driver** (none exists yet) |
| `Drop_marker`, `Fire_torpedo` | `Drop_marker` | **needs a marker/torpedo actuator driver** (none exists yet) |
| `Style_through_gate` | `Move_with_style_through_gate` | open-loop roll/pitch via `sendMovement` |
| `Determine_basket`, `Compute_slalom_path`, `Increment`, `Compute_reverse_heading` | pure logic — already correct in v4 | no ROS needed |

## Known issues to fix during the port

1. **`IsTimeRemaining` clock mismatch** (`safety_nodes.cpp`): it compares
   `steady_clock` epoch against `start_time` seeded from `ros_node->now()`
   (ROS time). Inject elapsed/remaining from `bt_executor` (which has the real
   `start_time`) onto the blackboard instead, or pass the ROS clock in.
2. **No `battery_pct` / `leak_detected` publisher** anywhere in the stack. The
   SafetyMonitor's battery + leak checks are effectively no-ops (they read the
   seeded blackboard defaults). Add a hardware monitor node (Pixhawk
   `SYS_STATUS` battery, leak GPIO) that `MissionIO` can subscribe to, then push
   onto the blackboard like `depth`.
3. **No gripper / marker / torpedo actuator drivers.** `Grab_object`,
   `Drop_marker`, `Fire_torpedo` can't do anything real until those exist.
4. **`bt_xml/robosub2026_mission.xml`** uses the v4 node vocabulary; keep it in
   sync as nodes are ported. Validate with `xmllint --noout`.

## Definition of done

- `colcon build --packages-select auv_msgs bt_mission` clean.
- `ros2 run bt_mission bt_executor` drives the sub through the gate in the test
  tank using the real Python stack (detector + autonomous_controller +
  thruster_node).
- Then, and only then: switch `src/run_stack.sh` from `mission bt_runner` to
  `bt_mission bt_executor` and delete the legacy `src/mission/` package.
