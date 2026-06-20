# RoboSub 2026 - SHRUB v4 Mission Planner

**SHRUB (Software for Handling and Regulating Underwater Behavior)** is an autonomous mission planner for the RoboSub 2026 competition, developed by Team IntelligentLion. Built on BehaviorTree.CPP v4 and ROS 2 Humble, SHRUB coordinates all high-level decision-making for our autonomous underwater vehicle (AUV).

> **Repo status:** SHRUB v4 (`src/robosub2026/`, package `bt_mission`,
> executable `bt_executor`) is the only mission planner; the legacy v3
> `src/mission/` package has been removed. `src/run_stack.sh` launches
> `bt_mission/bt_executor`. See
> [`src/robosub2026/MIGRATION.md`](src/robosub2026/MIGRATION.md) for the
> blackboard contract, vision label vocabulary, and known gaps.
> No hydrophones this season — pinger-related actions are gone; torpedo and
> octagon tasks enter via vision-only search. VSLAM is intentionally
> disabled for competition — see
> `src/localization/launch/vslam_localization_launch.py` for the rationale.

---

## Table of Contents

- [Overview](#overview)
- [Competition Background](#competition-background)
- [Mission Strategy](#mission-strategy)
- [Architecture](#architecture)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Depth Hold Tests](#depth-hold-tests)
- [Isolated Actions & Stage Tests](#isolated-actions--stage-tests)
- [Behavior Tree Structure](#behavior-tree-structure)
- [Custom Node Categories](#custom-node-categories)
- [Development](#development)
- [License](#license)

---

## Overview

SHRUB v4 is a complete rewrite of our mission planning system using hierarchical behavior trees. The system autonomously navigates an underwater course, completing various tasks to maximize points within a 15-minute time window.

### Key Features

- **Reactive Safety Monitoring**: Continuous battery, leak, depth, and time checks that can interrupt any task
- **Hierarchical Task Planning**: Modular subtrees for each competition task
- **Time Bonus Optimization**: Strategic task ordering to unlock maximum time bonus points early
- **Robust Failure Handling**: ForceSuccess wrappers and retry logic ensure one failed task doesn't abort the mission
- **ROS 2 Integration**: Full integration with perception, navigation, and control subsystems

### Technology Stack

- **BehaviorTree.CPP v4**: Core behavior tree engine
- **BehaviorTree.ROS2**: ROS 2 integration layer
- **ROS 2 Humble**: Robot Operating System framework
- **C++17**: Implementation language
- **CMake/ament**: Build system

---

## Competition Background

**RoboSub 2026** is an international autonomous underwater vehicle competition organized by RoboNation. Teams design, build, and program AUVs to complete a series of tasks in a controlled pool environment.

### 2026 Rule Changes

Based on the official RoboSub 2026 Team Handbook (updated 2026-03-27), key changes include:

- **Renamed Tasks**: "Deploy" and "Restore" (we run them vision-only — no hydrophones on the sub this year)
- **New Time Bonus Requirement**: "Touch a buoy" added as prerequisite
- **Floating Structure**: Replaces previous "octagon" terminology
- **Weight Limit**: 60 kg DQ threshold
- **Time Bonus Formula**: `(whole_minutes + fractional_seconds/60) × 100 points`

### Time Bonus Requirements

All three conditions must be met to unlock the time bonus:

1. **Touch a buoy**
2. **Drop ≥1 marker in bin** OR **fire ≥1 torpedo through opening**
3. **Fully surface within the floating structure**

Every remaining minute on the clock = 100 points. A sub that unlocks the time bonus in 5 minutes earns 10 min × 100 = **1000 points** just from time remaining.

---

## Mission Strategy

SHRUB's mission is ordered to **unlock the time bonus as fast as possible**, then complete additional tasks for bonus points:

### Phase 1: Gate (Mandatory)
- Submerge and detect the start gate
- Identify animal markers (Reef Shark / Sawfish)
- Perform style maneuver (barrel roll)
- Pass through cleanly

**Budget: 90 seconds**

### Phase 2: Time Bonus Unlock (Critical Path)
1. **Touch Buoy** — Fast detection and touch (45s)
2. **Drop BRUVS** — Drop 1+ marker in correct bin half (75s)
3. **Surface in Float Area** — Navigate to floating structure and surface (90s)

**Budget: 210 seconds total**

### Phase 3: Bonus Tasks
- **Deploy** — Vision search + torpedo through openings (120s; no pinger this season)
- **Navigate the Channel** — Slalom through 3 pipe sets (90s)
- **Return Home** — Pass back through start gate (90s)

**Budget: 300 seconds**

### Reserve Buffer
**180 seconds** remain for time bonus accumulation (~300 points)

---

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────┐
│              SHRUB BehaviorTree Executive           │
│            (bt_executor + bt_mission.so)            │
└─────────────────┬───────────────────────────────────┘
                  │
    ┌─────────────┼─────────────┬──────────────┐
    ▼             ▼             ▼              ▼
┌────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
│ Safety │  │   Nav    │  │  Percep  │  │  Manip   │
│ Nodes  │  │  Nodes   │  │  Nodes   │  │  Nodes   │
└────┬───┘  └────┬─────┘  └────┬─────┘  └────┬─────┘
     │           │             │             │
     └───────────┴─────────────┴─────────────┘
                      │
         ┌────────────┴────────────┐
         ▼                         ▼
    ┌─────────┐              ┌─────────┐
    │ ROS 2   │              │ ROS 2   │
    │ Topics  │              │ Actions │
    └─────────┘              └─────────┘
```

### Design Principles

1. **Separation of Concerns**: Each node type handles one responsibility (perception, navigation, manipulation, etc.)
2. **Blackboard Communication**: Nodes share state via the BehaviorTree blackboard
3. **ROS 2 Integration**: All hardware interaction goes through ROS topics/actions/services
4. **Declarative Mission Definition**: Mission logic lives in XML, not C++ code
5. **Testability**: Nodes can be tested individually or in subtrees

---

## Project Structure

```
robosub-2026/
├── src/
│   ├── robosub2026/                  # SHRUB v4 — package bt_mission, exec bt_executor
│   │   ├── bt_xml/robosub2026_mission.xml   # The 2026 mission tree (root: MainTree)
│   │   ├── include/bt_mission/
│   │   │   ├── mission_io.hpp               # Singleton ROS I/O layer (pub/sub)
│   │   │   └── shrub_nodes.hpp              # All BT node declarations
│   │   ├── src/
│   │   │   ├── bt_executor.cpp              # Main: loads XML, seeds BB, ticks tree
│   │   │   ├── mission_io.cpp               # MissionIO impl
│   │   │   ├── safety_nodes.cpp             # 39 conditions
│   │   │   ├── nav_nodes.cpp                # Init + nav/movement actions
│   │   │   ├── perception_nodes.cpp         # Detect/Estimate/Search/Identify
│   │   │   ├── manipulation_nodes.cpp       # Marker/torpedo/gripper actuator wrappers
│   │   │   └── task_logic_nodes.cpp         # Task logic + registerAllNodes
│   │   ├── launch/shrub.launch.py
│   │   └── MIGRATION.md                     # v3→v4 notes, blackboard contract
│   ├── auv_msgs/                     # MovementCommand, NavigationCommand, ObjectDetection*, DepthInfo, BehaviorStatus
│   ├── mavlink_thruster_control/     # thruster_node (Pixhawk/ArduSub via pymavlink)
│   ├── control/                      # autonomous_controller (vision + localization → MovementCommand)
│   ├── localization/                 # localization_node + depth_node
│   ├── vision/                       # detector (TensorRT/YOLOv8), bottom_camera_node
│   ├── BehaviorTree.ROS2/            # cloned dep — provides behaviortree_ros2
│   ├── run_stack.sh                  # launches the full pipeline
│   └── test_movement.sh
├── JETSON_SETUP.md                   # Jetson runbook
├── build_engine.py                   # ONNX → TensorRT FP16 engine
├── convert_to_onnx.py                # .pt → ONNX
├── test_pipeline.py                  # Topic-level integration check
├── test_pixhawk.py                   # Arm + per-thruster bench test
├── build/ install/ log/              # gitignored
└── README.md
```

---

## Installation

### Prerequisites

- **Ubuntu 22.04 LTS** (recommended for ROS 2 Humble)
- **ROS 2 Humble Hawksbill** ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- **BehaviorTree.CPP v4** ([installation guide](https://www.behaviortree.dev/docs/intro))
- **BehaviorTree.ROS2** package
- **CMake 3.8+**
- **C++17 compiler** (GCC 9+ or Clang 10+)

### Install BehaviorTree.CPP

```bash
# Install dependencies
sudo apt update
sudo apt install -y libzmq3-dev libboost-dev libncurses5-dev qtbase5-dev libqt5svg5-dev libdw-dev

# Clone and build BehaviorTree.CPP
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

### Install BehaviorTree.ROS2

```bash
# In your ROS 2 workspace src/ directory
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
```

### Build SHRUB

```bash
# Clone this repository
git clone https://github.com/IntelligentLion/robosub-2026.git
cd robosub-2026

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

---

## Usage

### Launch the Mission Planner

```bash
# Source the workspace
source install/setup.bash

# Launch SHRUB with default mission
ros2 launch bt_mission shrub.launch.py

# Or run the executor directly
ros2 run bt_mission bt_executor
```

### Visualize with Groot2

[Groot2](https://www.behaviortree.dev/groot) is the official BehaviorTree.CPP visualizer and editor.

```bash
# Install Groot2 (AppImage)
wget https://github.com/BehaviorTree/Groot2/releases/latest/download/Groot2-x86_64.AppImage
chmod +x Groot2-x86_64.AppImage

# Launch Groot2
./Groot2-x86_64.AppImage

# Load the mission XML
# File → Load Tree → src/robosub2026/bt_xml/robosub2026_mission.xml
```

### Running Different Mission Modes

The mission XML includes three main behavior trees:

1. **SHRUB** (default): Full 15-minute competition run
2. **QualificationRun**: Simple gate pass for qualification
3. **PreQualRun**: Submerge → gate → circle marker → return

To switch trees, modify the executor source or pass a parameter via launch file.

---

## Depth Hold Tests

Two standalone scripts at the repo root **submerge the sub to a fixed depth and
then actively hold it**. They are bench/pool tools — no behavior tree, no
mission stack — used to validate that the vertical (heave) axis and depth
control work before a full run. They differ only in **where depth comes from**:

| Script | Depth source | Mode | Deps |
|--------|--------------|------|------|
| `depth_hold_pix_test.py` | Pixhawk pressure sensor (Bar30) | ArduSub **ALT_HOLD** | `pymavlink` only |
| `depth_hold_test.py` | ZED 2i positional tracking | **MANUAL** + in-script P loop | ROS 2 + ZED SDK + workspace |

> ⚠️ **SAFETY — these ARM the Pixhawk and spin the real vertical thrusters.**
> Stop `thruster_node` first (it is the single owner of the Pixhawk serial
> port). Remove or clear the props for a bench check, run on a tether, and keep
> the kill switch within reach. Both scripts disarm on `Ctrl+C`.

### `depth_hold_pix_test.py` — Pixhawk / pressure-sensor depth hold

This is the recommended pool test: it relies only on the flight controller, so
it works with no camera and no ROS workspace sourced.

#### Hardware / firmware notes (Pixhawk 2.4.8)

The 2.4.8 is an FMUv2 board running an **older ArduSub** build. Two
compatibility choices were made for it:

- **Stream request uses `REQUEST_DATA_STREAM`** (the deprecated but
  universally-supported method) as the primary path. The modern
  `MAV_CMD_SET_MESSAGE_INTERVAL` is *frequently ignored* on old firmware, so it
  is only sent as a best-effort extra. Depth (`SCALED_PRESSURE2`) rides the
  `EXTRA3` stream.
- All commands used (`SET_MODE`, `COMPONENT_ARM_DISARM`, `MANUAL_CONTROL`,
  `REQUEST_DATA_STREAM`, `HEARTBEAT`) exist in **MAVLink 1**, which old boards
  may default to — `pymavlink` negotiates this automatically.

**Requires a working depth/pressure sensor** (Bar30 or equivalent) configured
in ArduSub — ALT_HOLD has nothing to hold without one. If your depth sensor
reports on `SCALED_PRESSURE` (msg id 29) instead of `SCALED_PRESSURE2` (137),
change `PRESSURE_MSG_ID` and the `recv_match` type strings near the top of the
script.

#### Run from scratch

```bash
# 1. Make sure NOTHING else owns the Pixhawk serial port.
#    If the ROS stack is up, stop it (Ctrl+C the launch / kill thruster_node).

# 2. Confirm the flight controller device (USB shows up as /dev/ttyACM0).
ls /dev/ttyACM*            # or /dev/ttyUSB* on a telemetry radio

# 3. Install the one dependency (if not already present).
pip3 install pymavlink

# 4. Clear/remove the props. Sub in the water, sitting at the surface.

# 5. Run — submerge 3 ft, hold 20 s, surface, disarm.
python3 depth_hold_pix_test.py --depth 3 --hold-duration 20
#    It prints the plan and waits for you to type "go" (skip with --yes).
```

#### Parameters

| Flag | Default | Meaning |
|------|---------|---------|
| `--depth` | `3.0` | Target depth in **feet** below the surface baseline. |
| `--hold-duration` | `20.0` | Seconds to hold once the target depth is reached, before surfacing. |
| `--port` | `/dev/ttyACM0` | Flight-controller serial device. |
| `--baud` | `115200` | Serial baud (USB ignores this; matters on a telemetry radio — often `57600`). |
| `--kp` | `2.0` | Proportional gain: vertical-effort fraction per **metre** of depth error. |
| `--min-speed` | `0.15` | Minimum vertical effort (0–1) while moving, so it doesn't stall near target. |
| `--max-speed` | `0.6` | Maximum vertical effort (0–1) — cap on descent/ascent aggressiveness. |
| `--deadband` | `0.07` | Half-width (m) of the neutral band; inside it the stick centres and ALT_HOLD locks depth. |
| `--settle-tol` | `0.1` | Error (m) under which the target counts as "reached" (starts the hold timer). |
| `--max-depth` | `0.0` | Hard safety abort: surface if measured depth exceeds this (m). `0` → 2× target. |
| `--water-density` | `1000.0` | kg/m³ for the pressure→depth conversion (fresh ≈ 1000, salt ≈ 1025). |
| `--yes` | off | Skip the "type go" confirmation prompt. |

#### How the code works

1. **Connect & baseline** (`connect`, `request_pressure`, `latch_surface`):
   open the serial link, wait for a heartbeat, start the `EXTRA3` data stream,
   then record the **median surface pressure** as depth-zero while the sub is
   still floating. Depth is later computed as
   `depth = (press_abs − surface_press) / (ρ · g)`.
2. **Arm in ALT_HOLD** (`set_alt_hold`, `arm`): switch to ArduSub custom mode
   `2` (ALT_HOLD) and arm. In ALT_HOLD the vertical stick is a *climb-rate*
   command — a centred stick (`z = 500`) tells the autopilot to **hold the
   current depth** using its own pressure-sensor loop.
3. **P control loop @ 10 Hz** (`main` loop, `send_frame`, `drain_depth`):
   each tick reads the latest depth and computes `error = target − depth`.
   - `|error| ≤ deadband` → `z = 500` (hand the hold to ALT_HOLD).
   - too shallow (`error > 0`) → `z < 500` (descend).
   - too deep (`error < 0`) → `z > 500` (ascend).
   The effort is `clamp(kp·|error|, min_speed, max_speed)` scaled to the
   0–1000 stick range. As the error shrinks the rate tapers to centre, so the
   script smoothly hands depth-keeping over to ALT_HOLD rather than fighting
   it. Every frame also sends a **GCS heartbeat** so ArduSub's GCS-failsafe
   doesn't trip.
4. **Hold, surface, disarm**: once within `settle-tol` it holds for
   `--hold-duration`, then drives up until back inside the deadband of the
   surface, flushes neutral frames, and disarms. A **hard abort** surfaces and
   stops if depth ever exceeds `--max-depth`.

### `depth_hold_test.py` — ZED positional-tracking depth hold

Same goal, but depth comes from the **ZED 2i** positional tracking (vertical =
world `Y` axis) instead of the pressure sensor, and it runs the production
`mavlink_thruster_control.ThrusterController` in-process (MANUAL mode). Needs
the ROS 2 workspace sourced and the ZED SDK:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 depth_hold_test.py                 # submerge 3 ft, then hold
python3 depth_hold_test.py --depth 1.5     # different target (feet)
python3 depth_hold_test.py --external-vslam # subscribe to a vslam node you launch
```

Use the ZED version when you specifically want to validate vision-based depth
estimation; use the Pixhawk version for a quick, dependency-light vertical/
thruster check.

---

## Isolated Actions & Stage Tests

A family of small water-test scripts at the repo root for bringing the sub up
piece by piece: first **isolated actions** (one motion at a time), then
**stages** (short scripted sequences), with and without vision. They all share
one engine, [`field_common.py`](field_common.py), which runs the production
`mavlink_thruster_control.ThrusterController` in-process and publishes
`auv_msgs/MovementCommand` with **linear speed ramping**.

> ⚠️ **SAFETY — these ARM the Pixhawk and spin the real thrusters.** Stop
> `thruster_node` first (the scripts own the serial port in-process). Clear the
> props, run on a tether, keep the kill switch reachable. Every script
> stops + disarms on `Ctrl+C` and prompts for a typed `go` (skip with `--yes`).
> Source the workspace first: `source install/setup.bash`.

**Flight mode.** These tools run `ThrusterController` in **ALT_HOLD** (its
default), so ArduSub holds depth on the pressure sensor between and during
moves while the horizontal axes stay manual — you tune surge/strafe/yaw without
the sub sinking. ALT_HOLD needs a working depth sensor **and water**; on a dry
bench use the MANUAL-mode tools instead. The mode is the `flight_mode`
parameter on `ThrusterController` (`ThrusterController(flight_mode='MANUAL')`),
defaulting to `ALT_HOLD`. `depth_hold_test.py` (ZED) and the dry-bench tools
deliberately use MANUAL so an external controller owns depth.

### Shared tuning flags

**Every** movement script understands the same speed/ramp knobs, so you can
tune the motion profile of any action or stage the same way:

| Flag | Meaning |
|------|---------|
| `--speed` | Target effort, 0–1 (the headline tuning knob). |
| `--ramp-up` | Seconds to ramp linearly from 0 → `--speed` (smooth spin-up). |
| `--ramp-down` | Seconds to ramp `--speed` → 0 at the end of a move. |
| `--duration` | Seconds to **hold** at target speed (between the ramps). |
| `--pause` | Seconds of neutral depth-hold after the action ("…and pause"). |
| `--yes` | Skip the `go` confirmation prompt. |

Ramping is the answer to "tune the speed **and ramping up** of speed": the
thrusters walk from 0 to `--speed` over `--ramp-up`, hold for `--duration`,
then ease back down over `--ramp-down`.

### Isolated actions

| Script | Action | Vision? |
|--------|--------|---------|
| [`act_forward.py`](act_forward.py) | Move forward, then pause | no |
| [`act_turn_left.py`](act_turn_left.py) | Turn left (yaw CCW), then pause | no |
| [`act_strafe_left.py`](act_strafe_left.py) | Strafe left, then pause | no |
| [`act_strafe_right.py`](act_strafe_right.py) | Strafe right, then pause | no |
| [`act_center_gate.py`](act_center_gate.py) | Closed-loop yaw to centre on the gate, then pause | **yes** (spawns detector) |
| [`act_coords.py`](act_coords.py) | Print ZED x/y/z pose — **no arming**, pure readout | ZED only |
| [`depth_hold_test.py`](depth_hold_test.py) | Submerge + hold depth via **ZED** P-controller | ZED |
| [`depth_hold_pix_test.py`](depth_hold_pix_test.py) | Submerge + hold depth via **Pixhawk** baro / ALT_HOLD | no |

```bash
python3 act_forward.py      --speed 0.4 --ramp-up 1.5 --duration 3 --pause 2
python3 act_turn_left.py    --speed 0.3 --duration 6          # tune for ~90°
python3 act_strafe_left.py  --speed 0.35 --duration 3
python3 act_strafe_right.py --speed 0.35 --duration 3
python3 act_center_gate.py  --speed 0.3 --gain 0.6 --tol 0.08
python3 act_coords.py                                          # safe, no thrusters
```

`act_center_gate.py` reads the gate's normalised image centre-x from
`vision/detections` and yaws with effort proportional to the centring error
(clamped to `[--min-speed, --speed]`) until within `--tol` or `--timeout`.

### Stages — movements only (no vision)

Timed descent, then the scripted motion. Use these to rehearse a run's motion
profile and tune speeds/ramps before trusting detection.

| Script | Sequence |
|--------|----------|
| [`stage_gate.py`](stage_gate.py) | submerge → pause → forward through gate |
| [`stage_marker.py`](stage_marker.py) | submerge → pause → around-marker maneuver |

```bash
python3 stage_gate.py   --submerge-speed 0.4 --submerge-duration 4 \
                        --speed 0.4 --duration 5
python3 stage_marker.py --submerge-duration 4 --speed 0.35 \
                        --leg-duration 3 --turn-duration 6
```

The around-marker maneuver is open-loop: strafe right → forward → turn left →
forward → turn left → forward, with `--leg-duration` per straight/strafe leg
and `--turn-duration` per ~90° turn.

### Stages — movements + detection

Same sequences, but the descent runs **until the target is detected** on
`vision/detections` or a timeout expires. These spawn the TensorRT detector
in-process (one command does everything).

| Script | Sequence |
|--------|----------|
| [`stage_gate_detect.py`](stage_gate_detect.py) | submerge **until gate detected / timeout** → pause → forward through gate |
| [`stage_marker_detect.py`](stage_marker_detect.py) | submerge **until marker detected / timeout** → pause → around-marker |

```bash
python3 stage_gate_detect.py   --submerge-speed 0.4 --gate-timeout 25 \
                               --speed 0.4 --duration 5
python3 stage_marker_detect.py --submerge-speed 0.4 --marker-timeout 25 \
                               --speed 0.35 --leg-duration 3 --turn-duration 6
```

`--conf` sets the minimum detection confidence; `--label` overrides the
detection class (`gate` / `marker` by default).

---

## Behavior Tree Structure

The mission tree is declarative XML at
[`src/robosub2026/bt_xml/robosub2026_mission.xml`](src/robosub2026/bt_xml/robosub2026_mission.xml)
(root: `MainTree`). Top-level sequence:

```
InitializeSystems → HeadingOutAndGate → SlalomTask → ReconBinsTask
   → DeployTorpedoesTask → ResupplyOctagonTask → ReturnHomeTask
   → Surface → MissionComplete
```

Each task expands into subtrees per phase (search → align → act → verify).
A `GlobalRecovery` subtree handles localization loss, depth instability,
obstacles, task timeouts, and critical failure → surface safely.

For the full per-task structure, open the XML in
[Groot2](https://www.behaviortree.dev/groot/) or read it directly — it is
intentionally human-readable.

---

## Custom Node Categories

The full list (39 conditions, ~130 actions) is declared in
[`include/bt_mission/shrub_nodes.hpp`](src/robosub2026/include/bt_mission/shrub_nodes.hpp)
and implemented across
[`safety_nodes.cpp`](src/robosub2026/src/safety_nodes.cpp),
[`nav_nodes.cpp`](src/robosub2026/src/nav_nodes.cpp),
[`perception_nodes.cpp`](src/robosub2026/src/perception_nodes.cpp),
[`manipulation_nodes.cpp`](src/robosub2026/src/manipulation_nodes.cpp),
and [`task_logic_nodes.cpp`](src/robosub2026/src/task_logic_nodes.cpp).
Categories:

- **Safety conditions** — battery, leak, depth, time-remaining, localization,
  task timeout, critical failure, plus role/coin-flip/state-flag predicates.
- **Initialization** — self-test stubs + `SubmergeToMissionDepth`.
- **Navigation / movement** — hold-depth/heading/roll/pitch, yaw sweeps,
  open-loop forward bursts, vision-guided `AlignGateCenter`/`AlignThroughGap`,
  bin/basket/octagon transits, surface/recovery.
- **Perception** — `Detect_*`, `Search*`, `Identify*`, `Estimate*` wrappers
  around `MissionIO::bestDetection(label, conf)`. Vision label vocabulary in
  [MIGRATION.md](src/robosub2026/MIGRATION.md).
- **Manipulation** — marker drop, torpedo launch, gripper release, magnetic
  tool. Update blackboard counters today; hardware drivers TODO.
- **Task logic** — `CalculateRotationCount` for the octagon bonus.

> Pinger-based navigation has been **removed** — no hydrophones this season.
> Torpedo and octagon tasks enter via vision-only search.

---

## Development

### Adding a New Node

1. **Declare** the node class in `include/bt_mission/shrub_nodes.hpp`
2. **Implement** the node logic in the appropriate `src/*_nodes.cpp` file
3. **Register** the node in `registerAllNodes()` function
4. **Add** to `TreeNodesModel` section in `bt_xml/robosub2026_mission.xml`
5. **Use** the node in your behavior tree XML

#### Example: Adding a "DetectCoral" Node

**shrub_nodes.hpp:**
```cpp
DECLARE_DETECT_NODE(Detect_coral,
  BT::OutputPort<std::string>("result"))
```

**perception_nodes.cpp:**
```cpp
BT::NodeStatus Detect_coral::onStart() {
  // Subscribe to coral detection topic
  // ...
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Detect_coral::onRunning() {
  // Check for new detection
  if (coral_detected) {
    setOutput("result", detection_msg);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void Detect_coral::onHalted() {
  // Cleanup
}
```

**Register in bt_executor.cpp or registration function:**
```cpp
factory.registerNodeType<shrub::Detect_coral>("Detect_coral");
```

**Add to TreeNodesModel in XML:**
```xml
<Action ID="Detect_coral">
  <output_port name="result"/>
</Action>
```

**Use in tree:**
```xml
<Detect_coral result="{coral_det}"/>
```

### Python: no rebuild needed (`--symlink-install`)

Build the workspace **once** with `--symlink-install` and the `install/` tree
points at the Python sources in `src/` instead of copying them:

```bash
colcon build --symlink-install
```

After that, editing an **existing** `.py` file (nodes, the depth/field-test
tools, prequal logic) takes effect on the next `ros2 run` / script launch with
**no rebuild** — just re-source is not even needed. You only need to rebuild
when you add a **new** file, change `entry_points`/`package.xml`, edit installed
data (launch/config/YAML), or touch C++ (`bt_mission`).

> If a code change isn't taking effect, you almost certainly have a **stale
> `install/`** from a non-symlink build (files copied, not linked) — this is the
> same trap that hid earlier commits during a water test. Re-run the
> `--symlink-install` build above once to convert it.

### Testing

#### Unit Testing Individual Nodes
```bash
# Build tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select bt_mission
colcon test-result --verbose
```

#### Integration Testing with Gazebo
(Assuming a separate Gazebo simulation setup exists)

```bash
# Launch simulation
ros2 launch auv_gazebo pool_world.launch.py

# Launch SHRUB
ros2 launch bt_mission shrub.launch.py
```

### Code Style

- **C++ Standard**: C++17
- **Formatting**: Follow ROS 2 style guide
- **Naming**:
  - Classes: `PascalCase`
  - Functions: `snake_case`
  - Variables: `snake_case`
  - Constants: `UPPER_SNAKE_CASE`

### Git Workflow

```bash
# Create feature branch
git checkout -b feature/new-task

# Make changes, commit
git add .
git commit -m "Add coral detection node"

# Push to remote
git push origin feature/new-task

# Create pull request on GitHub
```

---

## Mission Execution Checklist

Before a competition run:

- [ ] Check battery charge (≥80% recommended)
- [ ] Test leak sensors
- [ ] Calibrate IMU and depth sensor
- [ ] Verify camera feeds (forward, down, manipulator cams)
- [ ] Test torpedo firing mechanism
- [ ] Test gripper open/close
- [ ] Test marker drop solenoids
- [ ] Load correct mission XML (`bt_xml/robosub2026_mission.xml`)
- [ ] Set initial blackboard values via launch params (`coin_flip`, `role`, `gate_red_side`, `style_enabled`)
- [ ] Review safety monitor thresholds
- [ ] Dry-run in test tank if available

---

## Troubleshooting

### Behavior Tree Won't Load
- Check XML syntax with `xmllint`:
  ```bash
  xmllint --noout src/robosub2026/bt_xml/robosub2026_mission.xml
  ```
- Ensure all nodes in XML are registered in `registerAllNodes()`

### Node Stuck in RUNNING
- Add debug logging to `onRunning()` to see what condition isn't met
- Check if ROS topic/action server is available:
  ```bash
  ros2 topic list
  ros2 action list
  ```

### Safety Monitor Triggering Early
- Review safety thresholds in `SafetyMonitor` subtree
- Check sensor calibration (depth, battery voltage)

### Poor Detection Performance
- Verify camera exposure, focus
- Check lighting conditions
- Review detection confidence thresholds
- Collect more training data for ML models

---

## Competition Day Tips

1. **Arrive Early**: Set up, calibrate, and test 2+ hours before your slot
2. **Backup Plan**: Have a minimal "qualification-only" tree ready
3. **Monitor Logs**: Watch `ros2 topic echo /bt_executor/status` for tree state
4. **Time the Run**: Practice with a 15-minute timer
5. **Fail Fast**: If a task isn't working, let ForceSuccess skip it and move on
6. **Surface Strategy**: Confirm float area before surfacing (breach = DQ)
7. **Judges**: Make movements deliberate and visible for scoring
8. **Document Everything**: Record video, save logs for post-analysis

---

## Resources

### Official Documentation
- [RoboSub 2026 Handbook](https://robonation.org/programs/robosub/) - Competition rules
- [BehaviorTree.CPP Docs](https://www.behaviortree.dev/) - BT framework
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/) - Robot middleware

### Learning Resources
- [BehaviorTree.CPP Tutorial](https://www.behaviortree.dev/tutorial_01_first_tree/) - Getting started
- [Groot2 User Guide](https://www.behaviortree.dev/groot/) - Tree visualization
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html) - ROS basics

### Team Resources
- [Team Website](https://intelligentlion.org) - IntelligentLion homepage
- [GitHub Organization](https://github.com/IntelligentLion) - All team repositories

---

## License

MIT License

Copyright (c) 2026 IntelligentLion

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---

## Acknowledgments

- **RoboNation** for organizing the RoboSub competition
- **BehaviorTree.CPP** maintainers for the excellent framework
- **Open Robotics** for ROS 2
- **IntelligentLion Team Members** for countless hours of design, coding, and testing
- **Sponsors and Mentors** for supporting our journey

---

**Built with passion by Team IntelligentLion for RoboSub 2026**

*For questions or collaboration opportunities, contact: team@intelligentlion.org*
