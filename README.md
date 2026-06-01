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
