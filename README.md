# RoboSub 2026 - SHRUB v4 Mission Planner

**SHRUB (Software for Handling and Regulating Underwater Behavior)** is an autonomous mission planner for the RoboSub 2026 competition, developed by Team IntelligentLion. Built on BehaviorTree.CPP v4 and ROS 2 Humble, SHRUB coordinates all high-level decision-making for our autonomous underwater vehicle (AUV).

> **⚠️ Repo status (read before running):** SHRUB v4 lives in `src/robosub2026/`
> (package `bt_mission`) and is the canonical planner going forward, but its BT
> nodes are still being ported from the legacy `src/mission/` package and are
> **not yet pool-verified**. Until the port is done, `src/run_stack.sh` launches
> the working **legacy** `mission/bt_runner`. See
> [`src/robosub2026/MIGRATION.md`](src/robosub2026/MIGRATION.md) for the plan and
> current state. VSLAM is intentionally disabled for competition — see
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

- **Pinger Tasks Renamed**: "Deploy" and "Restore" (pinger-only navigation, no path markers)
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
- **Deploy** — Pinger navigation + torpedo through openings (120s)
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
│   ├── robosub2026/              # Main SHRUB package (bt_mission)
│   │   ├── bt_xml/
│   │   │   └── robosub2026_mission.xml    # Main behavior tree definition
│   │   ├── include/
│   │   │   └── bt_mission/
│   │   │       └── shrub_nodes.hpp        # All node declarations
│   │   ├── src/
│   │   │   ├── bt_executor.cpp            # Main entry point
│   │   │   ├── safety_nodes.cpp           # Battery, leak, depth, time checks
│   │   │   ├── nav_nodes.cpp              # Movement and navigation
│   │   │   ├── perception_nodes.cpp       # Vision and sensor detection
│   │   │   ├── manipulation_nodes.cpp     # Gripper, torpedo, marker drop
│   │   │   └── task_logic_nodes.cpp       # High-level task coordination
│   │   ├── launch/
│   │   │   └── shrub.launch.py            # ROS 2 launch file
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── auv_msgs/                 # Custom message definitions
│       └── msg/
│           └── ObjectDetectionArray.msg
├── build/                        # Build artifacts (gitignored)
├── install/                      # Install space (gitignored)
├── log/                          # Build/runtime logs (gitignored)
└── README.md                     # This file
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

### Top Level: SHRUB

```xml
<BehaviorTree ID="SHRUB">
  <ReactiveSequence>
    <SubTree ID="SafetyMonitor" ... />
    <SubTree ID="MissionExecution" ... />
  </ReactiveSequence>
</BehaviorTree>
```

**ReactiveSequence**: Re-evaluates `SafetyMonitor` every tick. If safety fails, mission halts immediately.

### SafetyMonitor Subtree

Checks (in order):
1. **Battery ≥ 20%** → else emergency surface
2. **No leak detected** → else emergency surface
3. **Depth ≤ 1.9m** (off pool floor) → else ascend to 1.2m
4. **Breach prevention**: If depth < 0.15m and NOT inside float area → descend to 0.5m
5. **Time remaining ≥ 30s** → else emergency surface

### MissionExecution Subtree

```xml
<Sequence>
  <!-- Phase 1: Gate (mandatory) -->
  <Timeout msec="90000">
    <SubTree ID="HeadingOut_and_Gate" />
  </Timeout>

  <!-- Phase 2: Time Bonus Unlock -->
  <SequenceWithMemory name="time_bonus_unlock">
    <ForceSuccess><SubTree ID="TouchBuoy" /></ForceSuccess>
    <ForceSuccess><SubTree ID="DropBRUVS" /></ForceSuccess>
    <ForceSuccess><SubTree ID="SurfaceInFloatArea" /></ForceSuccess>
  </SequenceWithMemory>

  <!-- Phase 3: Additional Points -->
  <SequenceWithMemory name="bonus_tasks">
    <ForceSuccess><SubTree ID="Deploy" /></ForceSuccess>
    <ForceSuccess><SubTree ID="NavigateTheChannel" /></ForceSuccess>
    <ForceSuccess><SubTree ID="ReturnHome" /></ForceSuccess>
  </SequenceWithMemory>
</Sequence>
```

**SequenceWithMemory**: Remembers completed children, doesn't re-run them on retry.
**ForceSuccess**: Wraps a task so failure doesn't block the rest of the mission.
**Timeout**: Hard deadline for each task.

---

## Custom Node Categories

### Condition Nodes

| Node | Purpose |
|------|---------|
| `IsBatteryOk` | Check battery percentage above threshold |
| `IsLeakDetected` | Check for hull leak |
| `IsDepthSafe` | Validate depth within min/max bounds |
| `IsInsideFloatArea` | Check if positioned under floating structure |
| `IsTimeRemaining` | Check if enough mission time left |
| `AlwaysSuccess` | Utility node for control flow |

### Navigation Nodes

| Node | Purpose |
|------|---------|
| `Submerge` | Descend to target depth |
| `AscendTo` | Rise to target depth |
| `EmergencySurface` | Immediate surface (safety abort) |
| `Turn` | Rotate by specified degrees |
| `Navigate_to` | Move to named waypoint |
| `Navigate_to_bearing` | Follow pinger bearing |
| `Navigate_on_heading` | Travel on compass heading for distance |
| `Move_through_gate` | Execute gate-passing maneuver |
| `Reposition_to_gate_side` | Shift left/right relative to gate |
| `Record_heading` | Save current heading to blackboard |
| `Recalibrate_nav` | Reset IMU drift |
| `Hold_depth` | Maintain depth within tolerance |
| `Surface_in_float_area` | Surface only when confirmed inside |
| `Stabilize` | Hold position for specified milliseconds |
| `Wait` | Pause for specified milliseconds |

### Perception Nodes

All perception nodes write detections to the blackboard for downstream consumers.

| Node | Outputs |
|------|---------|
| `Detect_gate` | Gate bounding box/pose |
| `Detect_animal_on_gate` | Animal type, side, confidence |
| `Detect_buoy` | Buoy detection |
| `Detect_bin_below` | Bin location, correct animal half |
| `Detect_pinger` | Bearing to acoustic pinger |
| `Detect_float_area_below` | Floating structure edges |
| `Detect_task_board` | Task board for torpedoes |
| `Detect_slalom_pipes` | Pipe positions |
| `Detect_object` | Trash objects (bottle/ladle) |
| `Detect_path_marker` | Colored path markers |

### Alignment Nodes

Visual servoing nodes that center the AUV on detected targets.

| Node | Purpose |
|------|---------|
| `Align_to` | Center on detection (XY plane) |
| `Align_above` | Position above bin half |
| `Align_to_opening` | Aim at torpedo opening |
| `Align_to_basket` | Aim at trash basket |
| `Center_beneath` | Position beneath floating structure |
| `Approach_and_touch` | Close distance + make contact |

### Manipulation Nodes

| Node | Purpose |
|------|---------|
| `Grab_object` | Close gripper on object |
| `Release_object` | Open gripper |
| `Drop_marker` | Release BRUVS marker |
| `Fire_torpedo` | Launch torpedo from tube |

### Task Logic Nodes

| Node | Purpose |
|------|---------|
| `Style_through_gate` | Barrel roll maneuver |
| `Determine_basket` | Compute correct basket for object (opposite color rule) |
| `Compute_slalom_path` | Plan path through pipes |
| `Rotation_bonus` | Rotate N times on surface |
| `Increment` | Increment blackboard counter |

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
- [ ] Confirm pinger frequencies (25-40 kHz)
- [ ] Load correct mission XML
- [ ] Set initial blackboard values (preferred_animal, coin_flip, etc.)
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
