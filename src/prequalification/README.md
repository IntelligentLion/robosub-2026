# prequalification

Scripted **RoboSub 2026 prequalification** run for the AUV. A single ROS 2
node (`prequalification_node`) walks a fixed, vision-triggered state machine
and publishes low-level `auv_msgs/MovementCommand` on `movement_command` —
the same topic the thruster controller (`mavlink_thruster_control`) consumes.
No PID / localization stack is required: every state has a vision trigger plus
a timed safety fallback, so the run always completes and surfaces.

## Mission sequence

| # | State | Action | Advance when |
|---|-------|--------|--------------|
| 1 | `submerge` | Descend | depth ≥ `target_depth_m` |
| 2 | `detect_gate` | Hold depth, centre on gate | gate centred |
| 3 | `through_gate_1` | Approach + drive through gate | passed (timed) |
| 4 | `forward_to_marker` | Surge forward | marker detected |
| 5 | `strafe_marker_left` | Strafe right | marker centre-x ≤ `marker_left_threshold` |
| 6 | `forward_past_marker` | Surge forward | marker passes out of view |
| 7 | `turn_left_1` | Turn left 90° | heading reached / timed |
| 8 | `forward_marker_behind_1` | Surge forward | marker behind |
| 9 | `turn_left_2` | Turn left 90° (face gate) | heading reached / timed |
| 10 | `forward_marker_behind_2` | Surge forward | marker behind again |
| 11 | `align_gate` | Centre on gate | gate centred |
| 12 | `through_gate_2` | Drive through gate | passed (timed) |
| 13 | `surface` | Ascend | at surface → `done` |

Every state also has a per-state timeout (see `config/prequalification.yaml`);
on timeout the machine advances to the next state rather than stalling.

## Interfaces

**Publishes**
- `movement_command` (`auv_msgs/MovementCommand`) — thruster commands.

**Subscribes**
- `vision/detections` (`auv_msgs/ObjectDetectionArray`) — detector output.
- `depth/sub_depth` (`std_msgs/Float32`) — depth below surface.
- `localization/pose` (`geometry_msgs/PoseStamped`) — *optional*; enables
  closed-loop 90° turns. Without it, turns fall back to calibrated timed turns
  (`turn_90_duration_s`).

Detection coordinates follow `vision/detector.py`: `position.x/y` are the bbox
centre normalised to `[0, 1]` (0.5 = image centre), `position.z` is
range-to-target in metres (`-1` if unknown), and `bbox_width/height` are
normalised.

## Build

```bash
cd ~/robosub2026/robosub-2026
colcon build --packages-select prequalification
source install/setup.bash
```

## Run

Full deployment stack (vision + thrusters + mission):

```bash
ros2 launch prequalification prequalification.launch.py
```

Common overrides:

```bash
# Dry run on the bench — simulate thrusters, no camera, never command motors:
ros2 launch prequalification prequalification.launch.py \
    simulate:=true include_vision:=false publish_commands:=false

# Tune the run:
ros2 launch prequalification prequalification.launch.py \
    target_depth_m:=1.2 marker_label:=slalom_pole

# If vision / thrusters are already running, just start the mission node:
ros2 launch prequalification prequalification.launch.py \
    include_vision:=false include_thrusters:=false
```

Or run the node directly:

```bash
ros2 run prequalification prequalification_node \
    --ros-args --params-file \
    install/prequalification/share/prequalification/config/prequalification.yaml
```

## Tuning

All behaviour is parameterised in `config/prequalification.yaml`. The two you
will almost always touch:

- **`gate_label` / `marker_label`** — must match the class names your trained
  detector emits. The prequal "vertical marker" maps to whatever your model
  calls the pole (`marker`, `slalom_pole`, …). See the label vocabulary in
  [`robosub2026/MIGRATION.md`](../robosub2026/MIGRATION.md).
- **`target_depth_m`** — run depth.

Speeds, detection thresholds, the "marker is left" threshold, gate-pass
duration, turn calibration, and per-state timeouts are all exposed there too.

> **Safety:** `prequalification_node` and `control/autonomous_controller`
> both publish `movement_command`. Run **only one** of them at a time.
