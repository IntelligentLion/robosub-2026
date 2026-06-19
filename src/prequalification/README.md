# prequalification

Scripted **RoboSub 2026 prequalification** run for the AUV. A single ROS 2
node (`prequalification_node`) walks a fixed, vision-triggered state machine
and publishes low-level `auv_msgs/MovementCommand` on `movement_command` —
the same topic the thruster controller (`mavlink_thruster_control`) consumes.
No PID / localization stack is required: every state has a vision trigger plus
timed/spatial safety fallbacks, so the run always completes and surfaces.

## Mission sequence

| # | State | Action | Advance when | Fallback |
|---|-------|--------|--------------|----------|
| 1 | `submerge` | Descend until gate seen | gate detected (pause) | `submerge_timeout_s` **or** depth ≥ `max_depth_m` → hold depth + drive forward |
| 2 | `submerge_clear_top` | Dive until gate top leaves frame, +`gate_top_clear_extra_s` | top edge ≤ `gate_top_clear_y` | `submerge_clear_top_timeout_s` / `max_depth_m` |
| 3 | `through_gate_1` | Approach + drive through gate | passed (timed) | `through_gate_timeout_s` |
| 4 | `forward_to_marker` | Surge forward | marker detected (pause) | `forward_to_marker_timeout_s` **or** travelled ≥ `max_forward_distance_m` → start maneuver |
| 5 | `strafe_marker_left` | Strafe right | marker centre-x ≤ `marker_left_threshold` | `strafe_timeout_s` |
| 6 | `forward_past_marker` | Surge forward | marker passes out of view | `forward_past_marker_timeout_s` |
| 7 | `turn_left_1` | Turn left until marker is left | marker centre-x ≤ threshold (cap ~90°) | `turn_timeout_s` |
| 8 | `forward_marker_behind_1` | Surge forward | marker behind | `forward_marker_behind_timeout_s` |
| 9 | `turn_left_2` | Turn left until marker is left | marker centre-x ≤ threshold (cap ~90°) | `turn_timeout_s` |
| 10 | `strafe_to_gate` | Strafe left | gate detected (pause) | `strafe_to_gate_timeout_s` |
| 11 | `align_gate` | Centre on gate | gate centred | `align_gate_timeout_s` |
| 12 | `through_gate_2` | Drive through gate | passed (timed) | `through_gate_timeout_s` |
| 13 | `final_forward` | Surge a little more to clear | `final_forward_duration_s` | `final_forward_timeout_s` |
| 14 | `surface` | Ascend | at surface → `done` | `surface_timeout_s` |

Every state also has a per-state safety timeout (see
`config/prequalification.yaml`); on timeout the machine advances rather than
stalling. The two headline fallbacks are spelled out above: **submerge** gives
up on the gate by *time or depth*, and **forward-to-marker** gives up on the
marker by *time or distance travelled* (distance needs `localization/pose`).

**Depth hold:** the only states that move the vertical axis are `submerge`,
`submerge_clear_top`, and `surface`. Once the descent ends, every other state
**actively holds** the depth it reached — a small proportional controller
re-commands `submerge`/`emerge` off `depth/sub_depth` error (deadband
`depth_hold_tol_m`), independent of the surge/strafe/yaw the state commands. So
depth does not drift while driving through the gate or circling the marker; it
only changes when the sub is deliberately submerging or surfacing. Set
`enable_depth_hold: false` to revert to passive neutral-thrust holding.

## Interfaces

**Publishes**
- `movement_command` (`auv_msgs/MovementCommand`) — thruster commands.

**Subscribes**
- `vision/detections` (`auv_msgs/ObjectDetectionArray`) — detector output.
- `depth/sub_depth` (`std_msgs/Float32`) — depth below surface.
- `localization/pose` (`geometry_msgs/PoseStamped`) — *optional*; enables the
  ~90° turn cap (closed-loop yaw) and the distance-travelled fallback in
  `forward_to_marker`. Without it, turns cap on `turn_90_duration_s` and the
  marker fallback is time-only.

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

# Tune the run (every param, incl. all timeouts, is a launch arg):
ros2 launch prequalification prequalification.launch.py \
    max_depth_m:=1.8 marker_label:=slalom_pole \
    submerge_timeout_s:=15 forward_to_marker_timeout_s:=40 \
    max_forward_distance_m:=10

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

All behaviour is parameterised. Change it **two ways**:

1. **VSCode** — edit `config/prequalification.yaml` (grouped, with the
   per-state timeouts collected at the bottom), then rebuild:
   `colcon build --packages-select prequalification`.
2. **Terminal** — every value is also a launch argument that overrides the
   YAML, e.g. `... submerge_timeout_s:=15 max_depth_m:=1.8`. The exposed list
   lives in `PARAM_ARGS` at the top of `launch/prequalification.launch.py`.

The ones you will almost always touch:

- **`gate_label` / `marker_label`** — must match the class names your trained
  detector emits. The prequal "vertical marker" maps to whatever your model
  calls the pole (`marker`, `slalom_pole`, …). See the label vocabulary in
  [`robosub2026/MIGRATION.md`](../robosub2026/MIGRATION.md).
- **`max_depth_m`** — descent safety cap / depth fallback for `submerge`.
- **Per-state `*_timeout_s`** and **`max_forward_distance_m`** — the safety
  fallbacks (see the Mission sequence table).

Speeds, detection thresholds, the "marker is left" threshold, gate-pass and
gate-top-clear behaviour, and turn calibration are all exposed too.

> **Safety:** `prequalification_node` and `control/autonomous_controller`
> both publish `movement_command`. Run **only one** of them at a time.
