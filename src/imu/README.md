# imu — ZED 2i IMU orientation visualization

Standalone RViz visualization of the RoboSub's orientation from the ZED 2i
built-in IMU. Connects to nothing else in the stack — it owns its own TF tree
(`map -> odom -> base_link -> imu_link`) and consumes only the ZED IMU topic.

## Build

```bash
cd /home/robosub/robosub2026/robosub-2026
colcon build --symlink-install --packages-select imu
source install/setup.bash
```

## Run (one command)

```bash
ros2 launch imu imu_viz.launch.py
```

This pre-kills any running ZED node (fresh fused orientation, no accumulation),
starts a fresh `zed2i` camera, the three nodes, and RViz. Pick up the sub and
rotate it — `base_link` rotates live in RViz with axes, IMU box, and vector
arrows; the diagnostics block prints in the terminal at 10 Hz.

### Options

```bash
# serial is hardcoded to the FRONT-facing zed2i (31166146).
# NEVER use 30758628 — that is the bottom-facing camera.

# attach to an already-running ZED (skip pre-kill + camera bringup)
ros2 launch imu imu_viz.launch.py start_zed:=false

# no RViz (headless)
ros2 launch imu imu_viz.launch.py rviz:=false
```

## Re-zero orientation live

The orientation is zeroed to a reference captured at startup. To re-zero
(e.g. after re-mounting), call:

```bash
ros2 service call /imu/reset_orientation std_srvs/srv/Trigger {}
```

## Nodes

| Node | Output |
|------|--------|
| `orientation_node` | TF `odom->base_link` (zeroed quat), static `map->odom` + `base_link->imu_link`, `imu/rpy` |
| `diagnostics_node` | 10 Hz orientation/quaternion/gyro/accel text block |
| `marker_node` | `imu/markers` — forward/up/right/gravity/gyro/accel arrows |

## Future fusion

All output is standard ROS messages. When adding robot_localization EKF, the
`odom->base_link` transform this package publishes is exactly what the EKF will
own — swap the broadcaster for the EKF and feed BAR02 depth / DVL / mag / VSLAM
as additional EKF inputs. No changes needed to the ZED IMU driver.
