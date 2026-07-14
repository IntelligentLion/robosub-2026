# ZED IMU Orientation Visualization — Design

Date: 2026-07-13
Status: Approved (pending written-spec review)

## Purpose

Live RViz visualization of the RoboSub's orientation and motion driven by the
ZED 2i camera's built-in IMU. Pick up the sub, rotate/tilt it, and immediately
see the vehicle frame, axes, and motion vectors update smoothly in RViz.

Standalone visualization/diagnostics layer. It **connects to nothing else** in
the stack (no EKF, no localization coupling) — it owns its own TF tree and
consumes only the ZED IMU topic.

## Scope decisions (from brainstorming)

- **IMU source:** ZED 2i built-in IMU. The `zed-ros2-wrapper` is already the
  IMU *driver* — it reads the hardware and publishes `sensor_msgs/Imu` +
  `sensor_msgs/MagneticField` at ~100–400 Hz with quaternion, angular velocity,
  linear acceleration, and covariances. **We do not re-implement the driver.**
  This work is the consumer / orientation / TF / marker / diagnostics layer.
- **Hardware only.** No synthetic/simulation fallback.
- **Camera:** model `zed2i`, `camera_name = zed2i`.
  - Topics: `/zed2i/zed_node/imu/data` (Imu), `/zed2i/zed_node/imu/mag`
    (MagneticField). Frame: `zed2i_imu_link`.
  - Two physical ZED 2i cameras exist → expose a `serial_number` launch arg to
    select which one; `camera_name` also overridable.
- **Package:** new isolated ROS 2 package `src/imu` (keeps it decoupled).
- **"Reset + auto-calibrate, no accumulation":**
  1. *Fresh ZED each launch (hardware reset).* The launch **pre-kills** any
     running `zed_wrapper` node, then starts a clean one → fused orientation
     starts from scratch, nothing carried across runs. Honors the single-owner
     rule from `reset_zed_node.sh`.
  2. *Relative-to-startup zeroing (auto-calibrate).* On startup the first ~1 s
     of IMU samples establish a **reference quaternion**. The published
     `odom → base_link` transform is the current orientation *relative to that
     reference*, so the sub reads level/identity at start and drift is measured
     from there. A `/imu/reset_orientation` (`std_srvs/Trigger`) service
     re-zeros live at any time.

## Architecture

New package `src/imu` with three rclpy nodes, a launch file, and an RViz config.

### TF tree

```
map
 └── odom                (static, identity)
      └── base_link       (DRIVEN by IMU quaternion, relative to startup reference)
           └── imu_link    (static, physical IMU mounting offset)
```

- `map → odom`: static identity broadcast (placeholder for future global frame).
- `odom → base_link`: dynamic. Carries the live, zeroed IMU quaternion. Putting
  the orientation here makes the whole `base_link` subtree (and any robot model
  or markers) rotate in RViz as the sub is turned. **This is the transform that
  produces the visible rotation.**
- `base_link → imu_link`: static mounting transform (identity by default;
  overridable params for real mount orientation). Kept static so markers anchored
  to `base_link` stay fixed to the hull.

### Node 1 — `orientation_node`

- Subscribes `/zed2i/zed_node/imu/data` (`sensor_msgs/Imu`), QoS
  `SensorDataQoS` (best-effort) to match the ZED publisher.
- Captures a startup **reference quaternion** `q_ref` by averaging the first
  `calib_samples` (~1 s worth). Until calibrated, publishes identity.
- Per message:
  - Normalize the incoming quaternion.
  - Compute the zeroed orientation `q_out = q_ref⁻¹ ⊗ q_current` (quaternion
    conjugate of the reference composed with current — pure quaternion math, no
    Euler intermediary, so no gimbal-lock introduced).
  - Broadcast `odom → base_link` with `q_out` via `tf2_ros.TransformBroadcaster`.
  - Publish roll/pitch/yaw on `imu/rpy` (`geometry_msgs/Vector3Stamped`, radians)
    computed with `euler_from_quaternion` — for diagnostics/downstream only, TF
    itself stays quaternion-based.
- Broadcasts `map → odom` and `base_link → imu_link` via
  `tf2_ros.StaticTransformBroadcaster` once at startup.
- Service `/imu/reset_orientation` (`std_srvs/Trigger`) → recapture `q_ref`.
- Params: `imu_topic`, `parent_frame` (odom), `child_frame` (base_link),
  `imu_frame` (imu_link), `calib_samples`, `mount_rpy` (imu_link offset).

### Node 2 — `diagnostics_node`

- Subscribes the IMU topic; caches latest message.
- 10 Hz timer prints the fixed text block:

```
Orientation

Roll:
Pitch:
Yaw:

Quaternion

x
y
z
w

Gyroscope

X
Y
Z

Accelerometer

X
Y
Z
```

- Uses the same zeroed orientation as `orientation_node` for consistency:
  shared quaternion math lives in one helper module (see Shared code).

### Node 3 — `marker_node`

- Subscribes the IMU topic; 20 Hz `visualization_msgs/MarkerArray` in
  `base_link`:
  - Forward (red, +X), Up (blue, +Z), Right (green, +Y) body-axis arrows.
  - Gravity vector arrow (from measured linear acceleration, in body frame).
  - Angular-velocity arrow (from `imu.angular_velocity`).
  - Linear-acceleration arrow (from `imu.linear_acceleration`).
- Each marker a distinct namespace/id so RViz updates in place.

### Shared code — `imu/imu_math.py`

Single module for quaternion helpers used by all nodes (no duplication):
`normalize`, `quat_conjugate`, `quat_multiply`, `quat_inverse`,
`euler_from_quat`, `vec_to_marker_points`. Prefer `tf_transformations` where it
exists; wrap the rest.

### Launch — `imu/launch/imu_viz.launch.py`

One command brings everything up:
1. `ExecuteProcess` pre-kill: `pkill -f zed_wrapper` (best-effort; ignore
   non-zero exit when nothing is running), then a short settle.
2. `IncludeLaunchDescription` of `zed_wrapper/launch/zed_camera.launch.py` with
   `camera_model:=zed2i`, `camera_name:=zed2i`, and optional `serial_number`.
3. `orientation_node`, `diagnostics_node` (output=screen), `marker_node`.
4. RViz with `imu/rviz/imu.rviz`.

Launch args: `serial_number` (default empty), `camera_name` (default `zed2i`),
`rviz` (default true), `start_zed` (default true — set false to attach to an
already-running ZED and skip pre-kill).

### RViz config — `imu/rviz/imu.rviz`

Displays: Grid, TF (all frames, names on), Axes on `base_link`, `Imu` display
(orientation box + accel arrow) on the IMU topic, MarkerArray on the marker
topic. Fixed frame `odom`.

## Data flow

```
ZED 2i HW ─► zed_wrapper ─► /zed2i/zed_node/imu/data ─┬─► orientation_node ─► TF(odom→base_link) + /imu/rpy
                                                       ├─► diagnostics_node ─► stdout (10 Hz)
                                                       └─► marker_node      ─► /imu/markers (RViz)
```

## Error handling

- No IMU messages arriving → `orientation_node` logs a throttled warning every
  5 s ("no IMU data on <topic>"); publishes nothing until data flows (hardware
  only, per decision).
- Pre-kill step is best-effort; a missing/already-dead ZED node is not an error.
- Quaternion normalization guards against zero-norm (fallback to identity).
- QoS mismatch avoided by using `SensorDataQoS` on all IMU subscriptions.

## Testing

- Unit: `imu_math.py` pure functions (normalize, conjugate, multiply, inverse,
  round-trip euler↔quat, zeroing identity when q_ref==q_current) via pytest —
  matches the repo's existing pytest suite; no hardware needed.
- Integration (manual, on-vehicle): launch, confirm TF tree in RViz, tilt the
  sub and confirm `base_link` rotates, diagnostics block updates at 10 Hz, all
  six markers render and track motion, `/imu/reset_orientation` re-zeros.

## Future compatibility

All output is standard ROS messages (`sensor_msgs/Imu`, TF, markers), so later
fusion needs no changes here:
- **robot_localization EKF:** already-standard `sensor_msgs/Imu` input; the
  `odom → base_link` transform we publish is exactly what an EKF would later own
  (swap our broadcaster for the EKF's when fusion is added).
- **BAR02 depth / DVL / magnetometer / VSLAM:** add as EKF inputs; our
  `map → odom → base_link` chain is the conventional REP-105 layout they expect.

## Package updates

- New `src/imu` package: `package.xml`, `setup.py`, `resource/imu`,
  entry points `orientation_node`, `diagnostics_node`, `marker_node`.
- Depends: `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`,
  `tf2_ros`, `std_srvs`, `tf_transformations`, and the ZED wrapper at launch.
- Build with `colcon build --symlink-install --packages-select imu`.

## Deliverables

Source (`orientation_node.py`, `diagnostics_node.py`, `marker_node.py`,
`imu_math.py`), `imu_viz.launch.py`, `imu.rviz`, `package.xml`/`setup.py`,
pytest for `imu_math`, and run instructions in the package README.
