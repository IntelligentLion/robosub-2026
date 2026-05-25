# ZED2i VSLAM — ROS2 Package

A full **Visual SLAM** package for the **Stereolabs ZED2i** camera.  
Publishes pose, odometry, IMU, magnetometer, point cloud (pseudo-LiDAR),  
path, RGB image, depth image, camera info, and all required TF frames.

---

## Published Topics

| Topic | Message Type | Description |
|---|---|---|
| `/zed2i/pose` | `geometry_msgs/PoseStamped` | Camera pose in the world (map) frame |
| `/zed2i/odom` | `nav_msgs/Odometry` | Visual odometry with linear velocity twist |
| `/zed2i/imu/data` | `sensor_msgs/Imu` | Accelerometer + gyroscope + fused orientation (200 Hz) |
| `/zed2i/imu/mag` | `sensor_msgs/MagneticField` | Calibrated magnetometer (Tesla) |
| `/zed2i/path` | `nav_msgs/Path` | Accumulated trajectory history |
| `/zed2i/point_cloud` | `sensor_msgs/PointCloud2` | Depth-fused XYZRGB point cloud (pseudo-LiDAR) |
| `/zed2i/image/left` | `sensor_msgs/Image` | Left RGB frame (`bgr8`) |
| `/zed2i/image/depth` | `sensor_msgs/Image` | 32-bit depth map in metres (`32FC1`) |
| `/zed2i/camera_info` | `sensor_msgs/CameraInfo` | Left camera intrinsics + distortion |

## TF Tree

```
map
 └── odom              (static identity; replace with EKF in production)
      └── base_link    (robot base, driven by VSLAM pose)
           └── camera_link  (static: 15 cm fwd, 10 cm up from base)
```

---

## Prerequisites

### System
- Ubuntu 22.04 + ROS2 Humble (or Iron / Jazzy)
- ZED SDK ≥ 4.0 — https://www.stereolabs.com/developers/release/
- `pyzed` Python API (bundled with ZED SDK installer)
- CUDA ≥ 11.8 (required by ZED SDK)

### ROS2 dependencies
```bash
sudo apt install ros-$ROS_DISTRO-tf2-ros \
                 ros-$ROS_DISTRO-tf2-geometry-msgs \
                 ros-$ROS_DISTRO-nav-msgs \
                 ros-$ROS_DISTRO-sensor-msgs \
                 ros-$ROS_DISTRO-geometry-msgs \
                 ros-$ROS_DISTRO-rviz2 \
                 ros-$ROS_DISTRO-rviz-imu-plugin  # optional
```

---

## Build & Install

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone <this-repo> zed2i_vslam

# Build
cd ~/ros2_ws
colcon build --packages-select zed2i_vslam --symlink-install

# Source
source install/setup.bash
```

---

## Usage

### Basic launch
```bash
ros2 launch zed2i_vslam vslam.launch.py
```

### With RViz2 visualisation
```bash
ros2 launch zed2i_vslam vslam.launch.py rviz:=true
```

### High-resolution with neural depth
```bash
ros2 launch zed2i_vslam vslam.launch.py \
    resolution:=HD1080 fps:=30 depth_mode:=NEURAL rviz:=true
```

### Run node directly
```bash
ros2 run zed2i_vslam vslam_node \
    --ros-args \
    -p resolution:=HD720 \
    -p fps:=30 \
    -p depth_mode:=ULTRA \
    -p publish_point_cloud:=true
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `resolution` | `HD720` | `HD2K` \| `HD1080` \| `HD720` \| `VGA` |
| `fps` | `30` | Frame rate (resolution-dependent max) |
| `depth_mode` | `ULTRA` | `ULTRA` \| `QUALITY` \| `PERFORMANCE` \| `NEURAL` |
| `enable_spatial_map` | `true` | Enable ZED 3D spatial mapping |
| `publish_point_cloud` | `true` | Enable PointCloud2 publisher |
| `point_cloud_downsample` | `4` | Keep every Nth pixel (reduce bandwidth) |
| `imu_rate_hz` | `200` | ZED2i IMU native rate |
| `base_frame` | `base_link` | Robot base TF frame |
| `camera_frame` | `camera_link` | Camera TF frame |
| `odom_frame` | `odom` | Odometry TF frame |
| `map_frame` | `map` | World/map TF frame |

---

## Simulation Mode

If the ZED SDK / `pyzed` is **not installed**, the node automatically falls back to
**simulation mode**: it generates a synthetic figure-8 (lemniscate) trajectory with
plausible IMU values and a random point cloud.  This lets you develop and test your
ROS2 pipeline without hardware.

---

## Integration with Nav2

To use this package as the localization source for Nav2, remap topics in your Nav2
`bringup` launch:

```python
remappings=[
    ('/odom', '/zed2i/odom'),
]
```

And set `robot_base_frame: base_link` in `amcl.yaml` / `ekf.yaml`.

### EKF fusion (recommended)

For robust state estimation, feed both odometry and IMU into
`robot_localization`'s EKF:

```yaml
# ekf.yaml
odom0: /zed2i/odom
odom0_config: [true, true, false, false, false, true, true, true, false, false, false, true, false, false, false]

imu0: /zed2i/imu/data
imu0_config: [false, false, false, true, true, true, false, false, false, true, true, true, false, false, false]
imu0_remove_gravitational_acceleration: true
```

---

## Coordinate System

The ZED SDK is configured with `RIGHT_HANDED_Z_UP_X_FWD` to match ROS convention:

| Axis | Direction |
|---|---|
| X | Forward |
| Y | Left |
| Z | Up |

---

## License
MIT
