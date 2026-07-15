# pix_imu — Pixhawk IMU orientation viz

RViz visualization of the sub's orientation driven by the **Pixhawk (ArduSub)**
IMU over MAVLink, instead of the ZED. Reuses the `imu` package's generic
`orientation_node` / `diagnostics_node` / `marker_node` — the only new piece is
`pixhawk_imu_bridge`, which converts MAVLink ATTITUDE + RAW_IMU into a single
`sensor_msgs/Imu` on `/pixhawk/imu/data`.

## Run

```bash
colcon build --symlink-install --packages-select pix_imu imu
source install/setup.bash
ros2 launch pix_imu pix_imu_viz.launch.py
```

Headless: append `rviz:=false`. Different serial: `port:=/dev/ttyACM1`.

## How it works

- **pixhawk_imu_bridge** connects `/dev/ttyACM0 @ 115200`, requests ATTITUDE
  (msg 30) and RAW_IMU (msg 27) at 50 Hz, and publishes `sensor_msgs/Imu`:
  - orientation ← ATTITUDE roll/pitch/yaw (rad)
  - angular_velocity ← ATTITUDE rollspeed/pitchspeed/yawspeed (rad/s)
  - linear_acceleration ← RAW_IMU xacc/yacc/zacc (mg → m/s²)
- Body frame is ArduPilot **FRD**. `orientation_node` zeroes orientation
  relative to a startup reference, so RViz reads level at launch and rotation
  displays correctly. Re-zero live with:
  ```bash
  ros2 service call /imu/reset_orientation std_srvs/srv/Trigger
  ```

## Notes

- Single serial reader thread — never point another script at the same
  `/dev/ttyACM0` concurrently (double-reader → "readiness to read but returned
  no data" stall).
- Ships its own copy of the pymavlink 2.4.49 `add_message` guard (same fix as
  `field_common.py`), since it connects independently of the mission code.
