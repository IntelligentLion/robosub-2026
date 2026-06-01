# Jetson Orin Nano — Setup & Verification

Fresh-session runbook for the **Jetson Orin Nano 8 GB / 2× ZED 2i / Pixhawk
ArduSub** flight computer. JetPack 6 / Ubuntu 22.04 / ROS 2 Humble. The
⚠️ items need a human + the hardware.

Branch: `cleanup/organize-vslam-shrub-v4`

---

## 0. TL;DR

```bash
cd ~/robosub/robosub-2026
git fetch origin && git checkout cleanup/organize-vslam-shrub-v4

# BehaviorTree.ROS2 is not in apt — clone it once into src/
[ -d src/BehaviorTree.ROS2 ] || git clone \
  https://github.com/BehaviorTree/BehaviorTree.ROS2.git src/BehaviorTree.ROS2

sudo ./setup_vision_env.sh                # udev, cudnn symlink, MAXN
source /opt/ros/humble/setup.bash

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select auv_msgs btcpp_ros2_interfaces behaviortree_ros2 \
                    bt_mission vision mavlink_thruster_control \
                    localization control
source install/setup.bash

# put model weights in place (gitignored — see §3), then:
./src/run_stack.sh --onnx src/vision/vision/yolov8n.onnx 0.4 320 cuda 1.5
```

---

## 1. System prerequisites (one-time per flash)

Expected base: **JetPack 6 / Ubuntu 22.04**.

- **ROS 2 Humble** — `source /opt/ros/humble/setup.bash`
- **ZED SDK ≥ 4.0** + **`pyzed`** (bundled with the SDK installer).
- **TensorRT** (ships with JetPack) — **primary detector backend** (FP16 engine).
- **onnxruntime** — fallback only when no TRT engine can be built.
- **pymavlink** — `pip install pymavlink` (Pixhawk/thruster comms).
- **ultralytics** — only if running a `.pt` model instead of ONNX.
- **BehaviorTree.CPP v4** — `apt install ros-humble-behaviortree-cpp` (already
  installed in the base image).
- **BehaviorTree.ROS2** — clone into `src/` (no apt pkg):
  `git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git src/BehaviorTree.ROS2`

Sanity check before building:
```bash
python3 -c "import pyzed.sl as sl; print('pyzed OK')"
python3 -c "import tensorrt, onnxruntime; print('trt+ort OK')"
ros2 doctor --report | head -20
```

---

## 2. Environment setup script

`setup_vision_env.sh` (must be root) handles the Jetson-specific gotchas:
- ZED udev rules (without these: "CAMERA STREAM FAILED TO START")
- `libcudnn.so.8 → .so.9` compat symlinks (onnxruntime built against cuDNN 8)
- `nvpmodel -m 0` (MAXN) + `jetson_clocks` for max perf / avoid ENOMEM

```bash
sudo ./setup_vision_env.sh
```

---

## 3. Model weights (gitignored — must be supplied out-of-band)

`*.onnx`, `*.engine`, `*.pt` are **not** in git. Drop into the locations the
code expects.

**Backend: TensorRT FP16 (primary).** The `.onnx` is the source the engine is
built from — at runtime we execute the compiled `.engine`, not onnxruntime.

- `detector.py` default model: `src/vision/vision/yolov8n.onnx`
  (override with `--onnx /path`; `--weights model.pt` uses Ultralytics).

**One-shot deploy from a `.pt`** — `deploy_model.sh` runs the full
`pt → onnx → engine` pipeline and updates `current.{pt,onnx,engine}`
symlinks under `src/vision/vision/`:

```bash
./deploy_model.sh ~/Downloads/yolov8n.pt              # default imgsz=640, fp16
./deploy_model.sh ~/Downloads/robosub-best.pt 320     # custom imgsz
./deploy_model.sh --run ~/Downloads/best.pt 320       # deploy + launch run_stack.sh
```

Manual path (what `deploy_model.sh` invokes under the hood):
```bash
python3 convert_to_onnx.py model.pt model.onnx --imgsz 640
python3 build_engine.py model.onnx --fp16        # ~2–5 min, ~OOM risk if other GPU work
```

---

## 4. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select auv_msgs btcpp_ros2_interfaces behaviortree_ros2 \
                    bt_mission vision mavlink_thruster_control \
                    localization control
source install/setup.bash
```

Notes:
- VSLAM is **disabled for competition** — see
  `src/localization/launch/vslam_localization_launch.py` for the rationale.
  Only `localization_node` is launched there.

---

## 5. Run the stack

`run_stack.sh` launches thruster → localization → autonomous_controller →
behavior tree (`bt_mission/bt_executor`) → vision detector → depth node.
Auto-falls back to thruster **simulation mode** if no Pixhawk serial device.

```bash
# Standard path — TensorRT FP16 engine built from this .onnx:
./src/run_stack.sh --onnx src/vision/vision/yolov8n.onnx 0.4 320 cuda 1.5
# PyTorch model (Ultralytics backend, not TRT):
./src/run_stack.sh src/vision/vision/model.pt 0.4 320 cuda 1.5
#                  <model> <conf> <imgsz> <device> <stop_dist_m>
```

To run just the v4 executor manually (without the rest of the stack):
```bash
ros2 run bt_mission bt_executor --ros-args -p tree_id:=MainTree
```

---

## 6. Verify it's actually working

```bash
source install/setup.bash
python3 test_pipeline.py          # prints detections + depth as they arrive
# or manually:
ros2 topic list
ros2 topic echo /vision/detections
ros2 topic echo /depth/info
ros2 topic echo /movement_command
ros2 topic echo /navigation_command
```

Pixhawk / thruster bring-up (standalone, before full stack):
```bash
python3 test_pixhawk.py           # arm + per-thruster checks
# dry-run with no hardware: run_stack falls back to simulate:=true automatically
```

`detector.py` runs **headless by default**. To see the annotated camera
window on a desk (NOT on the sub — it wastes CPU/USB), add `--view`:
```bash
ros2 run vision detector -- --onnx <path> --view
```

---

## 7. Hardware checklist

- [ ] Pixhawk on `/dev/ttyACM*` or `/dev/ttyUSB*` (thruster_node auto-detects).
- [ ] ZED 2i (forward) on USB 3.0; `fuser /dev/video*` shows nothing else holding it.
- [ ] (Optional) bottom ZED for `bottom_camera_node` — VIO + path markers.
- [ ] Depth/pressure sensor publishing (drives `depth/info`).
- [ ] Battery ≥ 80%, leak sensors tested.
- [ ] `/usr/local/zed/tools/ZED_Diagnostic` clean.

Note: **no hydrophones this season.** Pinger-based nav has been removed from
the mission tree — torpedo and octagon tasks use vision-only search/approach.

---

## 8. Known gaps (don't get surprised)

See `src/robosub2026/MIGRATION.md` for the full list. Highlights:

1. **Battery + leak publishers do not exist yet.** `battery_pct` and
   `leak_detected` keep their seeded defaults on the BT blackboard.
2. **No gripper / marker-dropper / torpedo actuator drivers.** The BT
   actions (`ReleaseMarker`, `LaunchTorpedo`, `ReleaseObject`, `ActivateTool`)
   log + update counters but don't fire real hardware.
3. **Roll/Pitch primitives missing.** `MovementCommand` has no roll/pitch
   axis, so style-points roll/pitch maneuvers are no-ops.

---

## 9. Troubleshooting quick hits

| Symptom | Check |
|---|---|
| "CAMERA STREAM FAILED TO START" | re-run `sudo ./setup_vision_env.sh`; replug ZED to USB 3.0 |
| onnxruntime cuDNN load error | the `libcudnn.so.8` symlinks from the setup script |
| TensorRT build OOM | pre-build offline with `build_engine.py`; `nvpmodel -m 0` |
| Thruster won't arm | `python3 test_pixhawk.py`; safety switch + ArduSub pre-arm |
| `bt_mission` won't build | `behaviortree_ros2` not in apt — clone it into `src/` per §1 |
| BT XML won't load | `xmllint --noout src/robosub2026/bt_xml/robosub2026_mission.xml` |
| Nodes stuck RUNNING | `ros2 topic list` / `ros2 topic echo`; confirm the Python stack is up |
