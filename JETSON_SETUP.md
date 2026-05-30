# Jetson Orin Nano — Setup & Verification Handoff

This file is the to-do list for a **fresh session on the Jetson** (Orin Nano
8 GB, 2× ZED 2i). It covers everything that could not be built or verified in
the dev environment (which has no ROS 2, no ZED SDK, no CUDA). Work top to
bottom; the ⚠️ items are the ones that actually need a human + the hardware.

Branch with the recent cleanup/migration work:
`cleanup/organize-vslam-shrub-v4`

---

## 0. TL;DR for the impatient

```bash
cd ~/robosub/robosub-2026
git fetch origin && git checkout cleanup/organize-vslam-shrub-v4

sudo ./setup_vision_env.sh                      # udev, cudnn symlink, max perf
source /opt/ros/humble/setup.bash

# ⚠️ verify the NEW C++ first — it was never compiled off-board
colcon build --symlink-install --packages-select auv_msgs bt_mission
# ...fix any errors (see §4 + src/robosub2026/MIGRATION.md), then build the rest:
colcon build --symlink-install --packages-select \
  auv_msgs vision mission bt_mission mavlink_thruster_control localization control
source install/setup.bash

# put model weights in place (gitignored — see §3), then:
./src/run_stack.sh --onnx src/vision/vision/yolov8n.onnx 0.4 320 cuda 1.5
```

---

## 1. System prerequisites (one-time per flash)

Expected base: **JetPack 6 / Ubuntu 22.04**.

- **ROS 2 Humble** — `source /opt/ros/humble/setup.bash`
- **ZED SDK ≥ 4.0** + the **`pyzed`** Python API (bundled with the SDK installer).
  Requires CUDA (ships with JetPack).
- **TensorRT** (ships with JetPack) — **primary detector backend** (FP16 engine).
  We run TensorRT, not onnxruntime, for the **lower inference latency** on the
  Orin Nano.
- **onnxruntime** — fallback only (used if no TRT engine can be built, e.g. low
  free VRAM). Keep it installed, but it is not the intended runtime path.
- **pymavlink** — `pip install pymavlink` — Pixhawk/thruster comms.
- **ultralytics** (only if running a `.pt` model instead of ONNX).
- For the C++ behavior tree (`bt_mission`):
  - **BehaviorTree.CPP v4** (`behaviortree_cpp`)
  - **BehaviorTree.ROS2** (`behaviortree_ros2`)
  - (legacy `mission` pkg additionally needs **`behaviortree_cpp_v3`**)

  Install BT.CPP v4 + ROS2 wrapper per the top-level `README.md` → Installation.

Sanity check before building:
```bash
python3 -c "import pyzed.sl as sl; print('pyzed OK')"
python3 -c "import tensorrt, onnxruntime; print('trt+ort OK')"
ros2 doctor --report | head -20
```

---

## 2. Run the environment setup script

`setup_vision_env.sh` (must be root) handles the Jetson-specific gotchas:
- ZED udev rules (without these: "CAMERA STREAM FAILED TO START")
- `libcudnn.so.8 → .so.9` compat symlinks (onnxruntime was built vs cuDNN 8)
- `nvpmodel -m 0` (MAXN) + `jetson_clocks` for max performance / to avoid CUDA
  init ENOMEM

```bash
sudo ./setup_vision_env.sh
```

> The repo also has a `libcudnn.so.9` symlink at the root and `src/`. Those are
> environment-specific; the script recreates the `.so.8` compat links it needs.

---

## 3. Model weights (gitignored — must be supplied out-of-band)

`*.onnx`, `*.engine`, `*.pt` are **not** in git. Get them from team storage and
place them where the code expects:

**Backend: TensorRT FP16 (primary, for latency).** The `.onnx` is just the *source*
the engine is built from — at runtime we execute the compiled `.engine`, not
onnxruntime. `get_shared_model()` in `detector.py` prefers TensorRT whenever an
engine exists (or enough VRAM is free to build one) and only falls back to
onnxruntime otherwise.

- `detector.py` default model: `src/vision/vision/yolov8n.onnx`
  (override with `--onnx /path`; `--weights model.pt` uses Ultralytics instead).
- **Pre-build the engine offline** (strongly recommended — building during a run
  competes with the ZED for VRAM and can OOM). It's cached next to the `.onnx` as
  `*.engine`:
  ```bash
  python3 build_engine.py src/vision/vision/yolov8n.onnx --fp16
  ```
  If you skip this, the engine is built on first run (~2–5 min) instead.
- `convert_to_onnx.py` regenerates an ONNX from a `.pt` if you need a new source.

---

## 4. ⚠️ Verify the v4 C++ migration (the part that was never compiled)

This branch added a ROS I/O layer to `bt_mission` (`src/robosub2026/`) and wired
the executor + `EmergencySurface`. **None of it was compiled off-board.** Build it
in isolation first:

```bash
colcon build --symlink-install --packages-select auv_msgs bt_mission
```

If it fails, the likely spots and the design intent are documented in
**`src/robosub2026/MIGRATION.md`**. Files involved:
- `include/bt_mission/mission_io.hpp`, `src/mission_io.cpp` — the `shrub::MissionIO`
  singleton (publishes `movement_command`/`navigation_command`; caches
  `vision/detections`, `depth/info`, `localization/pose`).
- `src/bt_executor.cpp` — calls `MissionIO::init(ros_node)`, injects live depth
  onto the blackboard.
- `src/nav_nodes.cpp` — `EmergencySurface` now commands `emerge`.
- `CMakeLists.txt` / `package.xml` — added `auv_msgs` dependency.

Message field names were matched against `src/auv_msgs/msg/*.msg`; if a build
error mentions a missing field, check there first.

---

## 5. Full workspace build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select auv_msgs vision mission bt_mission \
                    mavlink_thruster_control localization control
source install/setup.bash
```

Notes:
- The duplicate `src/zed2i_vslam(2)/` package is intentionally skipped (it has a
  `COLCON_IGNORE`; its parens-in-name breaks colcon anyway).
- VSLAM is **disabled for competition** — see
  `src/localization/launch/vslam_localization_launch.py` for the rationale. Only
  `localization_node` launches there.

---

## 6. Run the stack

`run_stack.sh` launches thruster → localization → autonomous_controller →
behavior tree → vision detector → depth node. It auto-falls back to thruster
**simulation mode** if no Pixhawk serial device is present.

```bash
# Standard path — TensorRT FP16 engine built from this .onnx (lower latency):
./src/run_stack.sh --onnx src/vision/vision/yolov8n.onnx 0.4 320 cuda 1.5
# PyTorch model (Ultralytics backend, not TRT):
./src/run_stack.sh src/vision/vision/model.pt 0.4 320 cuda 1.5
#                  <model> <conf> <imgsz> <device> <stop_dist_m>
```

> The `--onnx` flag does **not** mean "run onnxruntime" — it points at the model
> the **TensorRT** engine is built/loaded from. Pre-build with `build_engine.py`
> (§3) so the first run doesn't pay the ~2–5 min build cost or risk OOM.

> The behavior tree launched is still the **legacy** `mission/bt_runner` (the
> working one). SHRUB v4 (`bt_mission/bt_executor`) is canonical going forward but
> its nodes are mid-port — do **not** switch `run_stack.sh` to `bt_executor` until
> the port is pool-verified (see `src/robosub2026/MIGRATION.md` → "Definition of
> done").

To run the v4 executor manually for bring-up/testing:
```bash
ros2 run bt_mission bt_executor --ros-args -p tree_id:=SHRUB
```

---

## 7. Verify it's actually working

Topic-level integration check (run while the stack is up):
```bash
source install/setup.bash
python3 test_pipeline.py          # prints detections + depth as they arrive
# or manually:
ros2 topic list
ros2 topic echo /vision/detections
ros2 topic echo /depth/info
ros2 topic echo /movement_command
```

Pixhawk / thruster bring-up (standalone, before full stack):
```bash
python3 test_pixhawk.py           # arm + per-thruster checks
# dry-run with no hardware: run_stack falls back to simulate:=true automatically
```

`detector.py` runs **headless by default** now. To see the annotated camera
window on a desk (NOT on the sub — it wastes CPU/USB), add `--view`:
```bash
ros2 run vision detector -- --onnx <path> --view
```

---

## 8. Hardware checklist (from README, condensed)

- [ ] Pixhawk on `/dev/ttyACM*` or `/dev/ttyUSB*` (thruster_node auto-detects;
      else simulation mode). Safety switch / pre-arm checks pass.
- [ ] ZED 2i (forward) on USB 3.0; `fuser /dev/video*` shows nothing else holding it.
- [ ] (If used) bottom ZED for `bottom_camera_node` — it also does VIO + path markers.
- [ ] Depth/pressure sensor publishing (drives `depth/info`).
- [ ] Battery ≥ 80%, leak sensors tested.
- [ ] `/usr/local/zed/tools/ZED_Diagnostic` clean.

---

## 9. Known gaps / next work (don't get surprised)

From `src/robosub2026/MIGRATION.md`:
1. **Most v4 BT nodes are still stubs.** Only safety-depth + EmergencySurface are
   wired. Perception/alignment/manipulation nodes still return SUCCESS instantly.
   Port them per the node map in MIGRATION.md before trusting `bt_executor`.
2. **No `battery_pct` / `leak_detected` publisher** exists — the SafetyMonitor's
   battery & leak checks are currently no-ops (read seeded blackboard defaults).
3. **No gripper / marker-dropper / torpedo actuator drivers** — `Grab_object`,
   `Drop_marker`, `Fire_torpedo` can't actuate yet.
4. **`IsTimeRemaining` clock mismatch** (`safety_nodes.cpp`): compares
   `steady_clock` against a ROS-time `start_time`. Fix during the port.

---

## 10. Troubleshooting quick hits

| Symptom | Check |
|---|---|
| "CAMERA STREAM FAILED TO START" | re-run `sudo ./setup_vision_env.sh` (udev rules), replug ZED to USB 3.0 |
| onnxruntime cuDNN load error | the `libcudnn.so.8` symlinks from the setup script |
| TensorRT build OOM | pre-build offline with `build_engine.py`; ensure `nvpmodel -m 0` |
| Thruster won't arm | `python3 test_pixhawk.py`; check safety switch + ArduSub pre-arm |
| `bt_mission` won't build | §4 + `src/robosub2026/MIGRATION.md`; verify `auv_msgs` built first |
| BT XML won't load | `xmllint --noout src/robosub2026/bt_xml/robosub2026_mission.xml` |
| Nodes stuck RUNNING | `ros2 topic list` / `ros2 action list`; confirm the Python stack is up |
