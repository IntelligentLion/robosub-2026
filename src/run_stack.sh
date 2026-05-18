#!/usr/bin/env bash
# ==================================================================
#  run_stack.sh – launch the full AUV mission stack
#
#  Usage (PyTorch):
#    ./run_stack.sh /path/to/model.pt [conf_thres] [img_size] [device] [stop_dist]
#
#  Usage (ONNX):
#    ./run_stack.sh --onnx /path/to/model.onnx [conf_thres] [img_size] [device] [stop_dist]
#
#  stop_dist: metres at which the sub stops approaching a detected object (default 1.5)
#
#  If no Pixhawk serial device is found the thruster controller starts
#  in simulation mode so the rest of the stack can still run.
# ==================================================================
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

USE_ONNX=false
if [[ "${1:-}" == "--onnx" ]]; then
  USE_ONNX=true
  shift
fi

MODEL_PATH="${1:-}"
CONF_THRES="${2:-0.4}"
IMG_SIZE="${3:-320}"
DEVICE="${4:-cpu}"
STOP_DIST="${5:-1.5}"

if [[ -z "${MODEL_PATH}" ]]; then
  if [[ "${USE_ONNX}" == true ]]; then
    echo "Usage: $0 --onnx /absolute/path/to/model.onnx [conf_thres] [img_size] [device] [stop_dist_m]"
  else
    echo "Usage: $0 /absolute/path/to/model.pt [conf_thres] [img_size] [device] [stop_dist_m]"
  fi
  exit 1
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace not built yet. Run:"
  echo "  cd ${WORKSPACE_DIR} && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select auv_msgs vision mission mavlink_thruster_control localization"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

PIDS=()

cleanup() {
  echo ""
  echo "Stopping all nodes..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ─── Detect serial port for thruster controller ─────────────────────
SIMULATE_FLAG=""
if ! ls /dev/ttyACM* /dev/ttyUSB* &>/dev/null; then
  echo "[WARN] No serial device found – thruster controller will run in SIMULATION mode"
  SIMULATE_FLAG="--ros-args -p simulate:=true"
fi

# ─── 1. Thruster controller ─────────────────────────────────────────
echo "Starting thruster controller..."
# shellcheck disable=SC2086
ros2 run mavlink_thruster_control thruster_node ${SIMULATE_FLAG} &
PIDS+=($!)
sleep 2

# Verify the thruster controller is alive
if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
  echo "[WARN] Thruster controller failed to start – continuing in degraded mode"
fi

# ─── 2. Behavior tree ───────────────────────────────────────────────
echo "Starting behavior tree..."
ros2 run mission bt_runner &
PIDS+=($!)
sleep 1

# ─── 3. Vision detector ─────────────────────────────────────────────
echo "Starting vision detector..."
if [[ "${USE_ONNX}" == true ]]; then
  ros2 run vision detector -- \
    --onnx "${MODEL_PATH}" \
    --conf_thres "${CONF_THRES}" \
    --img_size "${IMG_SIZE}" \
    --device "${DEVICE}" &
else
  ros2 run vision detector -- \
    --weights "${MODEL_PATH}" \
    --conf_thres "${CONF_THRES}" \
    --img_size "${IMG_SIZE}" \
    --device "${DEVICE}" &
fi
PIDS+=($!)
sleep 1

# ─── 4. Depth sensing node ──────────────────────────────────────────
echo "Starting depth node (stop distance: ${STOP_DIST} m)..."
ros2 run localization depth_node -- --stop_distance "${STOP_DIST}" &
PIDS+=($!)

echo "Stack is running. Press Ctrl+C to stop all."
wait
