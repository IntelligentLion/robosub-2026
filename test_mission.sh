#!/usr/bin/env bash
# ==================================================================
#  test_mission.sh — exercise the BT pipeline with configurable run params.
#
#  Brings up the full SHRUB stack on the desk (thruster in sim if no Pixhawk,
#  safety_monitor in sim with configurable nominal battery, localization,
#  depth, autonomous_controller, optionally vision detector, and bt_executor
#  with the parameters from the CLI) for a fixed duration, then tears it
#  down and prints a compact summary of what fired.
#
#  Usage:
#    ./test_mission.sh                                # all defaults
#    ./test_mission.sh --role search_rescue --style off
#    ./test_mission.sh --coin-flip backward --gate-red left --duration 30
#    ./test_mission.sh --battery 10                   # trip critical_failure
#    ./test_mission.sh --vision src/vision/vision/current.onnx
#    ./test_mission.sh --vision src/vision/vision/current.onnx --view
# ==================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ─── Defaults ──────────────────────────────────────────────────────
ROLE="survey_repair"      # survey_repair | search_rescue
COIN_FLIP="normal"        # normal | backward
STYLE="on"                # on | off
GATE_RED="right"          # right | left
RUN_MODE="semifinal"      # semifinal | final | qualification
BATTERY_PCT="100"         # nominal battery % published by safety_monitor in sim
TICK_RATE_MS="50"
DURATION="20"             # seconds of mission tick before teardown
VISION_MODEL=""           # path to .onnx (empty = no detector)
VIEW=false                # detector --view flag (desk only)

# ─── CLI parse ─────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --role)         ROLE="$2"; shift 2 ;;
    --coin-flip)    COIN_FLIP="$2"; shift 2 ;;
    --style)        STYLE="$2"; shift 2 ;;
    --gate-red)     GATE_RED="$2"; shift 2 ;;
    --run-mode)     RUN_MODE="$2"; shift 2 ;;
    --battery)      BATTERY_PCT="$2"; shift 2 ;;
    --tick)         TICK_RATE_MS="$2"; shift 2 ;;
    --duration)     DURATION="$2"; shift 2 ;;
    --vision)       VISION_MODEL="$2"; shift 2 ;;
    --view)         VIEW=true; shift ;;
    -h|--help)
      sed -n '3,17p' "$0"
      exit 0
      ;;
    *)
      echo "[error] unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

# ─── Validate ──────────────────────────────────────────────────────
case "$ROLE"      in survey_repair|search_rescue) ;; *) echo "bad --role"; exit 1;; esac
case "$COIN_FLIP" in normal|backward)             ;; *) echo "bad --coin-flip"; exit 1;; esac
case "$STYLE"     in on|off)                      ;; *) echo "bad --style"; exit 1;; esac
case "$GATE_RED"  in right|left)                  ;; *) echo "bad --gate-red"; exit 1;; esac
STYLE_BOOL=$([ "$STYLE" = "on" ] && echo true || echo false)

if [[ ! -f "${REPO_DIR}/install/setup.bash" ]]; then
  echo "[error] workspace not built — run colcon build first" >&2
  exit 1
fi

echo "─────────────────────────────────────────────"
echo "test_mission.sh"
echo "  role          : ${ROLE}"
echo "  coin_flip     : ${COIN_FLIP}"
echo "  style_enabled : ${STYLE_BOOL}"
echo "  gate_red_side : ${GATE_RED}"
echo "  run_mode      : ${RUN_MODE}"
echo "  battery_pct   : ${BATTERY_PCT}"
echo "  tick_rate_ms  : ${TICK_RATE_MS}"
echo "  duration      : ${DURATION}s"
echo "  vision model  : ${VISION_MODEL:-<none>}"
[[ "${VIEW}" = true ]] && echo "  vision --view : ON"
echo "─────────────────────────────────────────────"

# ─── Env ───────────────────────────────────────────────────────────
set +u
source /opt/ros/humble/setup.bash
source "${REPO_DIR}/install/setup.bash"
set -u

LOG_DIR="/tmp/test_mission"
rm -rf "${LOG_DIR}" && mkdir -p "${LOG_DIR}"
PIDS=()

cleanup() {
  echo ""
  echo "[teardown] stopping nodes..."
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ─── Bring-up ──────────────────────────────────────────────────────
# Thruster: simulate if no Pixhawk on /dev/ttyACM* or /dev/ttyUSB*.
SIM_FLAG=""
if ! ls /dev/ttyACM* /dev/ttyUSB* &>/dev/null; then
  SIM_FLAG="--ros-args -p simulate:=true"
fi
echo "[1/6] thruster_node"
ros2 run mavlink_thruster_control thruster_node ${SIM_FLAG} \
  >"${LOG_DIR}/thruster.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[2/6] safety_monitor_node (battery=${BATTERY_PCT})"
ros2 run mavlink_thruster_control safety_monitor_node \
  --ros-args -p nominal_battery_pct:="${BATTERY_PCT}" \
  >"${LOG_DIR}/safety.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[3/6] localization_node + depth_node"
ros2 run localization localization_node >"${LOG_DIR}/loc.log" 2>&1 &
PIDS+=($!)
ros2 run localization depth_node >"${LOG_DIR}/depth.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[4/6] autonomous_controller"
ros2 run control autonomous_controller >"${LOG_DIR}/ctl.log" 2>&1 &
PIDS+=($!)
sleep 1

if [[ -n "${VISION_MODEL}" ]]; then
  echo "[5/6] vision detector (${VISION_MODEL})"
  VIEW_FLAG=""
  [[ "${VIEW}" = true ]] && VIEW_FLAG="--view"
  ros2 run vision detector -- \
    --onnx "${VISION_MODEL}" --conf_thres 0.4 --img_size 320 --device cuda \
    ${VIEW_FLAG} >"${LOG_DIR}/vision.log" 2>&1 &
  PIDS+=($!)
  sleep 2
else
  echo "[5/6] vision detector skipped (no --vision)"
fi

echo "[6/6] bt_executor (role=${ROLE}, coin_flip=${COIN_FLIP}, style=${STYLE_BOOL}, gate_red_side=${GATE_RED})"
ros2 run bt_mission bt_executor --ros-args \
  -p role:="${ROLE}" \
  -p coin_flip:="${COIN_FLIP}" \
  -p style_enabled:="${STYLE_BOOL}" \
  -p gate_red_side:="${GATE_RED}" \
  -p run_mode:="${RUN_MODE}" \
  -p tick_rate_ms:="${TICK_RATE_MS}" \
  >"${LOG_DIR}/bt.log" 2>&1 &
PIDS+=($!)

echo ""
echo "Running for ${DURATION}s — logs at ${LOG_DIR}/"
sleep "${DURATION}"

# ─── Summary ───────────────────────────────────────────────────────
echo ""
echo "─────────────── summary ───────────────"
echo "Topics seen during run:"
ros2 topic list 2>/dev/null | grep -E "vision|movement|navigation|depth|safety|localization|odom" | sort | sed 's/^/  /'

echo ""
echo "BT events (last 40 lines, filtered):"
grep -E "SUCCESS|FAILURE|RUNNING|SAFETY|task. timer reset|Mission completed" \
     "${LOG_DIR}/bt.log" 2>/dev/null | tail -40 | sed 's/^/  /'

echo ""
echo "Safety publishers:"
echo "  /safety/battery_pct latest: $(timeout 3 ros2 topic echo /safety/battery_pct --once 2>/dev/null | grep data || echo '<none>')"
echo "  /safety/leak_detected latest: $(timeout 3 ros2 topic echo /safety/leak_detected --once 2>/dev/null | grep data || echo '<none>')"

echo ""
echo "Movement commands published by BT/controller (last 5):"
grep "Movement:" "${LOG_DIR}/thruster.log" 2>/dev/null | tail -5 | sed 's/^/  /'

echo ""
echo "Nav commands published by BT (last 5):"
grep "Nav command:" "${LOG_DIR}/ctl.log" 2>/dev/null | tail -5 | sed 's/^/  /'

echo "───────────────────────────────────────"
echo "Full logs: ls ${LOG_DIR}/"
