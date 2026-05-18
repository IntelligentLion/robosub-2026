#!/usr/bin/env bash
# ==================================================================
#  test_movement.sh – interactively test individual movement commands
#
#  Usage:
#    ./test_movement.sh
#
#  Requires: thruster_node running (real or simulation mode)
#  Start it first:
#    ros2 run mavlink_thruster_control thruster_node --ros-args -p simulate:=true
# ==================================================================

set -uo pipefail

# Source ROS if not already
if ! command -v ros2 &>/dev/null; then
  source /opt/ros/humble/setup.bash 2>/dev/null || {
    echo "[ERROR] Cannot source /opt/ros/humble/setup.bash"
    exit 1
  }
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
    source "${SCRIPT_DIR}/install/setup.bash" 2>/dev/null
  fi
fi

# Verify ros2 is actually available after sourcing
if ! command -v ros2 &>/dev/null; then
  echo "[ERROR] ros2 command not found. Is ROS 2 installed?"
  exit 1
fi

TOPIC="movement_command"
MSG_TYPE="auv_msgs/msg/MovementCommand"

# ── Check that the message type is available ──────────────────────────
if ! ros2 interface show "${MSG_TYPE}" &>/dev/null; then
  echo "[ERROR] Message type '${MSG_TYPE}' not found."
  echo "  Build the workspace first:  colcon build --packages-select auv_msgs"
  exit 1
fi

send_cmd() {
  local cmd="$1"
  local speed="${2:-0.5}"
  local duration="${3:-3.0}"

  # Validate speed is a number in range
  if ! [[ "${speed}" =~ ^[0-9]*\.?[0-9]+$ ]] || \
     (( $(echo "${speed} > 1.0" | bc -l 2>/dev/null || echo 0) )); then
    echo "  [WARN] Speed '${speed}' out of range 0.0–1.0, clamping."
    speed=$(echo "${speed}" | awk '{if($1<0)$1=0; if($1>1)$1=1; print $1}')
  fi

  # Validate duration is a non-negative number
  if ! [[ "${duration}" =~ ^[0-9]*\.?[0-9]+$ ]]; then
    echo "  [WARN] Invalid duration '${duration}', defaulting to 3.0"
    duration="3.0"
  fi

  echo "  → Sending: command='${cmd}' speed=${speed} duration=${duration}"

  # Use timeout to avoid hanging if ros2 pub gets stuck
  timeout 10 ros2 topic pub --once "${TOPIC}" "${MSG_TYPE}" \
    "{command: '${cmd}', speed: ${speed}, duration: ${duration}}" 2>/dev/null
  local rc=$?
  if [[ $rc -ne 0 ]]; then
    echo "  [WARN] Publish may have failed (exit code ${rc})"
  fi
}

print_menu() {
  echo ""
  echo "╔══════════════════════════════════════════════╗"
  echo "║         AUV Movement Test Console            ║"
  echo "╠══════════════════════════════════════════════╣"
  echo "║  1) submerge        2) emerge                ║"
  echo "║  3) surge_forward   4) surge_backward        ║"
  echo "║  5) strafe_left     6) strafe_right          ║"
  echo "║  7) rotate_cw       8) rotate_ccw            ║"
  echo "║  9) stop            0) depth_hold            ║"
  echo "║  c) custom command  e) emergency stop        ║"
  echo "║  m) monitor topic   q) quit                  ║"
  echo "╚══════════════════════════════════════════════╝"
}

read_speed_duration() {
  local default_speed="${1:-0.5}"
  local default_dur="${2:-3.0}"
  read -rp "  Speed (0.0-1.0) [${default_speed}]: " speed
  speed="${speed:-$default_speed}"
  read -rp "  Duration (seconds) [${default_dur}]: " duration
  duration="${duration:-$default_dur}"
}

# ── Pre-flight checks ────────────────────────────────────────────────
echo "Checking thruster controller..."

sub_count=0
if ros2 topic info "${TOPIC}" &>/dev/null; then
  sub_count=$(ros2 topic info "${TOPIC}" 2>/dev/null \
    | grep -oP 'Subscription count: \K[0-9]+' || echo "0")
fi

if [[ "${sub_count}" -eq 0 ]]; then
  echo "[WARN] No subscriber on '${TOPIC}' — start thruster_node first:"
  echo "  ros2 run mavlink_thruster_control thruster_node --ros-args -p simulate:=true"
  echo ""
  read -rp "Continue anyway? [Y/n]: " cont
  [[ "${cont,,}" == "n" ]] && exit 0
else
  echo "  Found ${sub_count} subscriber(s) on '${TOPIC}'"
fi
echo "Ready."

# ── Trap Ctrl+C to send stop before exiting ──────────────────────────
trap 'echo ""; echo "Emergency stop..."; send_cmd "stop" 0.0 0.0; echo "Bye."; exit 0' INT

while true; do
  print_menu
  read -rp "Choice: " choice

  case "${choice}" in
    1)
      read_speed_duration 0.4 5.0
      send_cmd "submerge" "$speed" "$duration"
      ;;
    2)
      read_speed_duration 0.4 5.0
      send_cmd "emerge" "$speed" "$duration"
      ;;
    3)
      read_speed_duration 0.5 3.0
      send_cmd "surge_forward" "$speed" "$duration"
      ;;
    4)
      read_speed_duration 0.5 3.0
      send_cmd "surge_backward" "$speed" "$duration"
      ;;
    5)
      read_speed_duration 0.5 3.0
      send_cmd "strafe_left" "$speed" "$duration"
      ;;
    6)
      read_speed_duration 0.5 3.0
      send_cmd "strafe_right" "$speed" "$duration"
      ;;
    7)
      read_speed_duration 0.5 3.0
      send_cmd "rotate_cw" "$speed" "$duration"
      ;;
    8)
      read_speed_duration 0.5 3.0
      send_cmd "rotate_ccw" "$speed" "$duration"
      ;;
    9)
      send_cmd "stop" 0.0 0.0
      ;;
    0)
      send_cmd "depth_hold" 0.0 0.0
      ;;
    e|E)
      echo "  *** EMERGENCY STOP ***"
      send_cmd "stop" 0.0 0.0
      # send a second time for reliability
      send_cmd "stop" 0.0 0.0
      ;;
    c|C)
      read -rp "  Command name: " custom_cmd
      read_speed_duration 0.5 3.0
      send_cmd "$custom_cmd" "$speed" "$duration"
      ;;
    m|M)
      echo "  Monitoring ${TOPIC} (Ctrl+C to stop)..."
      ros2 topic echo "${TOPIC}" "${MSG_TYPE}"
      ;;
    q|Q)
      echo "Sending stop..."
      send_cmd "stop" 0.0 0.0
      echo "Bye."
      exit 0
      ;;
    *)
      echo "  Invalid choice."
      ;;
  esac
done
