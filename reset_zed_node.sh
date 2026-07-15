#!/usr/bin/env bash
# ==================================================================
#  reset_zed_node.sh — restart a ZED-owning ROS2 node in place.
#
#  Node-level reset ONLY: kills the running ZED node process and
#  relaunches it. Does NOT reboot the Jetson and does NOT replug the
#  USB. Use it when the ZED node has wedged (frozen frames, "CAMERA
#  STREAM FAILED", stale pose) but the rest of the stack is healthy.
#
#  Two nodes open the front ZED directly (single-owner — never both):
#    * vision_node    (vision/detector)      -> detections + vslam/odometry
#    * vslam_zed_node (localization/vslam_node) -> standalone VSLAM
#
#  How it works:
#    1. Find the target node's PID.
#    2. Capture its exact argv from /proc (preserves --onnx, --svo, ...).
#    3. SIGINT (clean ZED close), escalate to SIGKILL if it hangs.
#    4. Wait for the ZED USB to settle so the re-open succeeds.
#    5. Relaunch the SAME command, detached, logging to /var/tmp.
#
#  If the node is not currently running, relaunches from a default
#  command for the requested node instead.
#
#  Usage:
#    ./reset_zed_node.sh                 # auto: reset whichever ZED node is up
#    ./reset_zed_node.sh vision_node     # target the detector node
#    ./reset_zed_node.sh vslam_zed_node  # target the standalone VSLAM node
#    ./reset_zed_node.sh --settle 6      # override USB settle wait (default 4s)
#    ./reset_zed_node.sh --no-relaunch vision_node   # kill only, don't restart
# ==================================================================
set -u

REPO="/home/robosub/robosub2026/robosub-2026"
SETTLE=4            # seconds to wait for the ZED USB to release after kill
SIGINT_WAIT=8       # seconds to allow a clean SIGINT shutdown before SIGKILL
RELAUNCH=1
TARGET=""

# ---- ZED device (Stereolabs vendor id) for the settle check ----------
ZED_VENDOR="2b03"

# ---- node table: name -> cmdline match pattern -> default relaunch ----
pattern_for() {
  case "$1" in
    vision_node)    echo "lib/vision/detector" ;;
    vslam_zed_node) echo "lib/localization/vslam_node" ;;
    *) echo "" ;;
  esac
}
default_cmd_for() {
  case "$1" in
    vision_node)    echo "ros2 run vision detector" ;;
    vslam_zed_node) echo "ros2 run localization vslam_node" ;;
    *) echo "" ;;
  esac
}

# ---- arg parse -------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --settle)      SETTLE="$2"; shift 2 ;;
    --no-relaunch) RELAUNCH=0; shift ;;
    vision_node|vslam_zed_node) TARGET="$1"; shift ;;
    -h|--help) sed -n '2,34p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# ---- ROS environment (needed for pgrep names + relaunch) -------------
# ROS/colcon setup scripts reference unset vars, so disable -u while sourcing.
set +u
# shellcheck disable=SC1090
source /opt/ros/*/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "$REPO/install/setup.bash" 2>/dev/null || true
set -u

# ---- pick target: explicit, or auto-detect the running ZED node ------
find_pid() {  # $1 = node name -> echoes first matching PID (empty if none)
  local pat; pat="$(pattern_for "$1")"
  [ -z "$pat" ] && return 0
  pgrep -f "$pat" | head -n1
}

if [ -z "$TARGET" ]; then
  for n in vision_node vslam_zed_node; do
    if [ -n "$(find_pid "$n")" ]; then TARGET="$n"; break; fi
  done
  if [ -z "$TARGET" ]; then
    echo "No ZED node running. Specify one: vision_node | vslam_zed_node" >&2
    exit 1
  fi
  echo "Auto-detected running ZED node: $TARGET"
fi

PID="$(find_pid "$TARGET")"

# ---- capture argv so relaunch is byte-identical ----------------------
CAPTURED_ARGV=""
if [ -n "$PID" ] && [ -r "/proc/$PID/cmdline" ]; then
  # /proc cmdline is NUL-delimited; turn into a quoted, re-runnable string.
  mapfile -d '' -t _argv < "/proc/$PID/cmdline"
  for a in "${_argv[@]}"; do
    [ -n "$a" ] && CAPTURED_ARGV+=" $(printf '%q' "$a")"
  done
  echo "Target $TARGET  PID=$PID"
  echo "  cmd:$CAPTURED_ARGV"
else
  echo "Target $TARGET not currently running."
fi

# ---- kill: SIGINT (clean ZED close), then SIGKILL --------------------
if [ -n "$PID" ]; then
  echo "Sending SIGINT to $PID (clean ZED shutdown)..."
  kill -INT "$PID" 2>/dev/null
  for _ in $(seq 1 "$SIGINT_WAIT"); do
    kill -0 "$PID" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$PID" 2>/dev/null; then
    echo "Still alive after ${SIGINT_WAIT}s — SIGKILL."
    kill -KILL "$PID" 2>/dev/null
    sleep 1
  fi
  # Sweep any stragglers sharing the same match pattern (respawned children).
  pat="$(pattern_for "$TARGET")"
  pkill -KILL -f "$pat" 2>/dev/null
  echo "Node killed."
else
  echo "Nothing to kill."
fi

# ---- wait for the ZED USB to settle ----------------------------------
echo "Waiting ${SETTLE}s for ZED USB to release..."
sleep "$SETTLE"
if command -v lsusb >/dev/null 2>&1; then
  if lsusb -d "${ZED_VENDOR}:" >/dev/null 2>&1; then
    echo "ZED USB present (vendor ${ZED_VENDOR})."
  else
    echo "WARNING: ZED USB (vendor ${ZED_VENDOR}) not visible on the bus." >&2
    echo "         Node re-open will retry, but a hardware replug may be needed." >&2
  fi
fi

# ---- relaunch --------------------------------------------------------
if [ "$RELAUNCH" -eq 0 ]; then
  echo "--no-relaunch set. Done (killed only)."
  exit 0
fi

LOG="/var/tmp/${TARGET}.log"
if [ -n "$CAPTURED_ARGV" ]; then
  RUN="$CAPTURED_ARGV"
else
  RUN="$(default_cmd_for "$TARGET")"
  echo "No captured argv — using default: $RUN"
fi

echo "Relaunching $TARGET, logging to $LOG ..."
# setsid detaches from this shell so the node survives the script exiting.
setsid bash -c "source /opt/ros/*/setup.bash 2>/dev/null; \
                source '$REPO/install/setup.bash' 2>/dev/null; \
                exec$RUN" >>"$LOG" 2>&1 &
NEWPID=$!
sleep 2
if kill -0 "$NEWPID" 2>/dev/null; then
  echo "Relaunched. new PID (launcher)=$NEWPID  log=$LOG"
  echo "Tail the log:  tail -f $LOG"
else
  echo "WARNING: relaunched process exited within 2s — check $LOG" >&2
  tail -n 20 "$LOG" >&2
  exit 1
fi
