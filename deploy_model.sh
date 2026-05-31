#!/usr/bin/env bash
# ==================================================================
#  deploy_model.sh – one-shot model deployment for the vision stack.
#
#  Takes an Ultralytics YOLO .pt file and:
#    1. exports it to ONNX (via convert_to_onnx.py)
#    2. builds a TensorRT FP16 .engine (via build_engine.py)
#    3. installs all three artifacts into src/vision/vision/
#    4. (re)points the `current.{pt,onnx,engine}` symlinks at them so
#       run_stack.sh / the detector can pick up the new model without
#       editing any paths.
#
#  Usage:
#    ./deploy_model.sh /path/to/yolov8n.pt
#    ./deploy_model.sh /path/to/model.pt 320          # custom imgsz
#    ./deploy_model.sh /path/to/model.pt 320 fp32     # disable FP16 (TRT)
#    ./deploy_model.sh --run /path/to/model.pt        # deploy AND launch run_stack.sh
# ==================================================================
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VISION_DIR="${REPO_DIR}/src/vision/vision"

# ─── Parse args ────────────────────────────────────────────────────
RUN_STACK_AFTER=false
if [[ "${1:-}" == "--run" ]]; then
  RUN_STACK_AFTER=true
  shift
fi

PT_INPUT="${1:-}"
IMGSZ="${2:-640}"
PRECISION="${3:-fp16}"  # fp16 | fp32

if [[ -z "${PT_INPUT}" ]]; then
  cat <<EOF
Usage: $0 [--run] <path-to-model.pt> [imgsz] [fp16|fp32]

  --run     after deploy, launch ./src/run_stack.sh with the new .onnx
  imgsz     square input size for ONNX export (default: 640)
  fp16|fp32 TensorRT precision (default: fp16)

Examples:
  $0 ~/Downloads/yolov8n.pt
  $0 ~/Downloads/robosub-best.pt 320 fp16
  $0 --run ~/Downloads/robosub-best.pt 320
EOF
  exit 1
fi

if [[ ! -f "${PT_INPUT}" ]]; then
  echo "[error] file not found: ${PT_INPUT}" >&2
  exit 1
fi
if [[ "${PT_INPUT}" != *.pt ]]; then
  echo "[error] expected a .pt file, got: ${PT_INPUT}" >&2
  exit 1
fi
if [[ ! -d "${VISION_DIR}" ]]; then
  echo "[error] vision package dir missing: ${VISION_DIR}" >&2
  exit 1
fi

# ─── Resolve destination filenames ─────────────────────────────────
BASE="$(basename "${PT_INPUT}" .pt)"
DEST_PT="${VISION_DIR}/${BASE}.pt"
DEST_ONNX="${VISION_DIR}/${BASE}.onnx"
DEST_ENGINE="${VISION_DIR}/${BASE}.engine"

echo "─────────────────────────────────────────────"
echo "deploy_model.sh"
echo "  source .pt   : ${PT_INPUT}"
echo "  imgsz        : ${IMGSZ}"
echo "  precision    : ${PRECISION}"
echo "  install into : ${VISION_DIR}/${BASE}.{pt,onnx,engine}"
echo "─────────────────────────────────────────────"

# ─── 0. Stage the .pt next to its siblings ─────────────────────────
if [[ "$(readlink -f "${PT_INPUT}")" != "$(readlink -f "${DEST_PT}" 2>/dev/null || true)" ]]; then
  echo "[1/4] Copying .pt → ${DEST_PT}"
  cp -f "${PT_INPUT}" "${DEST_PT}"
else
  echo "[1/4] .pt already at destination — skipping copy"
fi

# ─── 1. Convert .pt → .onnx ────────────────────────────────────────
echo "[2/4] Exporting ONNX (imgsz=${IMGSZ})"
python3 "${REPO_DIR}/convert_to_onnx.py" "${DEST_PT}" "${DEST_ONNX}" --imgsz "${IMGSZ}"

if [[ ! -f "${DEST_ONNX}" ]]; then
  echo "[error] ONNX export did not produce ${DEST_ONNX}" >&2
  exit 1
fi

# ─── 2. Build TensorRT engine ──────────────────────────────────────
echo "[3/4] Building TensorRT engine (${PRECISION})"
TRT_FLAGS=()
case "${PRECISION}" in
  fp16) TRT_FLAGS+=(--fp16) ;;
  fp32) TRT_FLAGS+=(--fp32) ;;
  *)    echo "[error] precision must be fp16 or fp32" >&2; exit 1 ;;
esac
python3 "${REPO_DIR}/build_engine.py" "${DEST_ONNX}" "${TRT_FLAGS[@]}"

if [[ ! -f "${DEST_ENGINE}" ]]; then
  echo "[warn] no .engine produced — detector will fall back to onnxruntime"
fi

# ─── 3. Update `current.*` symlinks ────────────────────────────────
echo "[4/4] Updating current.{pt,onnx,engine} symlinks"
ln -sfn "${BASE}.pt"     "${VISION_DIR}/current.pt"
ln -sfn "${BASE}.onnx"   "${VISION_DIR}/current.onnx"
if [[ -f "${DEST_ENGINE}" ]]; then
  ln -sfn "${BASE}.engine" "${VISION_DIR}/current.engine"
fi

echo ""
echo "─────────────────────────────────────────────"
echo "Done. Artifacts:"
ls -la "${VISION_DIR}/${BASE}.pt" "${VISION_DIR}/${BASE}.onnx" \
       "${VISION_DIR}/${BASE}.engine" 2>/dev/null || true
echo ""
echo "current.* symlinks:"
ls -la "${VISION_DIR}/current.pt" "${VISION_DIR}/current.onnx" \
       "${VISION_DIR}/current.engine" 2>/dev/null || true
echo "─────────────────────────────────────────────"
echo ""
echo "Launch the stack with:"
echo "  ./src/run_stack.sh --onnx ${VISION_DIR}/current.onnx 0.4 ${IMGSZ} cuda 1.5"
echo ""

# ─── 4. Optional auto-launch ───────────────────────────────────────
if [[ "${RUN_STACK_AFTER}" == "true" ]]; then
  echo "[--run] handing off to run_stack.sh..."
  exec "${REPO_DIR}/src/run_stack.sh" --onnx "${VISION_DIR}/current.onnx" 0.4 "${IMGSZ}" cuda 1.5
fi
