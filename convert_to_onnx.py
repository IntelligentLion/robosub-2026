#!/usr/bin/env python3
"""Convert a Ultralytics YOLO .pt model to ONNX and validate with onnxruntime.

Usage:
    python3 convert_to_onnx.py /path/to/model.pt [output.onnx] [--imgsz 640] [--opset 17]

If output path is omitted, the ONNX file is written next to the .pt file.
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────

def ensure_package(name: str, pip_name: str | None = None):
    """Import *name*; install via pip if missing."""
    try:
        return __import__(name)
    except ModuleNotFoundError:
        pkg = pip_name or name
        print(f"[install] '{name}' not found – running: pip install {pkg}")
        subprocess.check_call([sys.executable, "-m", "pip", "install", pkg, "-q"])
        return __import__(name)


# ─────────────────────────────────────────────────────────────────────────────
#  Conversion
# ─────────────────────────────────────────────────────────────────────────────

def convert(pt_path: Path, onnx_path: Path, imgsz: int, opset: int) -> Path:
    """Export *pt_path* to ONNX using Ultralytics and return the output path."""
    from ultralytics import YOLO

    print(f"\n{'='*60}")
    print(f"  Input  : {pt_path}")
    print(f"  Output : {onnx_path}")
    print(f"  imgsz  : {imgsz}   opset: {opset}")
    print(f"{'='*60}\n")

    model = YOLO(str(pt_path))

    print(f"[model] task   : {model.task}")
    print(f"[model] classes: {model.names}\n")

    t0 = time.perf_counter()
    exported = model.export(
        format="onnx",
        imgsz=imgsz,
        opset=opset,
        dynamic=False,
        simplify=True,
    )
    elapsed = time.perf_counter() - t0

    # Ultralytics returns the export path as a string
    exported_path = Path(str(exported))
    if not exported_path.exists():
        raise FileNotFoundError(f"Export claimed to write {exported_path} but it is missing.")

    # Move to requested output location if different
    if exported_path.resolve() != onnx_path.resolve():
        onnx_path.parent.mkdir(parents=True, exist_ok=True)
        exported_path.rename(onnx_path)

    size_mb = onnx_path.stat().st_size / 1024 / 1024
    print(f"\n[export] Done in {elapsed:.1f}s  –  {onnx_path.name} ({size_mb:.1f} MB)")
    return onnx_path


# ─────────────────────────────────────────────────────────────────────────────
#  Validation
# ─────────────────────────────────────────────────────────────────────────────

def validate(onnx_path: Path, imgsz: int, class_names: dict):
    """Load the ONNX model with onnxruntime, run a dummy inference, print results."""
    import numpy as np
    import onnxruntime as ort

    print(f"\n{'='*60}")
    print("  Validation with onnxruntime")
    print(f"{'='*60}")

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    sess = ort.InferenceSession(str(onnx_path), providers=providers)
    active_provider = sess.get_providers()[0]
    print(f"[ort] provider : {active_provider}")

    # Print input / output metadata
    for inp in sess.get_inputs():
        print(f"[ort] input    : name={inp.name!r}  shape={inp.shape}  dtype={inp.type}")
    for out in sess.get_outputs():
        print(f"[ort] output   : name={out.name!r}  shape={out.shape}  dtype={out.type}")

    # Determine actual input dimensions from the model
    inp_meta = sess.get_inputs()[0]
    shape = inp_meta.shape          # e.g. [1, 3, 640, 640] or [1, 3, None, None]
    h = imgsz if not isinstance(shape[2], int) or shape[2] <= 0 else shape[2]
    w = imgsz if not isinstance(shape[3], int) or shape[3] <= 0 else shape[3]

    # Dummy random image (values in [0, 1], normalised like YOLO preprocessing)
    dummy = np.random.rand(1, 3, h, w).astype(np.float32)

    print(f"\n[test] Running inference on random {h}×{w} image ...")
    t0 = time.perf_counter()
    outputs = sess.run(None, {inp_meta.name: dummy})
    latency_ms = (time.perf_counter() - t0) * 1000
    print(f"[test] Latency : {latency_ms:.1f} ms")

    # Parse detections from YOLOv8 output: shape (1, 4+nc, na)
    raw = outputs[0]                # (1, 4+nc, na)
    print(f"[test] Raw output shape: {raw.shape}")

    if raw.ndim == 3 and raw.shape[1] > 4:
        data = raw[0].T             # (na, 4+nc)
        scores = data[:, 4:]
        class_ids = np.argmax(scores, axis=1)
        confidences = scores[np.arange(len(class_ids)), class_ids]

        conf_thres = 0.25
        mask = confidences >= conf_thres
        n_det = mask.sum()
        print(f"[test] Detections above {conf_thres} conf on dummy input: {n_det}")
        if n_det:
            for i in np.where(mask)[0][:5]:
                cid = class_ids[i]
                name = class_names.get(int(cid), str(cid))
                print(f"       • {name:<20} conf={confidences[i]:.3f}")
        else:
            print("       (expected for random noise – model loaded correctly)")
    else:
        print(f"[test] Output shape {raw.shape} – skipping detection parsing")

    print("\n[validate] PASSED – model runs correctly with onnxruntime")


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Convert YOLO .pt → .onnx and validate")
    parser.add_argument("pt_file",           help="Path to the input .pt model")
    parser.add_argument("onnx_file", nargs="?", default=None,
                        help="Output .onnx path (default: same dir as .pt, .onnx extension)")
    parser.add_argument("--imgsz",  type=int, default=640, help="Input image size (default: 640)")
    parser.add_argument("--opset",  type=int, default=17,  help="ONNX opset version (default: 17)")
    args = parser.parse_args()

    pt_path = Path(args.pt_file).expanduser().resolve()
    if not pt_path.exists():
        sys.exit(f"[error] File not found: {pt_path}")
    if pt_path.suffix.lower() != ".pt":
        sys.exit(f"[error] Expected a .pt file, got: {pt_path.suffix}")

    onnx_path = (
        Path(args.onnx_file).expanduser().resolve()
        if args.onnx_file
        else pt_path.with_suffix(".onnx")
    )

    # Ensure onnx is available (onnxruntime already confirmed present)
    ensure_package("onnx")

    # ── Convert ──────────────────────────────────────────────────────
    onnx_path = convert(pt_path, onnx_path, args.imgsz, args.opset)

    # ── Grab class names from the original model ──────────────────────
    from ultralytics import YOLO
    model = YOLO(str(pt_path))
    class_names = model.names          # {0: 'gate', 1: 'buoy', ...}

    # ── Validate ─────────────────────────────────────────────────────
    validate(onnx_path, args.imgsz, class_names)

    print(f"\n[done] ONNX model saved to: {onnx_path}")
    print("       Use with the detector:")
    print(f"       ./src/run_stack.sh --onnx {onnx_path}")


if __name__ == "__main__":
    main()
