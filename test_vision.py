#!/usr/bin/env python3
"""test_vision.py — exercise the vision detector in isolation.

What it does
------------
1. Optionally launches `ros2 run vision detector` with the given .onnx (or
   uses the `current.onnx` symlink dropped by `deploy_model.sh`).
2. Subscribes to /vision/detections for --duration seconds and aggregates:
     * frame count + measured FPS
     * detections per frame (avg / max)
     * per-label stats: count, max confidence, last cx/cy/range
3. Compares the labels seen against the expected vocabulary the BT uses
   (the list in src/robosub2026/MIGRATION.md → "Vision label vocabulary").
4. Prints a one-screen summary and exits with a non-zero code if NO frames
   were received (useful in CI / smoke tests).

Usage
-----
  ./test_vision.py                                       # use current.onnx, 15s, headless
  ./test_vision.py --model src/vision/vision/yolov8n.onnx
  ./test_vision.py --duration 30 --view                  # show annotated window
  ./test_vision.py --no-launch                           # subscribe only; assume detector already up
  ./test_vision.py --imgsz 640 --conf 0.25 --device cuda
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    from auv_msgs.msg import ObjectDetectionArray
except ImportError:
    print("[error] auv_msgs not on PYTHONPATH — `source install/setup.bash` first", file=sys.stderr)
    sys.exit(2)


# Labels the BT's perception nodes look for (mirror of MIGRATION.md).
# Frames are not required to contain ALL of these — this is just for the
# end-of-run "seen vs expected" diff.
EXPECTED_LABELS = {
    "gate", "role_sign", "survey_repair", "search_rescue",
    "orange_path", "slalom_pole", "slalom_gap", "path_marker",
    "pipeline", "fire_bin", "blood_bin", "bin1", "bin2",
    "marker", "magnetic_target",
    "target_board", "large_opening", "small_opening",
    "octagon", "basket", "repair_object", "medical_object",
}


class VisionMonitor(Node):
    def __init__(self):
        super().__init__("vision_monitor")
        self.frames = 0
        self.t_first = None
        self.t_last = None
        self.det_counts = []
        # per-label: {label: {"count": int, "max_conf": float, "last": Detection-ish}}
        self.labels = defaultdict(lambda: {
            "count": 0, "max_conf": 0.0,
            "cx": 0.0, "cy": 0.0, "range": 0.0,
        })
        self.create_subscription(
            ObjectDetectionArray, "vision/detections", self._cb, 10)

    def _cb(self, msg: ObjectDetectionArray):
        now = time.monotonic()
        if self.t_first is None:
            self.t_first = now
        self.t_last = now
        self.frames += 1
        self.det_counts.append(len(msg.detections))
        for d in msg.detections:
            s = self.labels[d.label]
            s["count"] += 1
            if d.confidence > s["max_conf"]:
                s["max_conf"] = d.confidence
                s["cx"] = d.position.x
                s["cy"] = d.position.y
                s["range"] = d.position.z


def launch_detector(args) -> subprocess.Popen:
    """Spawn `ros2 run vision detector` with --onnx <model>. Returns Popen."""
    cmd = [
        "ros2", "run", "vision", "detector", "--",
        "--onnx", args.model,
        "--conf_thres", str(args.conf),
        "--img_size", str(args.imgsz),
        "--device", args.device,
    ]
    if args.view:
        cmd.append("--view")
    print(f"[launch] {' '.join(cmd)}", flush=True)
    # Use a fresh process group so we can SIGTERM the whole thing cleanly.
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        bufsize=1, text=True,
    )


def stream_detector_log(proc: subprocess.Popen, sink_path: Path):
    """Mirror detector output to a file so we can grep it in the summary."""
    with open(sink_path, "w") as f:
        for line in proc.stdout:  # type: ignore[arg-type]
            f.write(line)
            f.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    repo = Path(__file__).resolve().parent
    default_model = repo / "src" / "vision" / "vision" / "current.onnx"
    ap.add_argument("--model", default=str(default_model),
                    help="path to .onnx (default: src/vision/vision/current.onnx)")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--duration", type=float, default=15.0,
                    help="seconds to subscribe before summarizing (default 15)")
    ap.add_argument("--view", action="store_true",
                    help="show detector's annotated camera window")
    ap.add_argument("--no-launch", action="store_true",
                    help="do not spawn the detector — assume it's already running")
    args = ap.parse_args()

    # Validate model
    if not args.no_launch:
        if not Path(args.model).exists():
            print(f"[error] model not found: {args.model}", file=sys.stderr)
            print("        run ./deploy_model.sh /path/to/model.pt first, "
                  "or pass --no-launch", file=sys.stderr)
            sys.exit(2)

    print("─────────────────────────────────────────────")
    print("test_vision.py")
    print(f"  model    : {args.model}")
    print(f"  imgsz    : {args.imgsz}")
    print(f"  conf     : {args.conf}")
    print(f"  device   : {args.device}")
    print(f"  duration : {args.duration}s")
    print(f"  view     : {args.view}")
    print(f"  launch   : {not args.no_launch}")
    print("─────────────────────────────────────────────")

    # Spawn detector
    proc = None
    log_path = Path("/tmp/test_vision_detector.log")
    log_thread = None
    if not args.no_launch:
        proc = launch_detector(args)
        # Drain its stdout in a background thread so the pipe doesn't fill.
        import threading
        log_thread = threading.Thread(
            target=stream_detector_log, args=(proc, log_path), daemon=True)
        log_thread.start()
        # Give the detector a moment to come up (TRT engine load can take a few s
        # the first run; cached engine is fast).
        print("[wait] giving detector ~4s to come up...", flush=True)
        time.sleep(4.0)
        if proc.poll() is not None:
            print(f"[error] detector died on startup. Last log:", file=sys.stderr)
            try:
                print(log_path.read_text()[-2000:], file=sys.stderr)
            except Exception:
                pass
            sys.exit(2)

    # Subscribe
    rclpy.init()
    monitor = VisionMonitor()
    print(f"[subscribe] /vision/detections for {args.duration}s...", flush=True)
    t_end = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(monitor, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Teardown detector
    if proc is not None and proc.poll() is None:
        print("[teardown] stopping detector...", flush=True)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    # Summary
    print()
    print("─────────────── summary ───────────────")
    if monitor.frames == 0:
        print("  NO FRAMES RECEIVED on /vision/detections")
        print()
        if not args.no_launch:
            print("  detector log tail:")
            try:
                print("    " + "\n    ".join(log_path.read_text().splitlines()[-30:]))
            except Exception:
                pass
        rclpy.shutdown()
        sys.exit(1)

    elapsed = (monitor.t_last - monitor.t_first) or 1e-9
    fps = (monitor.frames - 1) / elapsed if monitor.frames > 1 else 0.0
    avg_dets = sum(monitor.det_counts) / len(monitor.det_counts)
    max_dets = max(monitor.det_counts)

    print(f"  frames received : {monitor.frames}")
    print(f"  measured FPS    : {fps:.2f}")
    print(f"  detections/frame: avg={avg_dets:.2f}, max={max_dets}")
    print()

    seen_labels = set(monitor.labels.keys())
    if seen_labels:
        print(f"  labels seen ({len(seen_labels)}):")
        rows = sorted(monitor.labels.items(),
                      key=lambda kv: kv[1]["count"], reverse=True)
        print(f"    {'label':<22} {'count':>6} {'max_conf':>9} {'cx':>6} {'cy':>6} {'rng':>6}")
        for label, s in rows:
            print(f"    {label:<22} {s['count']:>6} {s['max_conf']:>9.2f} "
                  f"{s['cx']:>6.2f} {s['cy']:>6.2f} {s['range']:>6.2f}")
    else:
        print("  labels seen: NONE — model loaded but emitted zero detections")

    print()
    expected_seen = seen_labels & EXPECTED_LABELS
    expected_missing = EXPECTED_LABELS - seen_labels
    unexpected = seen_labels - EXPECTED_LABELS
    print(f"  BT-expected labels covered: {len(expected_seen)} / {len(EXPECTED_LABELS)}")
    if expected_missing:
        print(f"    missing  : {', '.join(sorted(expected_missing))}")
    if unexpected:
        print(f"    unexpected (model emits but BT doesn't use): "
              f"{', '.join(sorted(unexpected))}")

    print("───────────────────────────────────────")
    if not args.no_launch:
        print(f"  full detector log: {log_path}")

    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
