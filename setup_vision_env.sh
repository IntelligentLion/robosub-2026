#!/bin/bash
# Vision environment setup for JetPack 6 / ZED 2i / Jetson Orin
set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "Re-running with sudo..."
    exec sudo bash "$0" "$@"
fi

echo "=== Vision Environment Setup ==="
echo ""

# 1 — ZED udev rules (missing from SDK install, causes CAMERA STREAM FAILED TO START)
echo "[1/4] Installing ZED camera udev rules..."
cat > /etc/udev/rules.d/99-zed.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2b03", MODE="0660", GROUP="zed"
EOF
echo "      Written /etc/udev/rules.d/99-zed.rules"

# 2 — Reload rules and retrigger the ZED device (no replug needed)
echo "[2/4] Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger --attr-match=idVendor=2b03
sleep 1

# 3 — Verify ZED device permissions
echo "[3/4] Verifying ZED device permissions..."
ZED_DEV=$(find /dev/bus/usb -name "*" -type c 2>/dev/null | xargs -I{} sh -c \
    'udevadm info {} 2>/dev/null | grep -q "2b03" && echo {}' 2>/dev/null | head -1)
if [ -n "$ZED_DEV" ]; then
    ls -la "$ZED_DEV"
else
    echo "      ZED device not found — replug the camera and re-run this script."
fi

# 4 — libcudnn.so.8 compatibility symlink (onnxruntime 1.19 was built against cuDNN 8;
#     system has cuDNN 9 — symlink lets the library load; versioned-symbol fallback
#     handled by TensorRT inference path in detector.py)
echo "[4/4] Setting up libcudnn.so.8 symlink..."
ln -sf /usr/lib/aarch64-linux-gnu/libcudnn.so.9 /usr/lib/aarch64-linux-gnu/libcudnn.so.8
ln -sf /lib/aarch64-linux-gnu/libcudnn.so.9     /lib/aarch64-linux-gnu/libcudnn.so.8
echo "      /usr/lib/aarch64-linux-gnu/libcudnn.so.8 -> libcudnn.so.9"
echo "      /lib/aarch64-linux-gnu/libcudnn.so.8     -> libcudnn.so.9"

# 5 — Jetson max-performance mode (prevents GPU busy / ENOMEM on CUDA init)
echo "[5] Setting Jetson to max performance mode..."
nvpmodel -m 0 2>/dev/null && echo "      nvpmodel: mode 0 (MAXN)" || echo "      nvpmodel not available, skipping"
jetson_clocks  2>/dev/null && echo "      jetson_clocks: enabled"  || echo "      jetson_clocks not available, skipping"

echo ""
echo "=== Done ==="
echo "Run: ros2 run vision detector"
echo ""
echo "Note: First run will build a TensorRT engine from the ONNX model (~2-5 min)."
echo "      The engine is cached as zed_right_24_06_15.engine alongside the .onnx file."
