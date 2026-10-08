#!/usr/bin/env bash
# ==============================================================================
# setup_pi.sh - Automated 2-minute Raspberry Pi environment installer
#
# Sets up a lightweight virtual environment with CPU-only PyTorch,
# OpenCV, YOLOv8 (Ultralytics), and PySerial without any NVIDIA CUDA bloat.
# ==============================================================================

set -e

echo "=================================================="
echo "      Waste Robot — Raspberry Pi Setup Script     "
echo "=================================================="

# 1. Ensure we are in project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 2. Create clean virtual environment with system site-packages (for picamera2)
echo "[1/4] Creating virtual environment (.venv)..."
if [ -d ".venv" ]; then
    echo "      Existing .venv found. Re-creating clean environment..."
    rm -rf .venv
fi
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

# 3. Upgrade pip without caching
echo "[2/4] Updating pip..."
pip install --no-cache-dir --upgrade pip

# 4. Install CPU-only PyTorch (skips 4+ GB of NVIDIA CUDA bloat)
echo "[3/4] Installing lightweight CPU-only PyTorch (~160 MB)..."
pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

# 5. Install runtime dependencies for robot perception
echo "[4/4] Installing Ultralytics YOLO, OpenCV, and PySerial..."
pip install --no-cache-dir ultralytics opencv-python pyserial

echo "=================================================="
echo "[TEST] Verifying installation..."
python3 -c "import torch, cv2, ultralytics, serial; print('SUCCESS: PyTorch', torch.__version__, 'and YOLO ready!')"
echo "=================================================="
echo "Setup complete! To activate your environment in the future:"
echo "  source .venv/bin/activate"
echo "=================================================="
