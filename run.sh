#!/usr/bin/env bash
# run.sh - Automated runner for Phase 1 Waste-Detection Pointing System

set -e

echo "=========================================================="
echo "   Waste Robot — Phase 1 Continuous Tracker Launcher     "
echo "=========================================================="

# 1. Ensure we are in the script's root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 2. Activate virtual environment if it exists
if [ -d ".venv" ]; then
    echo "[INIT] Activating virtual environment (.venv)..."
    source .venv/bin/activate
else
    echo "[WARNING] No .venv directory found; running with system Python."
fi

# 3. Detect connected Arduino port
ARDUINO_PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1)

# 4. Resolve Model Checkpoint: prioritize real waste detector
MODEL="yolov8_trash.pt"
if [ -d "yolov8_trash_ncnn_model" ]; then
    echo "[SPEED] Fast NCNN waste model detected! Using yolov8_trash_ncnn_model."
    MODEL="yolov8_trash_ncnn_model"
elif [ -f "yolov8_trash.pt" ]; then
    echo "[MODEL] Roboflow 10-class waste model detected: yolov8_trash.pt."
    MODEL="yolov8_trash.pt"
elif [ -d "yolov8n_ncnn_model" ]; then
    echo "[MODEL] Using NCNN model: yolov8n_ncnn_model."
    MODEL="yolov8n_ncnn_model"
elif [ -f "yolov8n.pt" ]; then
    echo "[MODEL] Using fallback checkpoint: yolov8n.pt."
    MODEL="yolov8n.pt"
fi

# 5. Launch in appropriate mode
if [ -n "$ARDUINO_PORT" ]; then
    echo "[HARDWARE] Arduino detected at: $ARDUINO_PORT"
    echo "[LAUNCH] Starting in REAL HARDWARE CONTINUOUS TRACKING MODE..."
    echo "----------------------------------------------------------"
    python3 pi/perception.py --port "$ARDUINO_PORT" --baud 115200 --model "$MODEL" --imgsz 320 --show "$@"
else
    echo "[HARDWARE] No Arduino detected on /dev/ttyACM* or /dev/ttyUSB*."
    echo "[LAUNCH] Starting in MOCK SERIAL CONTINUOUS TRACKING MODE..."
    echo "----------------------------------------------------------"
    python3 pi/perception.py --mock --model "$MODEL" --imgsz 320 --show "$@"
fi
