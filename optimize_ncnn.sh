#!/usr/bin/env bash
# optimize_ncnn.sh - Exports YOLOv8n to NCNN format for 10x-15x FPS on Raspberry Pi 4B

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ -d ".venv" ]; then
    source .venv/bin/activate
fi

echo "=========================================================="
echo "    Exporting YOLOv8n to NCNN for 15+ FPS on Pi 4B       "
echo "=========================================================="
echo "This takes about 60 seconds and only needs to be run once."
echo "----------------------------------------------------------"

yolo export model=yolov8n.pt format=ncnn imgsz=192

echo "----------------------------------------------------------"
echo "SUCCESS! Model exported to 'yolov8n_ncnn_model/'."
echo "Now simply run: ./run.sh"
echo "=========================================================="
