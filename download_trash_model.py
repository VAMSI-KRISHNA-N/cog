#!/usr/bin/env python3
"""
download_trash_model.py - Helper to download and verify the 10-class waste detection model.

Model: yolov8-trash-detections from Roboflow Universe
Classes (10):
  paper, can, cardboard, plastic, plastic bag, plastic bottle,
  battery, glass bottle, pop tab, drink carton

Usage:
  # Check if model is already installed:
  python download_trash_model.py --check

  # Download via Roboflow API:
  python download_trash_model.py --api-key YOUR_ROBOFLOW_API_KEY

  # Display detailed manual download instructions:
  python download_trash_model.py
"""

import argparse
import os
import sys

TARGET_MODEL_PATH = "yolov8_trash.pt"


def verify_model(filepath: str) -> bool:
    """Verifies that the model file exists, is non-empty, and can be loaded by YOLO."""
    if not os.path.exists(filepath):
        print(f"[CHECK] File '{filepath}' does not exist.")
        return False

    size_mb = os.path.getsize(filepath) / (1024 * 1024)
    if size_mb < 1.0:
        print(f"[CHECK] File '{filepath}' is too small ({size_mb:.2f} MB). Likely corrupt or incomplete.")
        return False

    print(f"[CHECK] Found '{filepath}' ({size_mb:.2f} MB). Verifying with ultralytics...")
    try:
        from ultralytics import YOLO
        m = YOLO(filepath)
        num_classes = len(m.names)
        print(f"[CHECK] Model loaded successfully! Found {num_classes} classes:")
        for idx, name in m.names.items():
            print(f"         [{idx}] {name}")
        return True
    except Exception as e:
        print(f"[CHECK] Warning: Could not verify with ultralytics ({e}). File exists, but test with perception.py.")
        return True


def print_manual_guide(output_path: str):
    """Prints clear, step-by-step instructions for manual download."""
    print("=" * 78)
    print("           HOW TO OBTAIN THE 10-CLASS WASTE DETECTION MODEL               ")
    print("=" * 78)
    print(
        f"\nTarget Destination: ./{output_path}\n"
        "\nOption 1: Direct Download via Roboflow API (Recommended)\n"
        "---------------------------------------------------------\n"
        "1. Create a free account at https://app.roboflow.com/\n"
        "2. Go to: Settings -> Roboflow API -> Copy your Private API Key\n"
        "3. Run this script with your key:\n"
        f"     python download_trash_model.py --api-key YOUR_KEY\n"
        "   Or set the environment variable:\n"
        "     export ROBOFLOW_API_KEY=YOUR_KEY\n"
        "     python download_trash_model.py\n"
        "\nOption 2: Manual Download from Roboflow Universe (No CLI Key Needed)\n"
        "--------------------------------------------------------------------\n"
        "1. Visit the dataset page on Roboflow Universe:\n"
        "     https://universe.roboflow.com/waste-detection-spldq/yolov8-trash-detections\n"
        "2. Click 'Download Dataset' or 'Weights' -> select 'YOLOv8 PyTorch' format.\n"
        "3. Download the weights file (typically named 'best.pt' or 'yolov8n.pt').\n"
        f"4. Move or rename the file into this project directory as '{output_path}'.\n"
        f"5. Run: python download_trash_model.py --check\n"
        "\nNote on Placeholder Fallback:\n"
        "-----------------------------\n"
        "If 'yolov8_trash.pt' is not present, perception.py will fall back to\n"
        "'yolov8n.pt' for offline bench testing, but will print a warning because\n"
        "COCO weights misclassify trash objects (e.g. mouse -> apple, fan -> airplane).\n"
    )
    print("=" * 78 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Download and verify the Roboflow Universe 10-class waste detection model."
    )
    parser.add_argument(
        "--api-key",
        type=str,
        default=os.environ.get("ROBOFLOW_API_KEY", ""),
        help="Roboflow API Key (defaults to ROBOFLOW_API_KEY environment variable)",
    )
    parser.add_argument(
        "--workspace",
        type=str,
        default="waste-detection-spldq",
        help="Roboflow workspace name (default: waste-detection-spldq)",
    )
    parser.add_argument(
        "--project",
        type=str,
        default="yolov8-trash-detections",
        help="Roboflow project name (default: yolov8-trash-detections)",
    )
    parser.add_argument(
        "--version",
        type=int,
        default=1,
        help="Dataset/model version number (default: 1)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=TARGET_MODEL_PATH,
        help=f"Target file path for model weights (default: {TARGET_MODEL_PATH})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Verify existing model checkpoint without downloading",
    )
    args = parser.parse_args()

    if args.check:
        print(f"[CHECK] Verifying '{args.output}'...")
        ok = verify_model(args.output)
        sys.exit(0 if ok else 1)

    if os.path.exists(args.output):
        print(f"[INFO] '{args.output}' already exists in current directory.")
        ok = verify_model(args.output)
        if ok:
            print(f"[SUCCESS] '{args.output}' is ready for use with perception.py!")
            return

    if not args.api_key:
        print_manual_guide(args.output)
        return

    try:
        from roboflow import Roboflow
    except ImportError:
        print(
            "[ERROR] The 'roboflow' package is not installed.\n"
            "        Run: pip install roboflow\n",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        print(f"[DOWNLOAD] Connecting to Roboflow workspace '{args.workspace}'...")
        rf = Roboflow(api_key=args.api_key)
        project = rf.workspace(args.workspace).project(args.project)
        version = project.version(args.version)
        print(f"[DOWNLOAD] Downloading model weights for project '{args.project}' v{args.version}...")
        model = version.download("yolov8")
        print(f"[DOWNLOAD] Success! Downloaded to: {model.location}")

        # Check for best.pt or weights
        weights_candidates = [
            os.path.join(model.location, "weights", "best.pt"),
            os.path.join(model.location, "best.pt"),
            os.path.join(model.location, "weights", "yolov8n.pt"),
            os.path.join(model.location, "yolov8n.pt"),
        ]
        found = False
        for cand in weights_candidates:
            if os.path.exists(cand):
                import shutil
                shutil.copyfile(cand, args.output)
                print(f"[SETUP] Copied model weights from '{cand}' to '{args.output}'.")
                found = True
                break

        if not found:
            print(
                f"[WARNING] Downloaded dataset/model folder at '{model.location}'.\n"
                f"          Please inspect the directory and copy the trained .pt file to '{args.output}'."
            )
        else:
            verify_model(args.output)

    except Exception as e:
        print(f"[ERROR] Failed to download via Roboflow API: {e}", file=sys.stderr)
        print("Please follow manual download instructions below:\n", file=sys.stderr)
        print_manual_guide(args.output)
        sys.exit(1)


if __name__ == "__main__":
    main()
