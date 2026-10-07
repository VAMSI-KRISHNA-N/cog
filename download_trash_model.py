#!/usr/bin/env bash
"""
download_trash_model.py - Helper to download or configure the 10-class waste detection model.

Model: yolov8-trash-detections from Roboflow Universe
Classes (10):
  paper, can, cardboard, plastic, plastic bag, plastic bottle,
  battery, glass bottle, pop tab, drink carton
"""

import argparse
import os
import sys

TARGET_MODEL_PATH = "yolov8_trash.pt"


def main():
    parser = argparse.ArgumentParser(description="Download Roboflow Universe yolov8-trash-detections model")
    parser.add_argument("--api-key", type=str, default=os.environ.get("ROBOFLOW_API_KEY", ""), help="Roboflow API Key")
    parser.add_argument("--workspace", type=str, default="waste-detection-spldq", help="Roboflow workspace")
    parser.add_argument("--project", type=str, default="yolov8-trash-detections", help="Roboflow project name")
    parser.add_argument("--version", type=int, default=1, help="Dataset/model version number")
    parser.add_argument("--output", type=str, default=TARGET_MODEL_PATH, help="Target path for model weights")
    args = parser.parse_args()

    print("=" * 70)
    print("  Waste Detector Setup: Roboflow Universe yolov8-trash-detections")
    print("=" * 70)
    print(f"Target model destination: {args.output}")

    if not args.api_key:
        print("\n[NOTE] No Roboflow API key provided.")
        print("To download directly via Python API:")
        print("  1. Get your free Roboflow API key at https://app.roboflow.com/")
        print("  2. Run: python download_trash_model.py --api-key YOUR_KEY")
        print("\nAlternatively, download weights manually:")
        print("  1. Visit: https://universe.roboflow.com/search?q=yolov8-trash-detections")
        print(f"  2. Download the trained PyTorch YOLOv8 weights (.pt) and save as '{args.output}'")
        print(f"  3. perception.py will automatically pick up '{args.output}'.\n")
        return

    try:
        from roboflow import Roboflow
        print(f"[DOWNLOAD] Connecting to Roboflow workspace '{args.workspace}'...")
        rf = Roboflow(api_key=args.api_key)
        project = rf.workspace(args.workspace).project(args.project)
        version = project.version(args.version)
        print(f"[DOWNLOAD] Downloading model weights for version {args.version}...")
        model = version.download("yolov8")
        print(f"[DOWNLOAD] Success! Model downloaded to: {model.location}")
        
        # Check for best.pt or weights
        weights_candidates = [
            os.path.join(model.location, "weights", "best.pt"),
            os.path.join(model.location, "best.pt"),
            os.path.join(model.location, "weights", "yolov8n.pt")
        ]
        found = False
        for cand in weights_candidates:
            if os.path.exists(cand):
                import shutil
                shutil.copyfile(cand, args.output)
                print(f"[SETUP] Copied model weights to '{args.output}'.")
                found = True
                break
        if not found:
            print(f"[WARNING] Downloaded dataset/model folder at '{model.location}'. Please point --model to the trained .pt file.")
    except Exception as e:
        print(f"[ERROR] Failed to download via Roboflow API: {e}", file=sys.stderr)
        print("You can also place the weights manually as 'yolov8_trash.pt'.", file=sys.stderr)


if __name__ == "__main__":
    main()
