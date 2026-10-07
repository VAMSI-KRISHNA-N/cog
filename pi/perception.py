#!/usr/bin/env python3
"""
perception.py - Pi-side perception, continuous tracking, and stability-gated pointing.

Phase 1 (stationary) waste-detection pointing system.
Continuously captures frames, runs YOLO inference (10-class waste detector),
tracks target bearing live, updates servo every qualifying frame, estimates distance
from bounding box height, and gates ultrasonic confirmation on steady, centered targets.
"""

import argparse
import os
import sys
import time

# Import local serial communication interface and geometry utilities
from serial_interface import ArduinoSerialInterface
from geometry import (
    calculate_bearing_and_angle,
    determine_direction,
    estimate_distance_from_bbox,
    calibrate_focal_length,
    HORIZONTAL_FOV,
    SERVO_DIRECTION,
    DEADBAND_DEG,
    FOCAL_LENGTH_PX,
    CLASS_REAL_HEIGHTS_CM,
)

# 10 Standard Waste Classes from Roboflow Universe yolov8-trash-detections
WASTE_CLASSES = {
    "paper",
    "can",
    "cardboard",
    "plastic",
    "plastic bag",
    "plastic bottle",
    "battery",
    "glass bottle",
    "pop tab",
    "drink carton",
}

# Ultrasonic Validation Bounds (cm)
MIN_VALID_DISTANCE_CM = 2.0
MAX_VALID_DISTANCE_CM = 200.0

# Stability Confirmation Threshold (consecutive centered frames before ultrasonic query)
DEFAULT_STABLE_FRAMES = 5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Waste Robot Phase 1 - Continuous Tracking & Stability-Gated Pointing"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="yolov8_trash.pt",
        help="Path or name of YOLO model checkpoint (default: yolov8_trash.pt)",
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="Video source index or path (default: 0 for Pi Camera / default webcam)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=320,
        help="Camera capture and inference resolution width/height (default: 320)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.25,
        help="Confidence detection threshold (default: 0.25)",
    )
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="Arduino serial port (e.g. /dev/ttyACM0 or COM3)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run serial interface in mock mode without physical Arduino",
    )
    parser.add_argument(
        "--mock-dist",
        type=float,
        default=45.0,
        help="Simulated ultrasonic distance in cm when running in mock mode (default: 45.0)",
    )
    parser.add_argument(
        "--servo-direction",
        type=int,
        default=SERVO_DIRECTION,
        choices=[1, -1],
        help="Servo direction calibration multiplier: +1 or -1 (default: 1)",
    )
    parser.add_argument(
        "--fov",
        type=float,
        default=HORIZONTAL_FOV,
        help="Horizontal field of view in degrees (default: 62.2 for Pi Camera V2)",
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=DEADBAND_DEG,
        help="Angular deadband in degrees for 'center' classification (default: 3.0)",
    )
    parser.add_argument(
        "--stable-frames",
        type=int,
        default=DEFAULT_STABLE_FRAMES,
        help="Consecutive centered frames required before triggering ultrasonic read (default: 5)",
    )
    parser.add_argument(
        "--focal-length",
        type=float,
        default=FOCAL_LENGTH_PX,
        help="Calibrated camera focal length in pixels for distance estimation (default: 320.0)",
    )
    parser.add_argument(
        "--min-dist",
        type=float,
        default=MIN_VALID_DISTANCE_CM,
        help="Minimum valid ultrasonic distance in cm (default: 2.0)",
    )
    parser.add_argument(
        "--max-dist",
        type=float,
        default=MAX_VALID_DISTANCE_CM,
        help="Maximum valid ultrasonic distance in cm (default: 200.0)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        default=True,
        help="Display live preview window with overlays (default: True)",
    )
    parser.add_argument(
        "--no-show",
        dest="show",
        action="store_false",
        help="Disable GUI preview window (useful for headless Pi runs)",
    )
    parser.add_argument(
        "--calibrate-focal",
        nargs=5,
        type=float,
        metavar=("REAL_H", "D1_CM", "H1_PX", "D2_CM", "H2_PX"),
        help="Helper to compute calibrated FOCAL_LENGTH_PX from two distance measurements and exit.",
    )

    return parser.parse_args()


def is_qualifying_waste(cls_name: str, is_placeholder_model: bool = False) -> bool:
    """
    Checks if a detected class corresponds to a qualifying waste item.
    Retains real class names for the 10 waste classes.
    """
    norm = cls_name.lower().strip()
    if norm in WASTE_CLASSES:
        return True
    # If running with generic placeholder yolov8n.pt before trash weights are placed,
    # allow standard test objects so the user can test immediately.
    if is_placeholder_model and norm in {
        "bottle", "cup", "cell phone", "mouse", "apple", "banana", "fork", "knife", "spoon"
    }:
        return True
    return False


def resolve_model_path(requested_path: str) -> tuple[str, bool]:
    """
    Resolves the model checkpoint path. Falls back to yolov8n.pt if
    the requested waste model has not yet been downloaded.
    """
    if os.path.exists(requested_path):
        return requested_path, False

    # Check common alternate locations
    candidates = [
        os.path.join("models", requested_path),
        "yolov8_trash.pt",
        os.path.join("models", "yolov8_trash.pt"),
    ]
    for cand in candidates:
        if os.path.exists(cand):
            return cand, False

    # Graceful fallback to yolov8n.pt
    if os.path.exists("yolov8n.pt"):
        print(
            f"[WARNING] Requested waste model '{requested_path}' not found.\n"
            f"          Falling back to placeholder 'yolov8n.pt'.\n"
            f"          Run 'python download_trash_model.py' to get the 10-class waste weights.\n",
            file=sys.stderr,
        )
        return "yolov8n.pt", True

    # Return requested path even if absent so YOLO loader can raise or auto-fetch
    return requested_path, False


def main():
    args = parse_args()

    # Handle one-time focal length calibration helper if invoked
    if args.calibrate_focal is not None:
        real_h, d1, h1, d2, h2 = args.calibrate_focal
        f1, f2, avg_f, diff_pct = calibrate_focal_length(h1, d1, real_h, h2, d2, real_h)
        print("=" * 60)
        print("         FOCAL LENGTH CALIBRATION RESULTS               ")
        print("=" * 60)
        print(f"  Reference Object Real Height: {real_h:.1f} cm")
        print(f"  Measurement 1 (d={d1:.1f}cm, h={h1:.1f}px): f = {f1:.1f} px")
        print(f"  Measurement 2 (d={d2:.1f}cm, h={h2:.1f}px): f = {f2:.1f} px")
        print(f"  Difference: {diff_pct:.1f}%")
        print(f"  --> RECOMMENDED FOCAL_LENGTH_PX: {avg_f:.1f}")
        if diff_pct > 20.0:
            print("  [WARNING] Measurements disagree by >20%! Consider re-measuring.")
        else:
            print("  [SUCCESS] Measurements consistent. Use --focal-length " + f"{avg_f:.1f}")
        print("=" * 60)
        return

    model_path, is_placeholder = resolve_model_path(args.model)

    print("=" * 75)
    print("  Waste Robot — Phase 1 Continuous Tracking & Pointing System")
    print("=" * 75)
    print(f"  Model Checkpoint : {model_path} (Placeholder: {is_placeholder})")
    print(f"  Camera Source    : {args.source}")
    print(f"  Capture Res      : {args.imgsz}x{args.imgsz}")
    print(f"  Confidence Cutoff: {args.conf}")
    print(f"  Deadband (Center): ±{args.deadband}°")
    print(f"  Stability Trigger: {args.stable_frames} consecutive centered frames")
    print(f"  Focal Length (px): {args.focal_length}")
    print(f"  Serial Port      : {args.port} @ {args.baud} baud (Mock: {args.mock})")
    print(f"  Servo Direction  : {args.servo_direction:+d}")
    print(f"  Ultrasonic Range : [{args.min_dist:.1f}, {args.max_dist:.1f}] cm")
    print("=" * 75)

    # 1. Initialize Serial Interface
    serial_iface = ArduinoSerialInterface(
        port=args.port,
        baudrate=args.baud,
        timeout=1.5,
        mock=args.mock,
        mock_distance=args.mock_dist,
    )

    try:
        serial_iface.connect()
    except Exception as e:
        print(f"[FATAL] Could not initialize serial interface: {e}", file=sys.stderr)
        print("[HINT] If testing without hardware, run with '--mock'.", file=sys.stderr)
        sys.exit(1)

    # 2. Check and Import Vision Dependencies
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
    except ImportError as e:
        print(f"[FATAL] Missing required computer vision library: {e}", file=sys.stderr)
        print("Please install requirements using: pip install -r requirements.txt", file=sys.stderr)
        serial_iface.close()
        sys.exit(1)

    print(f"[INIT] Loading YOLO model '{model_path}'...")
    try:
        model = YOLO(model_path)
        class_names = model.names
        print(f"[INIT] Model loaded successfully. Found {len(class_names)} classes.")
    except Exception as e:
        print(f"[FATAL] Failed to load YOLO model: {e}", file=sys.stderr)
        serial_iface.close()
        sys.exit(1)

    # 3. Open Video Stream (Picamera2 for native Pi Camera, or OpenCV VideoCapture fallback)
    use_picam = False
    picam2 = None
    cap = None

    if str(args.source) == "0":
        try:
            from picamera2 import Picamera2
            print("[INIT] Attempting native Picamera2 initialization...")
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(
                main={"size": (args.imgsz, args.imgsz), "format": "BGR888"}
            )
            picam2.configure(config)
            picam2.start()
            use_picam = True
            print(f"[INIT] Native Picamera2 initialized successfully at {args.imgsz}x{args.imgsz}.")
        except Exception as e:
            print(f"[INIT] Picamera2 not active or unavailable ({e}); falling back to OpenCV VideoCapture...")
            use_picam = False

    if not use_picam:
        src = int(args.source) if args.source.isdigit() else args.source
        print(f"[INIT] Opening camera source via OpenCV: {src}...")
        cap = cv2.VideoCapture(src)

        if not cap.isOpened():
            print(f"[FATAL] Could not open video source '{src}'.", file=sys.stderr)
            serial_iface.close()
            sys.exit(1)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.imgsz)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imgsz)

    # Optimize PyTorch CPU settings for Raspberry Pi 4B (4 cores)
    try:
        import torch
        torch.set_num_threads(4)
    except Exception:
        pass

    print("[SYSTEM] Starting continuous tracking loop. Press Ctrl+C (or 'q' in preview) to stop.\n")
    frame_idx = 0
    is_file_source = not use_picam and isinstance(src, str) and not src.isdigit()
    consecutive_grab_failures = 0

    # Continuous Tracking & Stability State
    last_servo_angle = 90
    consecutive_stable_frames = 0
    is_locked_and_confirmed = False
    last_confirmed_dist = None

    try:
        while True:
            if use_picam:
                frame = picam2.capture_array()
                ret = True
            else:
                ret, frame = cap.read()
                if not ret or frame is None:
                    if is_file_source:
                        print("[INFO] Reached end of video file.")
                        break
                    consecutive_grab_failures += 1
                    if consecutive_grab_failures > 30:
                        print("[FATAL] Camera disconnected or continuous frame drop. Exiting...", file=sys.stderr)
                        break
                    print("[WARNING] Failed to grab camera frame. Retrying...", file=sys.stderr)
                    time.sleep(0.05)
                    continue

            consecutive_grab_failures = 0
            frame_idx += 1
            h, w = frame.shape[:2]

            # 4. Run YOLO Inference (avoid passing imgsz directly to prevent ARM NEON crash on Pi 4)
            try:
                import torch
                with torch.inference_mode():
                    results = model(
                        frame,
                        conf=args.conf,
                        verbose=False,
                    )
            except Exception:
                results = model(
                    frame,
                    conf=args.conf,
                    verbose=False,
                )

            # 5. Extract Qualifying Waste Detections
            target = None
            boxes = results[0].boxes if len(results) > 0 else None

            if boxes is not None and len(boxes) > 0:
                best_conf = -1.0
                best_box_idx = -1

                for idx, box in enumerate(boxes):
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = class_names.get(cls_id, f"class_{cls_id}")

                    # Check if detection qualifies under waste categories
                    if is_qualifying_waste(cls_name, is_placeholder):
                        if conf > best_conf:
                            best_conf = conf
                            best_box_idx = idx

                if best_box_idx >= 0:
                    best_box = boxes[best_box_idx]
                    xyxy = best_box.xyxy[0].cpu().numpy()
                    cls_id = int(best_box.cls[0].cpu().numpy())
                    cls_name = class_names.get(cls_id, f"class_{cls_id}")

                    x1, y1, x2, y2 = xyxy
                    x_center = (x1 + x2) / 2.0
                    y_center = (y1 + y2) / 2.0
                    pixel_height = max(1.0, float(y2 - y1))

                    target = {
                        "class": cls_name,
                        "conf": best_conf,
                        "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        "center": (x_center, y_center),
                        "pixel_height": pixel_height,
                    }

            # 6. Continuous Tracking & Stability-Gated Fusion Logic
            if target is not None:
                x_center, _ = target["center"]

                # A. Bearing & Direction
                bearing, servo_angle = calculate_bearing_and_angle(
                    x_center=x_center,
                    img_width=w,
                    fov=args.fov,
                    servo_direction=args.servo_direction,
                )
                direction = determine_direction(bearing, args.deadband)

                # B. Bounding-Box Distance Estimation
                distance_est = estimate_distance_from_bbox(
                    target["class"],
                    target["pixel_height"],
                    args.focal_length,
                )
                dist_str = f"{distance_est:.1f}cm" if distance_est is not None else "unknown"

                # C. CONTINUOUS TRACKING: Servo updates EVERY qualifying frame
                serial_iface.send_point(servo_angle)
                last_servo_angle = servo_angle

                # D. STABILITY-GATED CONFIRMATION
                # Track consecutive frames where bearing stays within deadband (target centered and holding)
                if direction == "center":
                    consecutive_stable_frames += 1
                    # Fire ultrasonic query only when stability threshold is reached for this lock
                    if consecutive_stable_frames >= args.stable_frames and not is_locked_and_confirmed:
                        dist_cm = serial_iface.query_distance()
                        if args.min_dist <= dist_cm <= args.max_dist:
                            is_locked_and_confirmed = True
                            last_confirmed_dist = dist_cm
                            print(
                                f"\n>>> CONFIRMED: {target['class']} @ {dist_cm:.1f}cm "
                                f"(Stable lock at {servo_angle}°)\n"
                            )
                        else:
                            print(
                                f"\n[ULTRASONIC] Sensor read {dist_cm:.1f}cm (implausible/timeout); continuing tracking...\n"
                            )
                else:
                    # Target drifted out of deadband: reset stability counter & confirmation lock
                    consecutive_stable_frames = 0
                    is_locked_and_confirmed = False
                    last_confirmed_dist = None

                # E. Frame Logging
                print(
                    f"[FRAME {frame_idx:05d}] TARGET: class={target['class']:<14} | "
                    f"conf={target['conf']:.2f} | "
                    f"bearing={bearing:+5.1f}° ({direction:<6}) | "
                    f"dist_est={dist_str:<8} | "
                    f"servo={servo_angle:3d}° | "
                    f"stable={consecutive_stable_frames}/{args.stable_frames}"
                )

                if is_locked_and_confirmed:
                    status_text = f"CONFIRMED: {target['class']} @ {last_confirmed_dist:.1f}cm | Servo: {servo_angle}°"
                    status_color = (0, 255, 0)
                else:
                    status_text = f"TRACKING: {target['class']} ({direction}) | Est: {dist_str} | Servo: {servo_angle}°"
                    status_color = (0, 220, 255) if direction == "center" else (0, 165, 255)

            else:
                # No qualifying target: log "not waste", reset stability, servo holds last position
                consecutive_stable_frames = 0
                is_locked_and_confirmed = False
                last_confirmed_dist = None

                print(f"[FRAME {frame_idx:05d}] not waste — no target (holding {last_servo_angle}°)")
                status_text = f"Status: not waste — no target (holding {last_servo_angle}°)"
                status_color = (128, 128, 128)

            # 7. Live OpenCV Visualization Overlay
            if args.show:
                try:
                    display_frame = frame.copy()
                    cx, cy = w // 2, h // 2

                    # Draw center line and deadband corridor
                    cv2.line(display_frame, (cx, 0), (cx, h), (90, 90, 90), 1)
                    cv2.line(display_frame, (0, cy), (w, cy), (90, 90, 90), 1)

                    deadband_px = int((args.deadband / args.fov) * w)
                    cv2.line(display_frame, (cx - deadband_px, 0), (cx - deadband_px, h), (40, 120, 40), 1)
                    cv2.line(display_frame, (cx + deadband_px, 0), (cx + deadband_px, h), (40, 120, 40), 1)

                    if target is not None:
                        x1, y1, x2, y2 = target["bbox"]
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), status_color, 2)

                        # Center marker and vector from camera center
                        tx, ty = int(target["center"][0]), int(target["center"][1])
                        cv2.circle(display_frame, (tx, ty), 4, (0, 255, 255), -1)
                        cv2.arrowedLine(display_frame, (cx, cy), (tx, ty), (255, 200, 0), 2, tipLength=0.1)

                        # Target label
                        label = f"{target['class']} {target['conf']:.2f} ({dist_str})"
                        cv2.putText(
                            display_frame,
                            label,
                            (x1, max(y1 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            status_color,
                            1,
                        )

                    # Top Status Banner
                    cv2.rectangle(display_frame, (0, 0), (w, 26), (25, 25, 25), -1)
                    cv2.putText(
                        display_frame,
                        status_text,
                        (6, 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        status_color,
                        1,
                    )

                    cv2.imshow("Waste Robot — Phase 1 Tracker", display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        print("[USER] 'q' pressed. Exiting...")
                        break
                except Exception as e:
                    print(f"[WARNING] GUI preview error ({e}); disabling display.")
                    args.show = False

    except KeyboardInterrupt:
        print("\n[USER] Interrupted by keyboard. Exiting cleanly...")
    finally:
        if use_picam and picam2:
            try:
                picam2.stop()
            except Exception:
                pass
        elif cap:
            cap.release()

        if args.show:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        serial_iface.close()
        print("[SYSTEM] Perception pipeline terminated.")


if __name__ == "__main__":
    main()
