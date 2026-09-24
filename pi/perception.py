#!/usr/bin/env python3
"""
perception.py - Pi-side perception and pointing coordination pipeline.

Part of Phase 1 (stationary) waste-detection pointing system.
Captures frames, runs YOLO inference, calculates target bearing, confirms with
ultrasonic distance, and commands SG90 servo via Arduino serial.
"""

import argparse
import sys
import time

# Import local serial communication interface and geometry utilities
from serial_interface import ArduinoSerialInterface
from geometry import calculate_bearing_and_angle, HORIZONTAL_FOV, SERVO_DIRECTION

MIN_VALID_DISTANCE_CM = 2.0   # Minimum plausible distance for HC-SR04
MAX_VALID_DISTANCE_CM = 200.0 # Maximum plausible distance for confirmation


def parse_args():
    parser = argparse.ArgumentParser(
        description="Phase 1 Waste-Detection Pointing System (Perception Pipeline)"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="yolov8n.pt",
        help="Path or name of YOLO model checkpoint (default: yolov8n.pt)",
    )
    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help="Video source index or path (default: 0 for default camera)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=320,
        help="Inference frame resolution width/height (default: 320)",
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
        help="Horizontal field of view in degrees (default: 62.2)",
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
        help="Display OpenCV live visualization window with overlays (default: True)",
    )
    parser.add_argument(
        "--no-show",
        dest="show",
        action="store_false",
        help="Disable GUI preview window (useful for headless Pi runs)",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 70)
    print("  Waste-Detection Pointing System — Phase 1 (Stationary)")
    print("=" * 70)
    print(f"  Model Checkpoint : {args.model}")
    print(f"  Camera Source    : {args.source}")
    print(f"  Inference Size   : {args.imgsz}x{args.imgsz}")
    print(f"  Confidence Cutoff: {args.conf}")
    print(f"  Serial Port      : {args.port} @ {args.baud} baud (Mock: {args.mock})")
    print(f"  Camera FOV       : {args.fov}°")
    print(f"  Servo Direction  : {args.servo_direction:+d}")
    print(f"  Distance Bounds  : [{args.min_dist:.1f}, {args.max_dist:.1f}] cm")
    print("=" * 70)

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

    print(f"[INIT] Loading YOLO model '{args.model}'...")
    try:
        model = YOLO(args.model)
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

    print("[SYSTEM] Starting perception loop. Press Ctrl+C (or 'q' in preview) to stop.")
    frame_idx = 0

    is_file_source = not use_picam and isinstance(src, str) and not src.isdigit()
    consecutive_grab_failures = 0

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

            # 4. Run YOLO Inference (direct call to avoid ARM instruction issues on Pi 4)
            results = model(
                frame,
                conf=args.conf,
                verbose=False,
            )

            # 5. Extract Detections
            target = None
            boxes = results[0].boxes if len(results) > 0 else None

            if boxes is not None and len(boxes) > 0:
                best_conf = -1.0
                best_box_idx = -1

                for idx, box in enumerate(boxes):
                    conf = float(box.conf[0].cpu().numpy())
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

                    target = {
                        "class": cls_name,
                        "conf": best_conf,
                        "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        "center": (x_center, y_center),
                    }

            # 6. Target Decision & Fusion Logic
            if target is None:
                # No target detected: don't send serial command; log and continue
                print(f"[FRAME {frame_idx:05d}] No detection — scanning...")
                status_text = "Status: Scanning (No target)"
                status_color = (128, 128, 128)
            else:
                x_center, y_center = target["center"]
                bearing, servo_angle = calculate_bearing_and_angle(
                    x_center=x_center,
                    img_width=w,
                    fov=args.fov,
                    servo_direction=args.servo_direction,
                )

                # Query ultrasonic distance (Q -> Arduino replies R,<cm>)
                dist_cm = serial_iface.query_distance()

                # Confirm distance
                is_confirmed = (args.min_dist <= dist_cm <= args.max_dist)

                if is_confirmed:
                    # Proceed to point
                    serial_iface.send_point(servo_angle)

                    print(
                        f"[FRAME {frame_idx:05d}] CONFIRMED TARGET: "
                        f"class={target['class']} (conf={target['conf']:.2f}) | "
                        f"bearing={bearing:+.1f}° | "
                        f"servo_angle={servo_angle}° | "
                        f"distance={dist_cm:.1f} cm -> POINTED"
                    )
                    status_text = f"CONFIRMED: {target['class']} @ {dist_cm:.1f}cm | Angle: {servo_angle}°"
                    status_color = (0, 255, 0)
                else:
                    # Implausible distance: log unconfirmed warning, do not point
                    print(
                        f"[FRAME {frame_idx:05d}] UNCONFIRMED TARGET: "
                        f"class={target['class']} (conf={target['conf']:.2f}) | "
                        f"bearing={bearing:+.1f}° | "
                        f"servo_angle={servo_angle}° | "
                        f"distance={dist_cm:.1f} cm (OUT OF RANGE / TIMEOUT) -> SKIPPED"
                    )
                    status_text = f"UNCONFIRMED: {target['class']} (Dist: {dist_cm:.1f}cm)"
                    status_color = (0, 0, 255)

            # 7. Live OpenCV Visualization Overlay
            if args.show:
                display_frame = frame.copy()

                # Draw center reference reticle
                cx, cy = w // 2, h // 2
                cv2.line(display_frame, (cx, 0), (cx, h), (70, 70, 70), 1)
                cv2.line(display_frame, (0, cy), (w, cy), (70, 70, 70), 1)

                if target is not None:
                    x1, y1, x2, y2 = target["bbox"]
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), status_color, 2)

                    # Target center marker
                    tx, ty = int(target["center"][0]), int(target["center"][1])
                    cv2.circle(display_frame, (tx, ty), 5, (0, 255, 255), -1)

                    # Vector from camera center to target
                    cv2.arrowedLine(
                        display_frame, (cx, cy), (tx, ty), (255, 200, 0), 2, tipLength=0.1
                    )

                    label = f"{target['class']} {target['conf']:.2f}"
                    cv2.putText(
                        display_frame,
                        label,
                        (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        status_color,
                        2,
                    )

                # Status bar at top
                cv2.rectangle(display_frame, (0, 0), (w, 28), (20, 20, 20), -1)
                cv2.putText(
                    display_frame,
                    status_text,
                    (8, 19),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    status_color,
                    1,
                )

                cv2.imshow("Waste-Detection Pointing System (Phase 1)", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("[USER] 'q' pressed. Exiting...")
                    break

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
            cv2.destroyAllWindows()
        serial_iface.close()
        print("[SYSTEM] Perception pipeline terminated.")


if __name__ == "__main__":
    main()
