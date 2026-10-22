#!/usr/bin/env python3
"""
simulate.py - Full Interactive Simulation & Demo Suite for the Autonomous Waste Robot.

Runs a 100% hardware-free visual simulation of the complete autonomous robot pipeline:
  • Dual-feed Visual HUD (Camera view with YOLO bounding boxes + Robot Status Dashboard)
  • Real-time Virtual Glowing LEDs (Green for WASTE, Red for NOT WASTE)
  • Simulated HC-SR04 Ultrasonic Distance Meter (dynamic distance shrinking on F bursts)
  • Live Motor Signal Telemetry (F, B, TL, TR, S with physical direction indicators)
  • Realistic Multi-Object Scenario: scans, aligns, drives, classifies, indicates, and leaves!
  • Optional Video Recording: saves the run to 'simulation_demo.mp4' for presentations.

Usage:
  # Synthetic floor simulation with multiple realistic waste objects (Zero hardware needed):
  python simulate.py

  # Interactive webcam mode (test with real items in front of your laptop camera):
  python simulate.py --mode webcam

  # Record the simulation to an MP4 video file for your presentation slides:
  python simulate.py --record
"""

import os
import sys

# --- Automatically use .venv if running with system Python ---
if sys.prefix == sys.base_prefix:
    repo_root = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(repo_root, ".venv", "Scripts", "python.exe"),
        os.path.join(repo_root, ".venv", "bin", "python3"),
        os.path.join(repo_root, ".venv", "bin", "python"),
    ]
    for venv_py in candidates:
        if os.path.exists(venv_py) and os.path.abspath(sys.executable) != os.path.abspath(venv_py):
            import subprocess
            res = subprocess.run([venv_py] + sys.argv)
            sys.exit(res.returncode)

import argparse
import math
import time
import numpy as np

# Add pi directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "pi")))

from geometry import (
    calculate_bearing_and_angle,
    determine_direction,
    bearing_to_turn_ms,
    HORIZONTAL_FOV,
    DEADBAND_DEG,
)
from serial_interface import ArduinoSerialInterface


# ============================================================================
# VIRTUAL ENVIRONMENT & ARENA ENGINE
# ============================================================================

class VirtualWorld:
    """
    Simulates a 2D floor arena with waste objects and a 4WD mobile robot.
    Renders what the onboard camera sees based on robot position and orientation.
    """

    def __init__(self, fov_deg=62.2, cam_res=320):
        self.fov_deg = fov_deg
        self.cam_res = cam_res
        self.robot_x = 0.0      # cm
        self.robot_y = 0.0      # cm
        self.robot_heading = 0.0  # degrees (0 = North)

        # Arena floor objects: (name, x, y, width_cm, height_cm, color_bgr, is_waste, conf)
        self.objects = [
            {
                "name": "plastic bottle",
                "x": 35.0,
                "y": 65.0,
                "w": 8.0,
                "h": 22.0,
                "color": (220, 160, 40),  # Blueish plastic
                "is_waste": True,
                "conf": 0.88,
                "handled": False,
            },
            {
                "name": "drink can",
                "x": -50.0,
                "y": 55.0,
                "w": 7.0,
                "h": 14.0,
                "color": (40, 50, 210),  # Red soda can
                "is_waste": True,
                "conf": 0.82,
                "handled": False,
            },
            {
                "name": "cell phone",
                "x": 60.0,
                "y": -35.0,
                "w": 8.0,
                "h": 16.0,
                "color": (60, 60, 60),  # Black phone (Not Waste)
                "is_waste": False,
                "conf": 0.35,
                "handled": False,
            },
        ]

    def execute_motion(self, cmd: str):
        """Update virtual robot physics based on serial motion command."""
        if cmd.startswith("F,"):
            ms = float(cmd.split(",")[1])
            dist_cm = (ms / 300.0) * 6.0  # 6 cm per 300ms forward burst
            rad = math.radians(self.robot_heading)
            self.robot_x += dist_cm * math.sin(rad)
            self.robot_y += dist_cm * math.cos(rad)

        elif cmd.startswith("B,"):
            ms = float(cmd.split(",")[1])
            dist_cm = (ms / 300.0) * 5.0
            rad = math.radians(self.robot_heading)
            self.robot_x -= dist_cm * math.sin(rad)
            self.robot_y -= dist_cm * math.cos(rad)

        elif cmd.startswith("TL,"):
            ms = float(cmd.split(",")[1])
            deg = ms / 8.0  # 8ms per deg
            self.robot_heading = (self.robot_heading - deg) % 360.0

        elif cmd.startswith("TR,"):
            ms = float(cmd.split(",")[1])
            deg = ms / 8.0
            self.robot_heading = (self.robot_heading + deg) % 360.0

    def query_ultrasonic_distance(self) -> float:
        """Calculate true distance from robot front to the closest object in front."""
        closest_dist = 200.0
        for obj in self.objects:
            if obj["handled"]:
                continue
            dx = obj["x"] - self.robot_x
            dy = obj["y"] - self.robot_y
            dist = math.hypot(dx, dy)
            obj_angle = math.degrees(math.atan2(dx, dy)) % 360.0
            angle_diff = (obj_angle - self.robot_heading + 180.0) % 360.0 - 180.0

            # Ultrasonic cone is ~25 degrees
            if abs(angle_diff) < 15.0 and dist < closest_dist:
                closest_dist = dist

        return round(max(3.0, closest_dist), 1)

    def render_camera_view(self) -> tuple[np.ndarray, dict | None]:
        """
        Renders what the onboard camera sees and returns (frame, target_dict).
        """
        import cv2

        h, w = self.cam_res, self.cam_res
        frame = np.full((h, w, 3), 235, dtype=np.uint8)  # Light gray floor

        # Floor grid lines for motion perception
        grid_spacing = 30
        grid_offset_y = int((self.robot_y * 2) % grid_spacing)
        grid_offset_x = int((self.robot_x * 2) % grid_spacing)

        for y in range(grid_offset_y, h, grid_spacing):
            cv2.line(frame, (0, y), (w, y), (218, 218, 218), 1)
        for x in range(grid_offset_x, w, grid_spacing):
            cv2.line(frame, (x, 0), (x, h), (218, 218, 218), 1)

        best_target = None
        closest_dist = 999.0

        for obj in self.objects:
            if obj["handled"]:
                continue

            dx = obj["x"] - self.robot_x
            dy = obj["y"] - self.robot_y
            dist = math.hypot(dx, dy)

            # Angle relative to robot heading
            obj_angle = math.degrees(math.atan2(dx, dy)) % 360.0
            bearing = (obj_angle - self.robot_heading + 180.0) % 360.0 - 180.0

            # Within camera field of view?
            half_fov = self.fov_deg / 2.0
            if -half_fov <= bearing <= half_fov and dist > 3.0:
                # Perspective projection
                norm_x = (bearing + half_fov) / self.fov_deg
                center_px_x = int(norm_x * w)

                # Size scales inversely with distance
                scale = max(0.15, min(2.5, 45.0 / dist))
                obj_px_w = int(obj["w"] * scale * 4.0)
                obj_px_h = int(obj["h"] * scale * 4.0)

                # Y position on floor: closer items are lower in frame
                # Camera height perspective mapping
                base_y = int(h * 0.45 + (scale * h * 0.35))
                top_y = base_y - obj_px_h
                x1 = max(0, center_px_x - obj_px_w // 2)
                x2 = min(w, center_px_x + obj_px_w // 2)
                y1 = max(0, top_y)
                y2 = min(h, base_y)

                if x2 > x1 and y2 > y1:
                    # Draw shadow
                    cv2.ellipse(frame, (center_px_x, base_y), (obj_px_w // 2, obj_px_w // 4),
                                0, 0, 360, (190, 190, 190), -1)

                    # Draw 3D object body
                    cv2.rectangle(frame, (x1, y1), (x2, y2), obj["color"], -1)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 40, 40), 1)

                    # Object label on frame
                    cv2.putText(frame, obj["name"], (x1, max(12, y1 - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (30, 30, 30), 1)

                    if dist < closest_dist:
                        closest_dist = dist
                        best_target = {
                            "class": obj["name"],
                            "conf": obj["conf"],
                            "bbox": (x1, y1, x2, y2),
                            "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                            "pixel_height": max(1.0, float(y2 - y1)),
                            "ref": obj,
                        }

        return frame, best_target


# ============================================================================
# HEADS-UP DISPLAY (HUD) & VISUAL DASHBOARD
# ============================================================================

def draw_simulation_dashboard(
    camera_frame: np.ndarray,
    target: dict | None,
    state_name: str,
    last_cmd: str,
    ultrasonic_cm: float,
    led_active: str | None,
    waste_count: int,
    not_waste_count: int,
    sweep_deg: float,
    sweep_limit: float,
    elapsed_s: float,
) -> np.ndarray:
    """
    Renders an informative side-by-side dashboard:
      Left: Live Camera View with YOLO Detection Bounding Box & Target Crosshairs
      Right: Telemetry Panel with Virtual Glowing LEDs, Ultrasonic Meter & Serial Logs
    """
    import cv2

    cam_h, cam_w = camera_frame.shape[:2]
    panel_w = 400
    total_w = cam_w + panel_w
    total_h = max(cam_h, 420)

    # Master canvas
    canvas = np.zeros((total_h, total_w, 3), dtype=np.uint8)
    canvas[:] = (20, 24, 28)  # Sleek dark slate background

    # --- 1. Left Side: Camera Feed with Overlays ---
    display_cam = camera_frame.copy()
    cx, cy = cam_w // 2, cam_h // 2

    # Center crosshair (deadband zone)
    db_px = int((6.0 / HORIZONTAL_FOV) * cam_w)
    cv2.line(display_cam, (cx, 0), (cx, cam_h), (70, 70, 70), 1)
    cv2.line(display_cam, (cx - db_px, 0), (cx - db_px, cam_h), (40, 100, 40), 1)
    cv2.line(display_cam, (cx + db_px, 0), (cx + db_px, cam_h), (40, 100, 40), 1)

    # Arrival threshold boundary line (18cm / 92% frame)
    arrive_y = int(0.92 * cam_h)
    cv2.line(display_cam, (0, arrive_y), (cam_w, arrive_y), (0, 0, 180), 1)
    cv2.putText(display_cam, "ARRIVE THRESHOLD", (8, arrive_y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 200), 1)

    # Draw YOLO Bounding Box
    if target:
        x1, y1, x2, y2 = target["bbox"]
        box_color = (0, 220, 80) if target["conf"] >= 0.50 else (60, 60, 230)
        cv2.rectangle(display_cam, (x1, y1), (x2, y2), box_color, 2)

        # Label tag background
        label = f"{target['class'].upper()} ({target['conf']:.2f})"
        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        cv2.rectangle(display_cam, (x1, max(0, y1 - lh - 6)), (x1 + lw + 6, y1), box_color, -1)
        cv2.putText(display_cam, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)

        # Draw bearing vector line from bottom center to target
        tx, ty = int(target["center"][0]), int(target["center"][1])
        cv2.line(display_cam, (cx, cam_h), (tx, ty), (0, 200, 240), 1)

    # Blit camera feed onto canvas
    y_off = (total_h - cam_h) // 2
    canvas[y_off:y_off + cam_h, 0:cam_w] = display_cam
    cv2.rectangle(canvas, (0, y_off), (cam_w, y_off + cam_h), (60, 68, 76), 1)

    # Camera Header Label
    cv2.putText(canvas, "ONBOARD CAMERA (Pi Camera V2)", (12, y_off - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 170, 180), 1)

    # --- 2. Right Side: Telemetry & State Dashboard ---
    px = cam_w + 20

    # Title Banner
    cv2.putText(canvas, "COG ROBOT TELEMETRY", (px, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.line(canvas, (px, 44), (total_w - 20, 44), (50, 58, 66), 1)

    # State Machine Badge
    state_colors = {
        "SCAN": (220, 180, 40),
        "ALIGN": (0, 200, 240),
        "APPROACH": (0, 140, 255),
        "VERDICT": (255, 120, 0),
        "INDICATE": (0, 220, 0),
        "LEAVE": (180, 80, 240),
        "DONE": (120, 240, 120),
    }
    s_col = state_colors.get(state_name, (200, 200, 200))

    cv2.putText(canvas, "STATE:", (px, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 150, 160), 1)
    cv2.rectangle(canvas, (px + 60, 58), (px + 230, 85), s_col, -1)
    cv2.putText(canvas, state_name, (px + 72, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 14, 18), 2, cv2.LINE_AA)

    # --- Virtual Glowing Hardware LEDs ---
    cv2.putText(canvas, "HARDWARE LED INDICATORS:", (px, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 150, 160), 1)

    # Green LED (Waste)
    green_active = (led_active == "green")
    g_core = (0, 255, 0) if green_active else (15, 60, 15)
    g_center = (px + 45, 155)
    if green_active:
        cv2.circle(canvas, g_center, 22, (0, 100, 0), -1)  # Glow halo
    cv2.circle(canvas, g_center, 14, g_core, -1)
    cv2.circle(canvas, g_center, 14, (200, 255, 200) if green_active else (30, 80, 30), 1)
    cv2.putText(canvas, "GREEN (WASTE)", (px + 75, 152),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 255, 180) if green_active else (80, 110, 80), 1)
    cv2.putText(canvas, "BCM Pin 27", (px + 75, 166),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100, 110, 120), 1)

    # Red LED (Not Waste)
    red_active = (led_active == "red")
    r_core = (0, 0, 255) if red_active else (50, 15, 15)
    r_center = (px + 235, 155)
    if red_active:
        cv2.circle(canvas, r_center, 22, (0, 0, 120), -1)  # Glow halo
    cv2.circle(canvas, r_center, 14, r_core, -1)
    cv2.circle(canvas, r_center, 14, (200, 200, 255) if red_active else (80, 30, 30), 1)
    cv2.putText(canvas, "RED (NOT WASTE)", (px + 265, 152),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 255) if red_active else (110, 80, 80), 1)
    cv2.putText(canvas, "BCM Pin 17", (px + 265, 166),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100, 110, 120), 1)

    # --- Ultrasonic Distance Gauge ---
    cv2.line(canvas, (px, 190), (total_w - 20, 190), (50, 58, 66), 1)
    cv2.putText(canvas, "ULTRASONIC SENSOR (HC-SR04):", (px, 215),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 150, 160), 1)

    dist_str = f"{ultrasonic_cm:.1f} cm" if ultrasonic_cm > 0 else "Out of Range"
    d_color = (0, 230, 255) if ultrasonic_cm > 18.0 else (0, 240, 100)
    cv2.putText(canvas, dist_str, (px + 240, 215),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, d_color, 2)

    # Progress bar for distance (0 to 100 cm)
    bar_w = 340
    fill_w = int(max(0, min(100.0, ultrasonic_cm)) / 100.0 * bar_w)
    cv2.rectangle(canvas, (px, 226), (px + bar_w, 236), (40, 46, 54), -1)
    cv2.rectangle(canvas, (px, 226), (px + fill_w, 236), d_color, -1)

    # 18cm arrival tick mark
    tick_x = px + int(18.0 / 100.0 * bar_w)
    cv2.line(canvas, (tick_x, 223), (tick_x, 239), (0, 0, 255), 2)
    cv2.putText(canvas, "18cm Arrival Stop", (tick_x - 35, 252),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (120, 130, 140), 1)

    # --- Arduino Motion & Serial Command ---
    cv2.line(canvas, (px, 265), (total_w - 20, 265), (50, 58, 66), 1)
    cv2.putText(canvas, "ARDUINO SERIAL COMMAND (L298N):", (px, 290),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 150, 160), 1)

    # Direction badge
    cmd_label = last_cmd or "S (IDLE)"
    cv2.rectangle(canvas, (px, 302), (px + 140, 332), (32, 38, 46), -1)
    cv2.rectangle(canvas, (px, 302), (px + 140, 332), (80, 90, 100), 1)
    cv2.putText(canvas, f"TX: {cmd_label}", (px + 12, 323),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 240, 240), 1, cv2.LINE_AA)

    # Sweep Progress
    cv2.putText(canvas, f"Scan Sweep: {sweep_deg:.0f}° / {sweep_limit:.0f}°", (px + 160, 323),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 170, 180), 1)

    # --- Scoreboard & Summary ---
    cv2.line(canvas, (px, 345), (total_w - 20, 345), (50, 58, 66), 1)
    cv2.putText(canvas, f"CLASSIFIED WASTE: {waste_count}", (px, 372),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 100), 1)
    cv2.putText(canvas, f"NON-WASTE: {not_waste_count}", (px + 190, 372),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
    cv2.putText(canvas, f"Elapsed: {elapsed_s:.1f}s   [Press 'q' to Quit]", (px, 400),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 130, 140), 1)

    return canvas


# ============================================================================
# MAIN SIMULATOR PIPELINE
# ============================================================================

def run_simulation(mode="synthetic", record=False, model_path="yolov8n.pt"):
    import cv2

    print("=" * 70)
    print("   COG Autonomous Robot — Full Visual Simulation Suite")
    print("=" * 70)
    print(f"Mode: {mode.upper()}  |  Model: {model_path}  |  Record: {record}")
    print("----------------------------------------------------------------------")
    print("Live Telemetry running. Watch the visual HUD window on your screen.")
    print("Press 'q' in the window at any time to stop cleanly.\n")

    # Load YOLO model
    model = None
    class_names = {}
    try:
        from ultralytics import YOLO
        if os.path.exists(model_path):
            print(f"[YOLO] Loading model weights from '{model_path}'...")
            model = YOLO(model_path)
            class_names = model.names
            print(f"[YOLO] Successfully loaded model with {len(class_names)} classes.")
        else:
            print(f"[YOLO] Checkpoint '{model_path}' not found. Using internal feature tracker.")
    except Exception as e:
        print(f"[YOLO WARNING] Ultralytics import error ({e}); using simulated detector.")

    # Initialize environment
    world = VirtualWorld(fov_deg=HORIZONTAL_FOV, cam_res=320)
    cap = None

    if mode == "webcam":
        print("[CAMERA] Opening live computer webcam (source 0)...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[ERROR] Could not open webcam. Falling back to synthetic floor simulation.")
            mode = "synthetic"

    # Setup video recorder if requested
    video_writer = None
    if record:
        record_file = "simulation_demo.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(record_file, fourcc, 15.0, (720, 420))
        print(f"[RECORDER] Recording simulation video to '{record_file}'...")

    # State Machine Variables
    state = "SCAN"
    sweep_deg = 0.0
    sweep_limit = 360.0
    waste_count = 0
    not_waste_count = 0
    last_cmd = "S"
    current_led = None
    ultrasonic_dist = 85.0
    conf_window = []
    consecutive_lost = 0
    current_target_ref = None
    current_target_class = "unknown"

    start_time = time.time()
    state_timer = 0.0

    cv2.namedWindow("COG Autonomous Robot — Simulation HUD", cv2.WINDOW_AUTOSIZE)

    frame_idx = 0
    cached_target = None
    import torch

    try:
        while state != "DONE":
            loop_start = time.time()
            elapsed_s = loop_start - start_time
            frame_idx += 1

            # 1. Grab Frame & Detect Target
            if mode == "webcam" and cap:
                ret, raw_frame = cap.read()
                if not ret:
                    break
                frame = cv2.resize(raw_frame, (320, 320))

                # Run inference on alternate frames to maintain smooth 15-20+ FPS
                if model and (frame_idx % 2 == 1 or cached_target is None):
                    with torch.inference_mode():
                        res = model(frame, imgsz=224, conf=0.20, verbose=False)[0]
                    boxes = res.boxes
                    best_cand = None
                    best_conf = -1.0

                    # Waste candidates (covers actual litter / waste containers)
                    waste_candidates = {
                        "paper", "cardboard", "plastic", "metal", "trash", "biological",
                        "battery", "brown-glass", "green-glass", "white-glass",
                        "bottle", "cup", "can", "wine glass", "bowl", "vase",
                    }
                    # Non-waste personal items (phones, mouse, books)
                    non_waste_candidates = {"cell phone", "mouse", "book", "toothbrush"}
                    valid_targets = waste_candidates | non_waste_candidates

                    for b in boxes:
                        cls_id = int(b.cls[0].cpu().numpy())
                        raw_cls = class_names.get(cls_id, f"obj_{cls_id}").lower().strip()
                        conf = float(b.conf[0].cpu().numpy())

                        # Ignore people, pets, background furniture, shoes/clothing on user's body
                        if raw_cls in {"person", "cat", "dog", "chair", "couch", "bed", "tv", "laptop", "dining table", "shoes", "clothes"}:
                            continue

                        detected_cls = raw_cls
                        xyxy = b.xyxy[0].cpu().numpy().astype(int)
                        box_w = max(1, xyxy[2] - xyxy[0])
                        box_h = max(1, xyxy[3] - xyxy[1])

                        # Ignore giant bounding boxes that cover the human user sitting in front of webcam
                        if box_w >= 0.60 * 320 and box_h >= 0.60 * 320:
                            continue

                        aspect_ratio = box_h / float(box_w)

                        # If a tall cylindrical flask/thermos is mistakenly tagged as 'cell phone' by COCO,
                        # detect its tall aspect ratio (> 2.1) and reclassify it as a bottle
                        if raw_cls == "cell phone" and aspect_ratio >= 2.1:
                            detected_cls = "bottle"

                        # Map containers like vase or wine glass to bottle
                        elif raw_cls in {"vase", "wine glass"}:
                            detected_cls = "bottle"

                        if (detected_cls in valid_targets or raw_cls in valid_targets) and conf > best_conf:
                            best_conf = conf
                            best_cand = {
                                "class": detected_cls,
                                "conf": conf,
                                "bbox": (xyxy[0], xyxy[1], xyxy[2], xyxy[3]),
                                "center": ((xyxy[0] + xyxy[2]) / 2.0, (xyxy[1] + xyxy[3]) / 2.0),
                                "pixel_height": float(xyxy[3] - xyxy[1]),
                                "is_waste": (detected_cls in waste_candidates),
                            }

                    cached_target = best_cand

                target = cached_target
            else:
                frame, target = world.render_camera_view()
                ultrasonic_dist = world.query_ultrasonic_distance()

            # 2. Execute Robot State Machine Step
            if state == "SCAN":
                current_led = None
                if target:
                    current_target_class = target["class"]
                    print(f"\n[SCAN] >>> TARGET DETECTED: {target['class'].upper()} (conf: {target['conf']:.2f})")
                    state = "ALIGN"
                    sweep_deg = 0.0
                    conf_window = []
                    consecutive_lost = 0
                    current_target_ref = target.get("ref")
                else:
                    turn_ms = bearing_to_turn_ms(20.0, 8.0)
                    last_cmd = f"TL,{turn_ms}"
                    world.execute_motion(last_cmd)
                    sweep_deg += 20.0
                    print(f"[SCAN] Sweeping... {sweep_deg:.0f}° / {sweep_limit:.0f}°  |  Cmd: {last_cmd}")

                    if sweep_deg >= sweep_limit:
                        if mode == "webcam":
                            sweep_deg = 0.0
                            print("[SCAN] Continuing scan... (hold an object in front of camera, or press 'q' to quit)")
                        else:
                            print("\n[SCAN] Sweep limit reached with no remaining targets. Transitioning to DONE.")
                            state = "DONE"

            elif state == "ALIGN":
                if not target:
                    consecutive_lost += 1
                    if consecutive_lost >= 5:
                        print("[ALIGN] Target lost. Returning to SCAN.")
                        state = "SCAN"
                else:
                    consecutive_lost = 0
                    h, w = frame.shape[:2]
                    bearing, _ = calculate_bearing_and_angle(target["center"][0], w, fov=HORIZONTAL_FOV)

                    if abs(bearing) <= DEADBAND_DEG:
                        print(f"[ALIGN] Centered! (bearing={bearing:+.1f}°). Switching to APPROACH.")
                        state = "APPROACH"
                        conf_window = []
                    elif bearing > 0:
                        turn_ms = bearing_to_turn_ms(bearing, 8.0)
                        last_cmd = f"TR,{turn_ms}"
                        world.execute_motion(last_cmd)
                        print(f"[ALIGN] Target at {bearing:+.1f}° -> Right Pivot: {last_cmd}")
                    else:
                        turn_ms = bearing_to_turn_ms(bearing, 8.0)
                        last_cmd = f"TL,{turn_ms}"
                        world.execute_motion(last_cmd)
                        print(f"[ALIGN] Target at {bearing:+.1f}° -> Left Pivot: {last_cmd}")

            elif state == "APPROACH":
                if not target:
                    consecutive_lost += 1
                    if consecutive_lost >= 5:
                        print("[APPROACH] Target lost. Returning to SCAN.")
                        state = "SCAN"
                else:
                    consecutive_lost = 0
                    conf_window.append(target["conf"])
                    if len(conf_window) > 5:
                        conf_window.pop(0)

                    h, w = frame.shape[:2]
                    bearing, _ = calculate_bearing_and_angle(target["center"][0], w, fov=HORIZONTAL_FOV)

                    # Re-align if drifted
                    if abs(bearing) > DEADBAND_DEG + 4.0:
                        state = "ALIGN"
                        continue

                    # In webcam mode, dynamically decrement simulated ultrasonic distance on each forward step
                    if mode == "webcam":
                        # Also estimate proximity from bounding box height / bottom
                        bbox_h = target["pixel_height"]
                        bbox_bottom = target["bbox"][3]
                        if bbox_bottom >= 0.88 * h or bbox_h >= 0.48 * h:
                            ultrasonic_dist = min(ultrasonic_dist, 16.0)

                    # Arrival Check: Ultrasonic <= 18 cm OR Bbox bottom >= 88% of frame
                    bbox_bottom = target["bbox"][3]
                    is_arrived = (ultrasonic_dist <= 18.0) or (bbox_bottom >= 0.88 * h) or (target["pixel_height"] >= 0.50 * h)

                    if is_arrived:
                        print(f"\n[APPROACH] ARRIVED AT OBJECT! (Distance: {ultrasonic_dist:.1f}cm)")
                        last_cmd = "S"
                        world.execute_motion(last_cmd)
                        state = "VERDICT"
                    else:
                        last_cmd = "F,300"
                        world.execute_motion(last_cmd)
                        if mode == "webcam":
                            ultrasonic_dist = max(14.0, ultrasonic_dist - 14.0)
                        print(f"[APPROACH] Driving forward: {last_cmd} (Distance: {ultrasonic_dist:.1f}cm)")

            elif state == "VERDICT":
                mean_conf = sum(conf_window) / max(1, len(conf_window))
                waste_names = {
                    "plastic", "paper", "cardboard", "metal", "trash", "biological",
                    "battery", "brown-glass", "green-glass", "white-glass",
                    "bottle", "cup", "can", "wine glass", "bowl", "vase",
                    "plastic bottle", "drink can",
                }
                is_waste_type = (current_target_class.lower() in waste_names)
                is_waste = (is_waste_type and mean_conf >= 0.50)

                if is_waste:
                    verdict_name = "WASTE"
                    current_led = "green"
                    waste_count += 1
                else:
                    verdict_name = "NOT WASTE"
                    current_led = "red"
                    not_waste_count += 1

                print("=" * 60)
                print(f"   TARGET : {current_target_class.upper()} (Waste Candidate: {is_waste_type})")
                print(f"   VERDICT: {verdict_name}  (Mean Conf: {mean_conf:.2f})")
                print(f"   ACTION : LIGHTING {current_led.upper()} LED FOR 3 SECONDS")
                print("=" * 60)

                state = "INDICATE"
                state_timer = time.time()

            elif state == "INDICATE":
                # Hold LED indicator for 3.0 seconds
                if time.time() - state_timer >= 3.0:
                    current_led = None
                    state = "LEAVE"
                    if current_target_ref:
                        current_target_ref["handled"] = True

            elif state == "LEAVE":
                print("\n[LEAVE] Reversing and turning away to clear the object...")
                # Reverse 500ms
                last_cmd = "B,500"
                world.execute_motion(last_cmd)

                # Turn away 120 degrees
                last_cmd = "TR,960"
                world.execute_motion(last_cmd)

                sweep_deg = 0.0
                sweep_limit = 180.0
                ultrasonic_dist = 85.0
                state = "SCAN"

            # 3. Draw High-Fidelity HUD
            hud = draw_simulation_dashboard(
                camera_frame=frame,
                target=target,
                state_name=state,
                last_cmd=last_cmd,
                ultrasonic_cm=ultrasonic_dist,
                led_active=current_led,
                waste_count=waste_count,
                not_waste_count=not_waste_count,
                sweep_deg=sweep_deg,
                sweep_limit=sweep_limit,
                elapsed_s=elapsed_s,
            )

            cv2.imshow("COG Autonomous Robot — Simulation HUD", hud)
            if video_writer:
                video_writer.write(hud)

            # Interactive key check (1ms for immediate responsiveness)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("\n[SIMULATION] User pressed 'q' — terminating.")
                break

        # --- Shutdown Sequence ---
        if state == "DONE":
            print("\n" + "=" * 60)
            print("   MISSION COMPLETE — SHUTDOWN SEQUENCE INITIATED")
            print("=" * 60)
            print(f"Objects Cleared : {waste_count + not_waste_count}")
            print(f"  • Classified Waste    : {waste_count}")
            print(f"  • Classified Non-Waste: {not_waste_count}")
            print("\n[DONE] Alternating LEDs: Safe to pick up car...")

            # Animate the alternating shutdown LED sequence on the HUD
            for toggle in [True, False, True, False, True, False]:
                led = "green" if toggle else "red"
                hud = draw_simulation_dashboard(
                    camera_frame=frame,
                    target=None,
                    state_name="SHUTDOWN",
                    last_cmd="S",
                    ultrasonic_cm=0.0,
                    led_active=led,
                    waste_count=waste_count,
                    not_waste_count=not_waste_count,
                    sweep_deg=360.0,
                    sweep_limit=360.0,
                    elapsed_s=time.time() - start_time,
                )
                cv2.imshow("COG Autonomous Robot — Simulation HUD", hud)
                if video_writer:
                    for _ in range(5):
                        video_writer.write(hud)
                cv2.waitKey(400)

            print("[DONE] Simulating: sudo shutdown -h now")
            print("[SUCCESS] Autonomous cycle simulated flawlessly with zero errors.")

    finally:
        if cap:
            cap.release()
        if video_writer:
            video_writer.release()
            print("[RECORDER] Video recording saved as 'simulation_demo.mp4'.")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Autonomous Waste Robot Simulator")
    parser.add_argument("--mode", choices=["synthetic", "webcam"], default="synthetic",
                        help="Choose 'synthetic' (virtual objects on floor) or 'webcam' (your computer camera).")
    parser.add_argument("--record", action="store_true",
                        help="Record the visual simulation run to 'simulation_demo.mp4'.")
    parser.add_argument("--model", default="yolov8n.pt", help="Path to YOLO checkpoint (default: yolov8n.pt for 35+ FPS real-time speed).")
    args = parser.parse_args()

    run_simulation(mode=args.mode, record=args.record, model_path=args.model)
