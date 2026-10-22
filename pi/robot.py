#!/usr/bin/env python3
"""
robot.py - Autonomous waste-detection robot: scan, drive, classify, indicate, shut down.

Runs a stop-look-move state machine on a Raspberry Pi 4B controlling a 4WD L298N
car via Arduino Uno over USB serial.  Detects floor objects with YOLOv8, drives to
each one, decides WASTE / NOT WASTE, lights the corresponding LED (green / red),
then moves on.  When no objects remain the Pi shuts down safely.

Usage:
    # Headless (what the systemd service runs):
    python3 pi/robot.py

    # Dev preview with annotated window:
    python3 pi/robot.py --preview

    # Teleoperation calibration mode:
    python3 pi/robot.py --teleop

    # Mock mode (laptop with webcam, no Arduino):
    python3 pi/robot.py --mock
"""

import os
import sys

# --- Automatically use .venv if running with system Python ---
if sys.prefix == sys.base_prefix:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
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
import enum
import signal
import subprocess
import time

# Add pi directory to path so imports work from the repo root
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from serial_interface import ArduinoSerialInterface
from geometry import (
    calculate_bearing_and_angle,
    determine_direction,
    bearing_to_turn_ms,
    HORIZONTAL_FOV,
    DEADBAND_DEG,
)

# ============================================================================
# TUNABLES — named constants (overridable via CLI flags)
# ============================================================================
TURN_MS_PER_DEG = 8.0         # ms of pivot per degree of rotation
DRIVE_MS_PER_CM = 15.0        # ms of forward drive per cm (informational)
FORWARD_BURST_MS = 300         # ms per forward burst toward a target
SCAN_STEP_DEG = 20.0           # degrees per scan pivot step
ARRIVE_CM = 18.0               # ultrasonic distance threshold for arrival
ARRIVE_Y_FRAC = 0.92           # bbox bottom-edge fraction of frame height for arrival
ALIGN_DEADBAND_DEG = 6.0       # degrees of acceptable bearing error during align
CAMERA_SETTLE_S = 0.3          # seconds to wait before grabbing a frame
WASTE_CONF = 0.50              # mean confidence threshold: >= is WASTE
SWEEP_LIMIT_FIRST_DEG = 360.0  # sweep limit on the very first scan
SWEEP_LIMIT_AFTER_DEG = 180.0  # sweep limit on scans after handling an object
REVERSE_MS = 500               # ms to reverse after indicating
TURN_AWAY_MS = 960             # ms to turn away (~120 deg at default calibration)
LED_HOLD_S = 3.0               # seconds to hold LED on after verdict
VOTE_FRAMES = 5                # rolling confidence window size
START_DELAY_S = 10.0           # seconds to wait after ready blink before scanning
MAX_RUN_S = 300.0              # global time limit in seconds
RED_LED_PIN = 17               # BCM pin for red LED
GREEN_LED_PIN = 27             # BCM pin for green LED
CONF_THRESHOLD = 0.25          # YOLO confidence cutoff
IMGSZ = 224                    # capture and inference resolution (224 is fast on Pi 4)
EMERGENCY_STOP_CM = 8.0        # ultrasonic danger threshold
STALE_FRAMES_DISCARD = 2       # frames to discard before inference
MAX_CONSECUTIVE_LOST = 3       # consecutive lost frames before returning to SCAN

# Waste classes covering dedicated waste models and COCO containers
WASTE_CLASSES = {
    "paper", "can", "cardboard", "plastic", "plastic bag",
    "plastic bottle", "battery", "glass bottle", "pop tab", "drink carton",
    "biological", "brown-glass", "green-glass", "metal",
    "trash", "white-glass", "bottle", "cup", "bowl", "vase",
}


# ============================================================================
# STATE MACHINE
# ============================================================================

class State(enum.Enum):
    SCAN = "SCAN"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    VERDICT = "VERDICT"
    INDICATE = "INDICATE"
    LEAVE = "LEAVE"
    DONE = "DONE"


# ============================================================================
# LED CONTROLLER
# ============================================================================

class LEDController:
    """Controls red and green LEDs via RPi.GPIO (BCM) or mock prints."""

    def __init__(self, red_pin: int, green_pin: int, mock: bool = False):
        self.red_pin = red_pin
        self.green_pin = green_pin
        self.mock = mock
        self._gpio = None

        if not mock:
            try:
                import RPi.GPIO as GPIO
                self._gpio = GPIO
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.red_pin, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(self.green_pin, GPIO.OUT, initial=GPIO.LOW)
            except (ImportError, RuntimeError) as e:
                print(f"[LED WARNING] RPi.GPIO unavailable ({e}); falling back to mock LED mode.")
                self.mock = True

    def on(self, color: str):
        pin = self.red_pin if color == "red" else self.green_pin
        if self.mock:
            print(f"[MOCK LED] {color.upper()} ON (BCM {pin})")
        else:
            self._gpio.output(pin, self._gpio.HIGH)

    def off(self):
        if self.mock:
            print("[MOCK LED] ALL OFF")
        else:
            self._gpio.output(self.red_pin, self._gpio.LOW)
            self._gpio.output(self.green_pin, self._gpio.LOW)

    def blink(self, color: str, times: int = 1, on_s: float = 0.3, off_s: float = 0.3):
        for _ in range(times):
            self.on(color)
            time.sleep(on_s)
            self.off()
            time.sleep(off_s)

    def blink_both(self, times: int = 1, on_s: float = 0.3, off_s: float = 0.3):
        for _ in range(times):
            self.on("red")
            self.on("green")
            time.sleep(on_s)
            self.off()
            time.sleep(off_s)

    def alternate_pattern(self, duration_s: float = 10.0, interval_s: float = 1.0):
        """Alternate red/green for `duration_s` — signals 'safe to pick up'."""
        end = time.time() + duration_s
        toggle = True
        while time.time() < end:
            color = "green" if toggle else "red"
            self.on(color)
            time.sleep(interval_s)
            self.off()
            toggle = not toggle

    def cleanup(self):
        self.off()
        if not self.mock and self._gpio:
            try:
                self._gpio.cleanup([self.red_pin, self.green_pin])
            except Exception:
                pass


# ============================================================================
# MODEL RESOLVER
# ============================================================================

def resolve_model_path(requested_path: str) -> tuple[str, bool]:
    """
    Resolves the model checkpoint path. Falls back to yolov8n.pt if
    the requested waste model has not yet been downloaded.
    Returns (path, is_placeholder).
    """
    if os.path.exists(requested_path):
        return requested_path, False

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
        print_placeholder_warning(requested_path)
        return "yolov8n.pt", True

    return requested_path, False


def print_placeholder_warning(requested_path: str):
    """Prints a prominent warning when falling back to yolov8n.pt."""
    yellow = "\033[93m"
    bold = "\033[1m"
    red = "\033[91m"
    reset = "\033[0m"
    box_width = 80

    lines = [
        f"{red}{bold}╔{'═' * (box_width - 2)}╗{reset}",
        f"{red}{bold}║{'  CRITICAL WARNING: USING GENERIC PLACEHOLDER MODEL (yolov8n.pt)  ':^{box_width - 2}}║{reset}",
        f"{red}{bold}╠{'═' * (box_width - 2)}╣{reset}",
        f"{yellow}║  Requested waste model '{requested_path}' was NOT found!{' ' * max(0, box_width - 32 - len(requested_path))}║{reset}",
        f"{yellow}║  Falling back to generic 'yolov8n.pt' (trained on 80 COCO classes).           ║{reset}",
        f"{yellow}║                                                                              ║{reset}",
        f"{yellow}║  ACTION REQUIRED:                                                            ║{reset}",
        f"{yellow}║    1. Run: python download_trash_model.py --api-key <YOUR_ROBOFLOW_KEY>      ║{reset}",
        f"{yellow}║    2. Or download 'yolov8_trash.pt' from Roboflow Universe manually.         ║{reset}",
        f"{red}{bold}╚{'═' * (box_width - 2)}╝{reset}",
    ]
    print("\n" + "\n".join(lines) + "\n", file=sys.stderr)


# ============================================================================
# DETECTION HELPER
# ============================================================================

def detect_target(model, frame, conf_threshold: float, class_names: dict, is_placeholder: bool) -> dict | None:
    """
    Run YOLO inference on a frame and return the best qualifying detection.

    Returns a dict with keys: class, conf, bbox (x1,y1,x2,y2), center (x,y),
    pixel_height — or None if nothing qualifies.
    """
    try:
        import torch
        with torch.inference_mode():
            results = model(frame, imgsz=224, conf=conf_threshold, verbose=False)
    except Exception:
        results = model(frame, imgsz=224, conf=conf_threshold, verbose=False)

    boxes = results[0].boxes if len(results) > 0 else None
    if boxes is None or len(boxes) == 0:
        return None

    best_conf = -1.0
    best_idx = -1

    for idx, box in enumerate(boxes):
        conf = float(box.conf[0].cpu().numpy())
        cls_id = int(box.cls[0].cpu().numpy())
        cls_name = class_names.get(cls_id, f"class_{cls_id}").lower().strip()

        qualifies = cls_name in WASTE_CLASSES
        if not qualifies and is_placeholder:
            qualifies = cls_name in {
                "bottle", "cup", "cell phone", "mouse", "book", "apple", "banana",
                "fork", "knife", "spoon",
            }

        # Ignore giant bounding boxes covering more than 60% of the frame
        b_xyxy = box.xyxy[0].cpu().numpy()
        if (b_xyxy[2] - b_xyxy[0]) >= 0.60 * IMGSZ and (b_xyxy[3] - b_xyxy[1]) >= 0.60 * IMGSZ:
            qualifies = False

        if qualifies and conf > best_conf:
            best_conf = conf
            best_idx = idx

    if best_idx < 0:
        return None

    best_box = boxes[best_idx]
    xyxy = best_box.xyxy[0].cpu().numpy()
    cls_id = int(best_box.cls[0].cpu().numpy())
    cls_name = class_names.get(cls_id, f"class_{cls_id}").lower().strip()
    x1, y1, x2, y2 = xyxy

    # Reclassify tall cylindrical containers mistakenly predicted as cell phone
    if cls_name == "cell phone" and ((y2 - y1) / max(1.0, (x2 - x1))) >= 2.1:
        cls_name = "bottle"

    return {
        "class": cls_name,
        "conf": best_conf,
        "bbox": (int(x1), int(y1), int(x2), int(y2)),
        "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
        "pixel_height": max(1.0, float(y2 - y1)),
    }


# ============================================================================
# CAMERA WRAPPER
# ============================================================================

class Camera:
    """Wraps Picamera2 or OpenCV VideoCapture with frame settling."""

    def __init__(self, source, imgsz: int, settle_s: float, stale_frames: int = 2):
        self.imgsz = imgsz
        self.settle_s = settle_s
        self.stale_frames = stale_frames
        self._picam = None
        self._cap = None

        if str(source) == "0":
            try:
                from picamera2 import Picamera2
                print("[CAMERA] Attempting native Picamera2 initialization...")
                self._picam = Picamera2()
                config = self._picam.create_preview_configuration(
                    main={"size": (imgsz, imgsz), "format": "BGR888"}
                )
                self._picam.configure(config)
                self._picam.start()
                print(f"[CAMERA] Picamera2 initialized at {imgsz}x{imgsz}.")
                return
            except Exception as e:
                print(f"[CAMERA] Picamera2 unavailable ({e}); falling back to OpenCV.")

        import cv2
        src = int(source) if str(source).isdigit() else source
        print(f"[CAMERA] Opening source via OpenCV: {src}...")
        self._cap = cv2.VideoCapture(src)
        if not self._cap.isOpened():
            raise RuntimeError(f"Could not open video source '{src}'.")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, imgsz)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, imgsz)
        print(f"[CAMERA] OpenCV camera opened.")

    def grab(self):
        """Wait for camera to settle, discard stale frames, return a fresh frame."""
        import numpy as np
        time.sleep(self.settle_s)
        frame = None

        for _ in range(self.stale_frames + 1):
            if self._picam:
                frame = self._picam.capture_array()
            elif self._cap:
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    raise RuntimeError("Camera frame grab failed.")

        return frame

    def release(self):
        if self._picam:
            try:
                self._picam.stop()
            except Exception:
                pass
        if self._cap:
            self._cap.release()


# ============================================================================
# CLI ARGUMENT PARSER
# ============================================================================

def parse_args():
    p = argparse.ArgumentParser(description="COG Autonomous Waste Robot")

    # Run modes
    p.add_argument("--preview", action="store_true",
                   help="Dev mode: show annotated OpenCV window; press q to stop.")
    p.add_argument("--teleop", action="store_true",
                   help="Dev mode: WASD teleoperation for calibration.")
    p.add_argument("--mock", action="store_true",
                   help="Simulated serial/LEDs — runs full state machine on a laptop.")

    # Model / camera
    p.add_argument("--model", default="yolov8n.pt", help="YOLO model path (yolov8n.pt recommended for fast 30+ FPS).")
    p.add_argument("--source", default="0", help="Camera index or video path.")
    p.add_argument("--imgsz", type=int, default=IMGSZ, help="Capture resolution.")
    p.add_argument("--conf", type=float, default=CONF_THRESHOLD, help="Detection confidence.")

    # Serial
    p.add_argument("--port", default=None, help="Arduino serial port (auto-detect if omitted).")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")

    # LEDs
    p.add_argument("--red-led", type=int, default=RED_LED_PIN, help="BCM pin for red LED.")
    p.add_argument("--green-led", type=int, default=GREEN_LED_PIN, help="BCM pin for green LED.")

    # Tunables
    p.add_argument("--turn-ms-per-deg", type=float, default=TURN_MS_PER_DEG)
    p.add_argument("--drive-ms-per-cm", type=float, default=DRIVE_MS_PER_CM)
    p.add_argument("--forward-burst-ms", type=int, default=FORWARD_BURST_MS)
    p.add_argument("--scan-step-deg", type=float, default=SCAN_STEP_DEG)
    p.add_argument("--arrive-cm", type=float, default=ARRIVE_CM)
    p.add_argument("--arrive-y-frac", type=float, default=ARRIVE_Y_FRAC)
    p.add_argument("--align-deadband-deg", type=float, default=ALIGN_DEADBAND_DEG)
    p.add_argument("--camera-settle-s", type=float, default=CAMERA_SETTLE_S)
    p.add_argument("--waste-conf", type=float, default=WASTE_CONF)
    p.add_argument("--sweep-limit-deg", type=float, default=SWEEP_LIMIT_FIRST_DEG,
                   help="Initial sweep limit in degrees (default 360).")
    p.add_argument("--reverse-ms", type=int, default=REVERSE_MS)
    p.add_argument("--turn-away-ms", type=int, default=TURN_AWAY_MS)
    p.add_argument("--led-hold-s", type=float, default=LED_HOLD_S)
    p.add_argument("--vote-frames", type=int, default=VOTE_FRAMES)
    p.add_argument("--start-delay-s", type=float, default=START_DELAY_S)
    p.add_argument("--max-run-s", type=float, default=MAX_RUN_S)
    p.add_argument("--fov", type=float, default=HORIZONTAL_FOV)

    # Shutdown
    p.add_argument("--shutdown-when-done", action="store_true", default=True,
                   help="Shut down the Pi after DONE (default: True).")
    p.add_argument("--no-shutdown", dest="shutdown_when_done", action="store_false",
                   help="Do NOT shut down after DONE.")

    return p.parse_args()


# ============================================================================
# TELEOP MODE
# ============================================================================

def run_teleop(args):
    """WASD teleoperation for floor calibration."""
    serial_iface = ArduinoSerialInterface(
        port=args.port, baudrate=args.baud, mock=args.mock
    )
    serial_iface.connect()

    duration = 300  # ms per keypress

    print("=" * 60)
    print("  TELEOP MODE — Calibration Teleoperation")
    print("=" * 60)
    print("  Keys: w=Forward  s=Backward  a=Left  d=Right  q=Quit")
    print(f"        +=Increase duration  -=Decrease duration  (current: {duration}ms)")
    print("        u=Ultrasonic query")
    print("=" * 60)

    try:
        import cv2
        # Open a tiny window just to capture keypresses
        dummy = None
        if args.preview:
            cam = Camera(args.source, args.imgsz, settle_s=0.05, stale_frames=0)
        while True:
            if args.preview:
                try:
                    frame = cam.grab()
                    cv2.imshow("Teleop", frame)
                except Exception:
                    pass
                key = cv2.waitKey(100) & 0xFF
            else:
                # Headless: use terminal input
                import msvcrt  # Windows
                if msvcrt.kbhit():
                    key = ord(msvcrt.getch())
                else:
                    time.sleep(0.05)
                    continue

            if key == ord("q"):
                break
            elif key == ord("w"):
                print(f"[TELEOP] F,{duration}")
                serial_iface.send_forward(duration)
            elif key == ord("s"):
                print(f"[TELEOP] B,{duration}")
                serial_iface.send_backward(duration)
            elif key == ord("a"):
                print(f"[TELEOP] TL,{duration}")
                serial_iface.send_turn_left(duration)
            elif key == ord("d"):
                print(f"[TELEOP] TR,{duration}")
                serial_iface.send_turn_right(duration)
            elif key == ord("+") or key == ord("="):
                duration = min(1500, duration + 50)
                print(f"[TELEOP] Duration increased to {duration}ms")
            elif key == ord("-"):
                duration = max(50, duration - 50)
                print(f"[TELEOP] Duration decreased to {duration}ms")
            elif key == ord("u"):
                d = serial_iface.query_distance()
                print(f"[TELEOP] Ultrasonic: {d:.1f} cm")

    except ImportError:
        # Fallback for Linux terminal input
        import tty
        import termios
        import select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            print("[TELEOP] Using terminal raw input. Press keys directly.")
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    key = ord(ch)
                    if key == ord("q"):
                        break
                    elif key == ord("w"):
                        print(f"[TELEOP] F,{duration}")
                        serial_iface.send_forward(duration)
                    elif key == ord("s"):
                        print(f"[TELEOP] B,{duration}")
                        serial_iface.send_backward(duration)
                    elif key == ord("a"):
                        print(f"[TELEOP] TL,{duration}")
                        serial_iface.send_turn_left(duration)
                    elif key == ord("d"):
                        print(f"[TELEOP] TR,{duration}")
                        serial_iface.send_turn_right(duration)
                    elif key == ord("+") or key == ord("="):
                        duration = min(1500, duration + 50)
                        print(f"[TELEOP] Duration: {duration}ms")
                    elif key == ord("-"):
                        duration = max(50, duration - 50)
                        print(f"[TELEOP] Duration: {duration}ms")
                    elif key == ord("u"):
                        d = serial_iface.query_distance()
                        print(f"[TELEOP] Ultrasonic: {d:.1f} cm")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except KeyboardInterrupt:
        pass
    finally:
        serial_iface.send_stop()
        serial_iface.close()
        print("[TELEOP] Exited.")


# ============================================================================
# MAIN AUTONOMOUS LOOP
# ============================================================================

def main():
    args = parse_args()

    # --- Teleop shortcut ---
    if args.teleop:
        run_teleop(args)
        return

    # --- LED Controller ---
    leds = LEDController(args.red_led, args.green_led, mock=args.mock)

    # --- Signal Handlers for clean exit ---
    shutdown_requested = False

    def _signal_handler(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)

    serial_iface = None
    camera = None
    exit_code = 0

    try:
        # =============================================================
        # STARTUP SEQUENCE (fully automatic, no human input)
        # =============================================================

        print("=" * 70)
        print("  COG Autonomous Waste Robot — Starting Up")
        print("=" * 70)

        # Step A: Find and connect to the Arduino
        print("\n[STARTUP 1/5] Connecting to Arduino...")
        serial_iface = ArduinoSerialInterface(
            port=args.port, baudrate=args.baud, mock=args.mock
        )
        try:
            serial_iface.connect(auto_find_timeout=30.0)
        except Exception as e:
            print(f"[FATAL] Arduino connection failed: {e}", file=sys.stderr)
            leds.blink("red", times=5, on_s=0.1, off_s=0.1)
            leds.cleanup()
            sys.exit(1)

        # Step B: Load the YOLO model
        print("\n[STARTUP 2/5] Loading YOLO model...")
        model_path, is_placeholder = resolve_model_path(args.model)

        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            class_names = model.names
            print(f"[STARTUP] Model loaded: {model_path} ({len(class_names)} classes)")
        except Exception as e:
            print(f"[FATAL] Failed to load model: {e}", file=sys.stderr)
            leds.blink("red", times=5, on_s=0.1, off_s=0.1)
            serial_iface.close()
            leds.cleanup()
            sys.exit(1)

        # Step B2: Optimize PyTorch CPU settings for Pi 4B
        try:
            import torch
            torch.set_num_threads(4)
        except Exception:
            pass

        # Step C: Open camera
        print("\n[STARTUP 3/5] Opening camera...")
        try:
            camera = Camera(
                args.source, args.imgsz,
                settle_s=args.camera_settle_s,
                stale_frames=STALE_FRAMES_DISCARD,
            )
        except Exception as e:
            print(f"[FATAL] Camera initialization failed: {e}", file=sys.stderr)
            leds.blink("red", times=5, on_s=0.1, off_s=0.1)
            serial_iface.close()
            leds.cleanup()
            sys.exit(1)

        # Step D: Ready blink
        print("\n[STARTUP 4/5] Ready blink...")
        leds.blink_both(times=2, on_s=0.4, off_s=0.3)

        # Step E: Start delay
        print(f"\n[STARTUP 5/5] Waiting {args.start_delay_s}s for people to step away...")
        delay_end = time.time() + args.start_delay_s
        while time.time() < delay_end:
            if shutdown_requested:
                print("[STARTUP] Shutdown requested during start delay.")
                serial_iface.send_stop()
                leds.cleanup()
                serial_iface.close()
                sys.exit(0)
            time.sleep(0.5)

        # =============================================================
        # STATE MACHINE
        # =============================================================
        print("\n" + "=" * 70)
        print("  Entering autonomous scan-drive-classify loop")
        print("=" * 70 + "\n")

        state = State.SCAN
        sweep_deg = 0.0
        first_scan = True
        sweep_limit = args.sweep_limit_deg  # 360 first time
        waste_count = 0
        not_waste_count = 0
        conf_window: list[float] = []
        consecutive_lost = 0
        led_color = "red"
        target_class = "unknown"
        start_time = time.time()
        last_two_dists: list[float] = []  # last two Q readings for emergency stop
        done_reason = ""

        while state != State.DONE:
            # --- Global timeout ---
            elapsed = time.time() - start_time
            if elapsed >= args.max_run_s:
                print(f"[SAFETY] MAX_RUN_S ({args.max_run_s}s) exceeded. Forcing DONE.")
                done_reason = "timeout"
                state = State.DONE
                continue

            # --- Signal check ---
            if shutdown_requested:
                print("[SIGNAL] Shutdown/interrupt requested. Forcing DONE.")
                done_reason = "signal"
                state = State.DONE
                continue

            # -------------------------------------------------------
            # SCAN: pivot, look for targets
            # -------------------------------------------------------
            if state == State.SCAN:
                frame = camera.grab()
                target = detect_target(model, frame, args.conf, class_names, is_placeholder)

                if args.preview:
                    _show_preview(frame, target, state, args)

                if target:
                    print(f"[SCAN] Target found: {target['class']} conf={target['conf']:.2f}")
                    state = State.ALIGN
                    sweep_deg = 0.0
                    consecutive_lost = 0
                    conf_window = []
                else:
                    turn_ms = bearing_to_turn_ms(args.scan_step_deg, args.turn_ms_per_deg)
                    ok = serial_iface.send_turn_left(turn_ms)
                    if not ok:
                        # Missing A — retry once
                        print("[SCAN] No ack for turn. Sending S and retrying...")
                        serial_iface.send_stop()
                        ok = serial_iface.send_turn_left(turn_ms)
                        if not ok:
                            print("[SCAN] Retry failed. Entering DONE with error.")
                            done_reason = "serial_error"
                            leds.blink("red", times=10, on_s=0.05, off_s=0.05)
                            state = State.DONE
                            continue

                    sweep_deg += args.scan_step_deg
                    print(f"[SCAN] No target. Swept {sweep_deg:.0f}° / {sweep_limit:.0f}°")
                    if sweep_deg >= sweep_limit:
                        print("[SCAN] Sweep limit reached with no target. DONE.")
                        done_reason = "no_targets"
                        state = State.DONE

            # -------------------------------------------------------
            # ALIGN: rotate to center the target in frame
            # -------------------------------------------------------
            elif state == State.ALIGN:
                frame = camera.grab()
                target = detect_target(model, frame, args.conf, class_names, is_placeholder)

                if args.preview:
                    _show_preview(frame, target, state, args)

                if not target:
                    consecutive_lost += 1
                    print(f"[ALIGN] Target lost ({consecutive_lost}/{MAX_CONSECUTIVE_LOST})")
                    if consecutive_lost >= MAX_CONSECUTIVE_LOST:
                        print("[ALIGN] Target lost too many times. Back to SCAN.")
                        state = State.SCAN
                    continue

                consecutive_lost = 0
                target_class = target["class"]
                h, w = frame.shape[:2]
                bearing, _ = calculate_bearing_and_angle(
                    target["center"][0], w, fov=args.fov
                )

                if abs(bearing) <= args.align_deadband_deg:
                    print(f"[ALIGN] Centered (bearing={bearing:+.1f}°). -> APPROACH")
                    state = State.APPROACH
                    conf_window = []
                elif bearing > 0:
                    ms = bearing_to_turn_ms(bearing, args.turn_ms_per_deg)
                    print(f"[ALIGN] Bearing {bearing:+.1f}° -> TR,{ms}")
                    ok = serial_iface.send_turn_right(ms)
                    if not ok:
                        serial_iface.send_stop()
                        ok = serial_iface.send_turn_right(ms)
                        if not ok:
                            done_reason = "serial_error"
                            leds.blink("red", times=10, on_s=0.05, off_s=0.05)
                            state = State.DONE
                else:
                    ms = bearing_to_turn_ms(bearing, args.turn_ms_per_deg)
                    print(f"[ALIGN] Bearing {bearing:+.1f}° -> TL,{ms}")
                    ok = serial_iface.send_turn_left(ms)
                    if not ok:
                        serial_iface.send_stop()
                        ok = serial_iface.send_turn_left(ms)
                        if not ok:
                            done_reason = "serial_error"
                            leds.blink("red", times=10, on_s=0.05, off_s=0.05)
                            state = State.DONE

            # -------------------------------------------------------
            # APPROACH: drive toward the target
            # -------------------------------------------------------
            elif state == State.APPROACH:
                # Safety: ultrasonic check BEFORE moving
                dist = serial_iface.query_distance()

                # Track last two readings for emergency stop
                last_two_dists.append(dist)
                if len(last_two_dists) > 2:
                    last_two_dists.pop(0)

                # Emergency stop: two consecutive readings <= EMERGENCY_STOP_CM
                if len(last_two_dists) >= 2:
                    if (last_two_dists[-1] != -1 and last_two_dists[-1] <= EMERGENCY_STOP_CM and
                            last_two_dists[-2] != -1 and last_two_dists[-2] <= EMERGENCY_STOP_CM):
                        print(f"[SAFETY] Two consecutive readings <= {EMERGENCY_STOP_CM}cm. Emergency stop.")
                        serial_iface.send_stop()
                        # Treat as arrived
                        state = State.VERDICT
                        continue

                frame = camera.grab()
                target = detect_target(model, frame, args.conf, class_names, is_placeholder)

                if args.preview:
                    _show_preview(frame, target, state, args)

                if target:
                    consecutive_lost = 0
                    target_class = target["class"]
                    h, w = frame.shape[:2]

                    bearing, _ = calculate_bearing_and_angle(
                        target["center"][0], w, fov=args.fov
                    )

                    # Record confidence
                    conf_window.append(target["conf"])
                    if len(conf_window) > args.vote_frames:
                        conf_window.pop(0)

                    # Re-align if drifted
                    if abs(bearing) > args.align_deadband_deg:
                        print(f"[APPROACH] Drifted (bearing={bearing:+.1f}°). -> ALIGN")
                        state = State.ALIGN
                        continue

                    # Check arrival: ultrasonic
                    if dist != -1 and dist <= args.arrive_cm:
                        print(f"[APPROACH] Arrived (ultrasonic: {dist:.1f}cm <= {args.arrive_cm}cm)")
                        state = State.VERDICT
                        continue

                    # Check arrival: bbox bottom edge
                    bbox_bottom = target["bbox"][3]
                    if bbox_bottom >= args.arrive_y_frac * h:
                        print(f"[APPROACH] Arrived (bbox bottom {bbox_bottom} >= {args.arrive_y_frac * h:.0f})")
                        state = State.VERDICT
                        continue

                    # Not arrived yet — drive forward
                    # But NEVER send F if the last two readings are dangerously close
                    if (len(last_two_dists) >= 2 and
                            last_two_dists[-1] != -1 and last_two_dists[-1] <= EMERGENCY_STOP_CM and
                            last_two_dists[-2] != -1 and last_two_dists[-2] <= EMERGENCY_STOP_CM):
                        print("[SAFETY] Refusing F — ultrasonic too close.")
                    else:
                        print(f"[APPROACH] F,{args.forward_burst_ms} (dist={dist:.1f}cm)")
                        ok = serial_iface.send_forward(args.forward_burst_ms)
                        if not ok:
                            serial_iface.send_stop()
                            ok = serial_iface.send_forward(args.forward_burst_ms)
                            if not ok:
                                done_reason = "serial_error"
                                leds.blink("red", times=10, on_s=0.05, off_s=0.05)
                                state = State.DONE
                                continue
                else:
                    conf_window.append(0.0)
                    if len(conf_window) > args.vote_frames:
                        conf_window.pop(0)
                    consecutive_lost += 1
                    print(f"[APPROACH] Target lost ({consecutive_lost}/{MAX_CONSECUTIVE_LOST})")
                    if consecutive_lost >= MAX_CONSECUTIVE_LOST:
                        print("[APPROACH] Target lost too many times. Back to SCAN.")
                        state = State.SCAN

            # -------------------------------------------------------
            # VERDICT: classify based on class type and mean confidence
            # -------------------------------------------------------
            elif state == State.VERDICT:
                mean_conf = sum(conf_window) / max(len(conf_window), 1)
                is_waste_item = (target_class.lower() in WASTE_CLASSES or
                                 target_class.lower() in {"bottle", "cup", "can", "wine glass", "bowl", "plastic bottle", "drink can"})

                if is_waste_item and mean_conf >= args.waste_conf:
                    led_color = "green"
                    waste_count += 1
                    verdict = "WASTE"
                else:
                    led_color = "red"
                    not_waste_count += 1
                    verdict = "NOT WASTE"

                print(f"[VERDICT] {target_class} (mean_conf={mean_conf:.2f}, waste_type={is_waste_item}) -> "
                      f"{verdict} -> {led_color.upper()} LED")
                state = State.INDICATE

            # -------------------------------------------------------
            # INDICATE: light LED
            # -------------------------------------------------------
            elif state == State.INDICATE:
                leds.on(led_color)
                print(f"[INDICATE] {led_color.upper()} LED ON for {args.led_hold_s}s")
                time.sleep(args.led_hold_s)
                leds.off()
                state = State.LEAVE

            # -------------------------------------------------------
            # LEAVE: reverse, turn away, reset sweep
            # -------------------------------------------------------
            elif state == State.LEAVE:
                print(f"[LEAVE] Reversing B,{args.reverse_ms} then turning TR,{args.turn_away_ms}")
                serial_iface.send_backward(args.reverse_ms)
                serial_iface.send_turn_right(args.turn_away_ms)
                sweep_deg = 0.0
                first_scan = False
                sweep_limit = SWEEP_LIMIT_AFTER_DEG
                consecutive_lost = 0
                last_two_dists = []
                state = State.SCAN

        # =============================================================
        # DONE: stop, blink, report, shutdown
        # =============================================================
        serial_iface.send_stop()

        total = waste_count + not_waste_count
        print("\n" + "=" * 70)
        print(f"  DONE — Handled {total} objects: {waste_count} waste, {not_waste_count} not waste")
        print(f"  Reason: {done_reason or 'complete'}")
        print("=" * 70)

        # Blink both LEDs 3x to signal completion
        leds.blink_both(times=3, on_s=0.5, off_s=0.3)

        # Alternate pattern — "safe to pick up and cut power"
        print("[DONE] Alternating LEDs — safe to pick up. Shutting down in ~10s...")
        leds.alternate_pattern(duration_s=10.0, interval_s=1.0)

        # Shutdown
        if args.shutdown_when_done and not args.mock:
            print("[DONE] Executing: sudo shutdown -h now")
            subprocess.run(["sudo", "shutdown", "-h", "now"], check=False)
        elif args.mock:
            print("[MOCK] Would execute: sudo shutdown -h now")
        else:
            print("[DONE] --no-shutdown specified. Robot idle.")

    except Exception as e:
        print(f"\n[FATAL EXCEPTION] {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        exit_code = 1

    finally:
        # --- Clean exit: stop motors, LEDs off, close ports ---
        if serial_iface:
            serial_iface.send_stop()
            serial_iface.close()
        if camera:
            camera.release()
        leds.cleanup()
        if args.preview:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass
        print("[SYSTEM] Robot shutdown complete.")
        sys.exit(exit_code)


# ============================================================================
# PREVIEW OVERLAY (dev mode only)
# ============================================================================

def _show_preview(frame, target, state: State, args):
    """Draw an annotated preview window. Only called when --preview is set."""
    try:
        import cv2
        display = frame.copy()
        h, w = display.shape[:2]
        cx, cy = w // 2, h // 2

        # Center crosshair
        cv2.line(display, (cx, 0), (cx, h), (90, 90, 90), 1)
        cv2.line(display, (0, cy), (w, cy), (90, 90, 90), 1)

        # Deadband corridor
        db_px = int((args.align_deadband_deg / args.fov) * w)
        cv2.line(display, (cx - db_px, 0), (cx - db_px, h), (40, 120, 40), 1)
        cv2.line(display, (cx + db_px, 0), (cx + db_px, h), (40, 120, 40), 1)

        # Arrival line
        arrive_y = int(args.arrive_y_frac * h)
        cv2.line(display, (0, arrive_y), (w, arrive_y), (0, 0, 200), 1)

        if target:
            x1, y1, x2, y2 = target["bbox"]
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{target['class']} {target['conf']:.2f}"
            cv2.putText(display, label, (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # State banner
        cv2.rectangle(display, (0, 0), (w, 26), (25, 25, 25), -1)
        cv2.putText(display, f"State: {state.value}", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)

        cv2.imshow("COG Robot — Preview", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("[PREVIEW] 'q' pressed — stopping robot.")
            raise KeyboardInterrupt("User pressed q in preview")
    except ImportError:
        pass


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    main()
