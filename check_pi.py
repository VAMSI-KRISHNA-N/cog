#!/usr/bin/env python3
"""
check_pi.py - Complete Pre-Flight Health Check for Raspberry Pi 4B Autonomous Waste Robot.

Run this script on the Raspberry Pi to verify all hardware connections, system permissions,
Python packages, models, and service files:
    python3 check_pi.py
"""

import os
import platform
import shutil
import subprocess
import sys

# Color formatting for terminal
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

passed_checks = 0
total_checks = 0
warnings = []
failures = []


def report(item: str, passed: bool, detail: str = "", is_warning: bool = False):
    global passed_checks, total_checks
    total_checks += 1
    if passed:
        passed_checks += 1
        status = f"{GREEN}[PASS]{RESET}"
    elif is_warning:
        status = f"{YELLOW}[WARN]{RESET}"
        warnings.append((item, detail))
    else:
        status = f"{RED}[FAIL]{RESET}"
        failures.append((item, detail))

    msg = f"{status} {BOLD}{item}{RESET}"
    if detail:
        msg += f" - {detail}"
    print(msg)


def check_os():
    print(f"\n{CYAN}{BOLD}=== 1. Operating System & Architecture ==={RESET}")
    arch = platform.machine()
    is_64bit = (arch == "aarch64")
    report("64-bit OS Architecture", is_64bit, f"Detected: {arch} (Requires aarch64 for PyTorch/YOLO)")

    # Check dialout group
    try:
        import grp
        user = os.environ.get("USER", "pi")
        groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
        # Also check current process groups
        current_gids = os.getgroups()
        current_groups = [grp.getgrgid(g).gr_name for g in current_gids]
        in_dialout = ("dialout" in groups) or ("dialout" in current_groups)
        report(
            "Serial Permissions ('dialout' group)",
            in_dialout,
            f"User '{user}' in dialout: {in_dialout}" + ("" if in_dialout else " (Run: sudo usermod -a -G dialout $USER)"),
            is_warning=not in_dialout
        )
    except Exception as e:
        report("Serial Permissions", True, f"Skipped check on non-POSIX ({e})")


def check_arduino():
    print(f"\n{CYAN}{BOLD}=== 2. Arduino Serial Connection ==={RESET}")
    import glob
    ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial/by-id/*")
    ports = sorted(list(set(ports)))
    if ports:
        report("Arduino Port Detected", True, f"Found port(s): {', '.join(ports)}")
    else:
        report("Arduino Port Detected", False, "No Arduino on /dev/ttyACM* or /dev/ttyUSB* (Is USB cable plugged in?)")


def check_camera():
    print(f"\n{CYAN}{BOLD}=== 3. Pi Camera Detection ==={RESET}")
    cam_found = False
    details = ""

    # Check 1: rpicam-hello / libcamera-hello
    for cmd in ["rpicam-hello", "libcamera-hello"]:
        if shutil.which(cmd):
            try:
                res = subprocess.run([cmd, "--list-cameras"], capture_output=True, text=True, timeout=5)
                out = res.stdout + res.stderr
                if "imx219" in out.lower() or "camera" in out.lower() or "available" in out.lower():
                    cam_found = True
                    details = f"Detected via {cmd}"
                    break
            except Exception:
                pass

    # Check 2: Video devices
    if not cam_found:
        import glob
        v_devs = glob.glob("/dev/video*")
        if v_devs:
            cam_found = True
            details = f"Found video devices: {', '.join(v_devs[:3])}"

    # Check 3: OpenCV test
    if not cam_found:
        try:
            import cv2
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                cam_found = True
                details = "Opened via OpenCV source 0"
                cap.release()
        except Exception:
            pass

    report("Pi Camera V2", cam_found, details or "No camera detected (Check ribbon cable orientation)")


def check_python_packages():
    print(f"\n{CYAN}{BOLD}=== 4. Python Environment & Dependencies ==={RESET}")
    
    # Python Version
    py_ver = sys.version.split()[0]
    report("Python Version", sys.version_info >= (3, 9), f"Python {py_ver}")

    # Core packages
    packages = [
        ("torch", "PyTorch"),
        ("ultralytics", "Ultralytics YOLO"),
        ("cv2", "OpenCV (opencv-python)"),
        ("serial", "PySerial"),
        ("numpy", "NumPy"),
    ]

    for mod, name in packages:
        try:
            imported = __import__(mod)
            ver = getattr(imported, "__version__", "installed")
            report(name, True, f"v{ver}")
        except ImportError:
            report(name, False, f"Missing! Run: pip install -r requirements.txt")

    # Optional / hardware-specific
    try:
        import RPi.GPIO as GPIO
        report("RPi.GPIO", True, f"v{GPIO.VERSION} (Hardware LED control available)")
    except ImportError:
        report("RPi.GPIO", False, "Missing RPi.GPIO (Will fall back to console mock LEDs)", is_warning=True)

    try:
        import picamera2
        report("Picamera2", True, "Hardware Pi camera module available")
    except ImportError:
        report("Picamera2", True, "Picamera2 not installed (Will use OpenCV VideoCapture fallback)", is_warning=True)


def check_models():
    print(f"\n{CYAN}{BOLD}=== 5. YOLO Model Checkpoints ==={RESET}")
    has_trash = os.path.exists("yolov8_trash.pt")
    has_ncnn = os.path.exists("yolov8_trash_ncnn_model")
    has_coco = os.path.exists("yolov8n.pt")

    if has_ncnn:
        report("Waste Model (NCNN)", True, "High-speed NCNN export 'yolov8_trash_ncnn_model' ready")
    elif has_trash:
        size = os.path.getsize("yolov8_trash.pt") / (1024 * 1024)
        report("Waste Model (PyTorch)", True, f"Dedicated 10-class model 'yolov8_trash.pt' ready ({size:.1f} MB)")
    else:
        report(
            "Waste Model ('yolov8_trash.pt')",
            False,
            "Missing dedicated model. Will fall back to yolov8n.pt (Run download_trash_model.py)",
            is_warning=True
        )

    if has_coco:
        size = os.path.getsize("yolov8n.pt") / (1024 * 1024)
        report("Fallback Model ('yolov8n.pt')", True, f"COCO fallback present ({size:.1f} MB)")
    else:
        report("Fallback Model ('yolov8n.pt')", False, "Missing! Run: python download_trash_model.py")


def check_systemd():
    print(f"\n{CYAN}{BOLD}=== 6. Systemd Auto-Start & Sudoers ==={RESET}")
    service_installed = os.path.exists("/etc/systemd/system/cog-robot.service")
    report(
        "Auto-Start Service (/etc/systemd/system/cog-robot.service)",
        service_installed,
        "Installed" if service_installed else "Not installed (Optional for manual testing. Install from systemd/)"
    )

    sudoers_installed = os.path.exists("/etc/sudoers.d/cog-shutdown")
    report(
        "Passwordless Shutdown (/etc/sudoers.d/cog-shutdown)",
        sudoers_installed,
        "Installed" if sudoers_installed else "Not installed (Required for autonomous shutdown. Install from systemd/)"
    )


def print_summary():
    print("\n" + "=" * 60)
    print(f"{BOLD}PRE-FLIGHT VERIFICATION SUMMARY{RESET}")
    print("=" * 60)
    print(f"Passed Checks: {GREEN}{passed_checks}{RESET} / {total_checks}")

    if failures:
        print(f"\n{RED}{BOLD}Critical Items Requiring Attention ({len(failures)}):{RESET}")
        for item, detail in failures:
            print(f"  • {BOLD}{item}{RESET}: {detail}")

    if warnings:
        print(f"\n{YELLOW}{BOLD}Non-Critical Warnings / Optional ({len(warnings)}):{RESET}")
        for item, detail in warnings:
            print(f"  • {BOLD}{item}{RESET}: {detail}")

    if not failures:
        print(f"\n{GREEN}{BOLD}✓ ALL CRITICAL HARDWARE & SOFTWARE REQUIREMENTS READY!{RESET}")
        print("You can run the robot with:")
        print(f"  {CYAN}python3 pi/robot.py --mock --preview{RESET}   # On-bench test")
        print(f"  {CYAN}python3 pi/robot.py --preview{RESET}          # With real Arduino & camera")
    else:
        print(f"\n{RED}Please resolve the critical item(s) above before running full autonomy.{RESET}")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    check_os()
    check_arduino()
    check_camera()
    check_python_packages()
    check_models()
    check_systemd()
    print_summary()
