# COG — Autonomous Waste-Detection Robot

**DA3 Course Project Submission & Demonstration Documentation**  
*Assessment Due Date:* November 3, 2026 | 11:59 PM  
*Demonstration Status:* **Successfully Evaluated & Verified Live Before Course Faculty**

---

## Executive Summary

The **COG Autonomous Robot** is a fully autonomous 4-wheel-drive (4WD) robotic vehicle engineered to detect, navigate toward, and classify floor waste in indoor environments with **zero human intervention at runtime**. 

During the live faculty evaluation, the physical robot was placed in a test arena with target objects (including a paper cup and plastic water bottle). The robot autonomously executed a 360° sweeping scan, detected the target cup, performed closed-loop visual servoing to center the object in its optical axis, pulsed forward to the arrival threshold, classified the object as **WASTE**, illuminated the **Green Status LED** for 3.0 seconds, and executed an obstacle-clearing reverse and pivot maneuver to continue searching. The course faculty confirmed 100% operational success.

---

## 1. System Architecture

The robot employs a dual-tier edge computing architecture separating high-level perception from real-time motor actuation:

```
                         ┌─────────────────────┐
                         │   Raspberry Pi 4B   │
                         │  (Power Bank 5V/3A) │
                         ├──────────┬──────────┤
                         │ Pi Camera V2 (IMX219)│  ← YOLOv8 Edge Inference
                         │ GPIO 17 → RED LED   │  ← Non-Waste Indicator
                         │ GPIO 27 → GREEN LED │  ← Waste Indicator
                         └─────┬───────────────┘
                               │ USB Serial (115200 baud)
                               │ (Shared common ground)
                         ┌─────▼───────────────┐
                         │    Arduino Uno      │
                         ├─────────────────────┤
                         │ L298N Dual H-Bridge │  ← 4WD DC Geared Motors
                         │ HC-SR04 Ultrasonic  │  ← Distance Telemetry
                         └─────────────────────┘
```

### Finite State Machine Flow

The autonomous software pipeline is governed by a 7-state finite state machine (FSM):

```
 SCAN ──(target found)──► ALIGN ──(centered)──► APPROACH ──(arrived)──► VERDICT
  ▲                         │                       │                       │
  │                    (lost 3x)               (lost 3x)              (classify)
  │                         │                       │                       │
  └─────────────────────────┘                       │                       ▼
  ▲                                                 │                   INDICATE
  │                                                 │                       │
  │                                                 └───────────────────────┘
  │                                                                    (LED hold)
  │                                                                         │
  └─────────────── LEAVE ◄─────────────────────────────────────────────────┘
                     │
              (sweep exhausted)
                     │
                     ▼
                   DONE ──► blink ──► safe shutdown
```

---

## 2. Hardware Pinout & Wiring Specifications

> [!IMPORTANT]
> All pin mappings below are verified from the physical hardware setup.

### L298N Motor Driver → Arduino Uno
| L298N Pin | Arduino Pin | Function |
|:--|:--|:--|
| **ENA** | **5** (PWM) | Left Motors Speed Control (PWM 0–255) |
| **ENB** | **6** (PWM) | Right Motors Speed Control (PWM 0–255) |
| **IN1** | **7** | Left Motor Direction A |
| **IN2** | **8** | Left Motor Direction B |
| **IN3** | **9** | Right Motor Direction A |
| **IN4** | **11** | Right Motor Direction B |

#### Motor Direction Mapping:
| Motion | IN1 | IN2 | IN3 | IN4 |
|:--|:--|:--|:--|:--|
| **Forward** | LOW | HIGH | HIGH | LOW |
| **Backward** | HIGH | LOW | LOW | HIGH |
| **Pivot Left** | LOW | HIGH | LOW | HIGH |
| **Pivot Right** | HIGH | LOW | HIGH | LOW |

### HC-SR04 Ultrasonic Sensor → Arduino Uno
| HC-SR04 Pin | Arduino Pin | Description |
|:--|:--|:--|
| **TRIG** | **12** | 10µs ultrasonic pulse trigger |
| **ECHO** | **13** | Echo pulse width timing input |
| **VCC** | 5V | Regulated 5V rail |
| **GND** | GND | Common ground |

### Status Indicators → Raspberry Pi GPIO (BCM)
| Indicator | BCM Pin | Physical Pin | Function |
|:--|:--|:--|:--|
| **Red LED** | **17** | Pin 11 | NON-WASTE Detected |
| **Green LED** | **27** | Pin 13 | WASTE Detected |

*Power Setup:*
- **Raspberry Pi 4B**: Powered via dedicated 5V/3A USB-C power bank.
- **Arduino Uno**: Powered directly over USB serial from Raspberry Pi.
- **L298N & Motors**: Powered via 7.4V–12V external Li-ion battery pack with common ground connected to Arduino GND.

---

## 3. What to Install (Prerequisites & Dependencies)

### A. System Requirements
- **Python Version**: Python 3.9 through 3.12 (or 3.14 on host PC).
- **Arduino IDE**: 1.8.x or 2.x (for flashing `arduino/robot_control/robot_control.ino`).
- **Operating Systems Supported**: Linux (Raspberry Pi OS Bullseye/Bookworm), Windows 10/11, macOS.

### B. Installation Steps

#### 1. Clone the Repository
```bash
git clone https://github.com/VAMSI-KRISHNA-N/cog.git
cd cog
```

#### 2. Create and Activate Virtual Environment
```bash
# On Linux / Raspberry Pi:
python3 -m venv .venv
source .venv/bin/activate

# On Windows (PowerShell):
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

#### 3. Install Python Dependencies
```bash
pip install -r requirements.txt
```
*Core packages installed:*
- `ultralytics>=8.0.0` (YOLOv8 Edge Perception)
- `opencv-python>=4.8.0` (Computer vision & preview rendering)
- `pyserial>=3.5` (Serial communications protocol)
- `numpy>=1.23.0` (Geometric and matrix math)
- `python-docx>=1.0.0` (DA3 submission report generation)
- `matplotlib>=3.7.0` (Flowchart & visualization generation)
- `RPi.GPIO>=0.7.0` (Raspberry Pi hardware GPIO control on Linux)

---

## 4. How to Run

### Mode 1: Physical Autonomous Robot (On Raspberry Pi)

1. **Verify System Health:**
   ```bash
   python3 check_pi.py
   ```
   *Checks camera connection, serial port, GPIO access, and model weights.*

2. **Verify Motor Wiring:**
   ```bash
   python3 test_motors.py --port /dev/ttyACM0
   ```
   *Tests forward, backward, left, and right motion in sequence.*

3. **Launch Autonomous Waste Detection:**
   ```bash
   # Autonomous run with live preview window on monitor:
   python3 pi/robot.py --preview --no-shutdown

   # Full production headless run (auto-shuts down upon DONE):
   python3 pi/robot.py
   ```

### Mode 2: PC Simulation Mode (Interactive GUI & Mock Hardware)

Test the complete 7-state finite state machine without physical robot hardware:
```bash
# Run simulator with interactive PySide6 / OpenCV testbed:
python simulate.py

# Or run robot state machine in mock mode with sample test video:
python pi/robot.py --mock --preview --source tests/test_feed.mp4
```

### Mode 3: Automated Boot Service (systemd)

To run automatically when the Raspberry Pi boots:
```bash
# Copy and enable service
sudo cp systemd/cog-robot.service /etc/systemd/system/
sudo cp systemd/cog-shutdown.sudoers /etc/sudoers.d/cog-shutdown
sudo chmod 0440 /etc/sudoers.d/cog-shutdown
sudo visudo -c

sudo systemctl daemon-reload
sudo systemctl enable cog-robot.service

# View live system logs
journalctl -u cog-robot.service -f
tail -f ~/cog/logs/robot.log
```

---

## 5. Serial Communication Protocol

High-speed non-blocking newline-delimited serial protocol (115200 baud):

| Command | Direction | Parameters | Description | Response |
|:--|:--|:--|:--|:--|
| `F,<ms>` | Pi → Arduino | ms (1–1500) | Move forward for duration | `A` (Ack) |
| `B,<ms>` | Pi → Arduino | ms (1–1500) | Move backward for duration | `A` (Ack) |
| `TL,<ms>` | Pi → Arduino | ms (1–1500) | Pivot left for duration | `A` (Ack) |
| `TR,<ms>` | Pi → Arduino | ms (1–1500) | Pivot right for duration | `A` (Ack) |
| `S` | Pi → Arduino | none | Emergency stop | none |
| `V,<pwm>` | Pi → Arduino | 0–255 | Set motor PWM speed | none |
| `Q` | Pi → Arduino | none | Query ultrasonic distance | `R,<cm>` or `R,-1` |
| `READY` | Arduino → Pi | none | Microcontroller booted | — |

---

## 6. Waste Classification Rules

| Category | Indicator | Objects Detected |
|:--|:--|:--|
| **WASTE** | **GREEN LED** (BCM 27) | `bottle`, `cup`, `bowl`, `can`, `wine glass`, `apple`, `banana`, `orange`, `plastic`, `paper`, `cardboard`, `trash` |
| **NON-WASTE** | **RED LED** (BCM 17) | `cell phone`, `book`, `mouse`, `toothbrush`, `laptop` |
| **IGNORED** | *(Ignored by FSM)* | `person`, `chair`, `dining table`, `couch`, `bed`, `shoes`, `background` |

*Heuristic rules:* Handheld black cylindrical containers / flasks detected with aspect ratio $H/W \ge 2.1$ are mapped from `cell phone` to `bottle` (WASTE).

---

## 7. Submission Package Contents

As per the DA3 submission instructions:
1. **Codes**: Fully documented source code in `pi/`, `arduino/`, `simulate.py`, and `test_motors.py`.
2. **Sample Input and Output Files**: Located in `samples/`:
   - `sample_input_cup_scene.jpg`: Floor arena input image with paper cup.
   - `sample_input_bottle_scene.jpg`: Floor arena input image with plastic bottle.
   - `sample_test_feed.mp4`: Demonstration video input feed.
   - `sample_output_preview_approach.jpg`: Real-time HUD showing bounding box and `State: APPROACH`.
   - `sample_output_preview_align.jpg`: Real-time HUD showing `State: ALIGN`.
   - `live_demonstration_telemetry.txt`: Complete terminal log recorded during live faculty evaluation.
3. **Readme File**: This `README.md` file indicating what to install, how to run, and technical specifications.
4. **Word File (`DA3_Submission_Report.docx`)**:
   - High-resolution system flowchart.
   - Real-time working screen and camera HUD preview screenshots.
   - Live floor demonstration photos and robot hardware setup.
   - Output serial logs and state machine telemetry.
