# COG — Autonomous Waste-Detection Robot

Fully autonomous 4WD robot that scans for floor objects, drives to each one, classifies it as **WASTE** or **NOT WASTE**, lights the corresponding LED (green / red), and shuts down safely — all with **zero human input at runtime**.

---

## 1. System Architecture

```
                         ┌─────────────────────┐
                         │   Raspberry Pi 4B    │
                         │  (own power bank)    │
                         ├──────────┬───────────┤
                         │ Pi Camera V2         │  ← YOLOv8 Inference
                         │ GPIO 17 → RED LED    │
                         │ GPIO 27 → GREEN LED  │
                         └─────┬────────────────┘
                               │ USB Serial (115200 baud)
                               │ (also powers the Arduino)
                         ┌─────▼────────────────┐
                         │    Arduino Uno        │
                         ├──────────────────────┤
                         │ L298N Motor Driver    │  ← 4WD Chassis
                         │ HC-SR04 Ultrasonic    │
                         └──────────────────────┘
```

### State Machine Flow

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
                   DONE ──► blink ──► shutdown
```

---

## 2. Verified Wiring

> [!IMPORTANT]
> All pins below are **verified from working lab sketches**. No pin-probe step is needed.

### L298N Motor Driver → Arduino Uno

| L298N Pin | Arduino Pin | Purpose |
|:--|:--|:--|
| ENA | **5** (PWM) | Left motors speed |
| ENB | **6** (PWM) | Right motors speed |
| IN1 | **7** | Left motor direction A |
| IN2 | **8** | Left motor direction B |
| IN3 | **9** | Right motor direction A |
| IN4 | **11** | Right motor direction B |

**Direction mapping** (lab sketch names are swapped):

| Physical Motion | IN1 | IN2 | IN3 | IN4 |
|:--|:--|:--|:--|:--|
| **Forward** (toward camera) | LOW | HIGH | HIGH | LOW |
| **Backward** (away from camera) | HIGH | LOW | LOW | HIGH |
| **Pivot Left** | LOW | HIGH | LOW | HIGH |
| **Pivot Right** | HIGH | LOW | HIGH | LOW |

### HC-SR04 Ultrasonic → Arduino Uno

| HC-SR04 | Arduino Pin |
|:--|:--|
| TRIG | **12** |
| ECHO | **13** |
| VCC | 5V |
| GND | GND |

### LEDs → Raspberry Pi GPIO (BCM)

| LED | BCM Pin | Physical Pin | Wiring |
|:--|:--|:--|:--|
| **Red** | **17** | 11 | GPIO → Resistor → LED → GND |
| **Green** | **27** | 13 | GPIO → Resistor → LED → GND |

### Reserved Pins (IR sensors — DO NOT USE)

Pins **2**, **4**, **10** on the Arduino are reserved for IR line sensors. Do not use them.

### Power

- **Pi**: runs from its own USB power bank (5V / 3A).
- **Arduino**: powered over USB from the Pi. Grounds are shared through the USB cable.

---

## 3. Serial Protocol (115200 baud, newline-terminated)

| Direction | Command | Description | Reply |
|:--|:--|:--|:--|
| Pi → Arduino | `F,<ms>` | Forward for ms (clamp ≤ 1500) | `A` |
| Pi → Arduino | `B,<ms>` | Backward for ms | `A` |
| Pi → Arduino | `TL,<ms>` | Pivot left for ms | `A` |
| Pi → Arduino | `TR,<ms>` | Pivot right for ms | `A` |
| Pi → Arduino | `S` | Stop immediately | (none) |
| Pi → Arduino | `V,<0-255>` | Set PWM speed | (none) |
| Pi → Arduino | `Q` | Ultrasonic distance query | `R,<cm>` or `R,-1` |
| Arduino → Pi | `READY` | Boot handshake | — |

---

## 4. Run Modes

| Mode | Flag | Description |
|:--|:--|:--|
| **Headless** | *(default)* | No display, no keyboard. What the systemd service runs. |
| **Preview** | `--preview` | Shows annotated OpenCV window. Press `q` to stop. Dev only. |
| **Teleop** | `--teleop` | WASD manual driving for calibration. `+`/`-` adjust duration. |
| **Mock** | `--mock` | Simulated serial/LEDs. Full state machine on a laptop with webcam. |

### Quick Start

```bash
cd ~/cog
source .venv/bin/activate

# Mock mode on a laptop (no hardware needed):
python3 pi/robot.py --mock --preview

# Real hardware with preview:
python3 pi/robot.py --preview

# Headless (production):
python3 pi/robot.py
```

---

## 5. CLI Flags Reference

| Flag | Default | Description |
|:--|:--|:--|
| `--model` | `yolov8_trash.pt` | YOLO model path |
| `--source` | `0` | Camera index or video path |
| `--imgsz` | `320` | Capture resolution |
| `--conf` | `0.25` | Detection confidence threshold |
| `--port` | *(auto-detect)* | Arduino serial port |
| `--baud` | `115200` | Serial baud rate |
| `--mock` | `False` | Simulated hardware mode |
| `--preview` | `False` | Show annotated window |
| `--teleop` | `False` | Manual WASD driving mode |
| `--red-led` | `17` | BCM pin for red LED |
| `--green-led` | `27` | BCM pin for green LED |
| `--turn-ms-per-deg` | `8.0` | ms per degree of pivot |
| `--drive-ms-per-cm` | `15.0` | ms per cm of forward drive |
| `--forward-burst-ms` | `300` | ms per forward burst |
| `--scan-step-deg` | `20.0` | degrees per scan step |
| `--arrive-cm` | `18.0` | ultrasonic arrival threshold |
| `--arrive-y-frac` | `0.92` | bbox bottom arrival threshold |
| `--align-deadband-deg` | `6.0` | alignment tolerance |
| `--camera-settle-s` | `0.3` | frame settle delay |
| `--waste-conf` | `0.50` | mean confidence threshold for WASTE |
| `--sweep-limit-deg` | `360` | initial sweep limit |
| `--reverse-ms` | `500` | reverse duration after indicate |
| `--turn-away-ms` | `960` | turn-away after indicate (~120°) |
| `--led-hold-s` | `3.0` | LED on duration |
| `--vote-frames` | `5` | rolling confidence window size |
| `--start-delay-s` | `10.0` | delay after ready blink |
| `--max-run-s` | `300` | global time limit |
| `--shutdown-when-done` | `True` | shut down Pi after DONE |
| `--no-shutdown` | — | do NOT shut down after DONE |
| `--fov` | `62.2` | camera horizontal FOV |

---

## 6. Auto-Start on Boot (systemd)

### Install

```bash
# Create log directory
mkdir -p ~/cog/logs

# Copy service file
sudo cp systemd/cog-robot.service /etc/systemd/system/

# Install sudoers for passwordless shutdown
sudo cp systemd/cog-shutdown.sudoers /etc/sudoers.d/cog-shutdown
sudo chmod 0440 /etc/sudoers.d/cog-shutdown
sudo visudo -c  # Validate

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable cog-robot.service
```

### Disable for Development

```bash
sudo systemctl disable cog-robot.service
sudo systemctl stop cog-robot.service
```

### Read Logs

```bash
# Journal
journalctl -u cog-robot.service -f

# Log file
tail -f ~/cog/logs/robot.log
```

---

## 7. Model Download

```bash
# Automatic download:
python3 download_trash_model.py --api-key YOUR_ROBOFLOW_API_KEY

# Check if model is valid:
python3 download_trash_model.py --check
```

> [!WARNING]
> If `yolov8_trash.pt` is missing, the robot falls back to `yolov8n.pt` (generic COCO weights) with a loud warning. COCO weights cannot detect waste categories properly.

---

## 8. Tests

```bash
cd ~/cog
source .venv/bin/activate

# Unit tests (geometry, serial, LEDs):
python3 tests/test_components.py

# Integration tests (state machine):
python3 tests/test_integration.py
```

---

## 9. Safe Shutdown

After the robot reaches DONE, it:
1. Blinks both LEDs 3 times.
2. Alternates green/red for ~10 seconds ("safe to pick up").
3. Runs `sudo shutdown -h now`.

**Wait for the Pi's green ACT LED to stop blinking before cutting power.**
