# Complete End-to-End Execution Guide: Autonomous Waste Robot

From cold hardware to fully autonomous operation: wiring diagrams, exact terminal commands, calibration, test verification, and auto-start deployment.

---

## 1. Complete Physical Wiring & Connections

Make all wiring connections while **ALL POWER IS SWITCHED OFF**.

### 1.1 Raspberry Pi 4B Connections

```
 Raspberry Pi 4B GPIO Header (Pin 1 Top-Left, Pin 40 Bottom-Right)
 +---------------------------------------------------------------+
 |  3V3 (1)  (2) 5V                                              |
 |  GPIO2(3)  (4) 5V                                             |
 |  GPIO3(5)  (6) GND                                            |
 |  GPIO4(7)  (8) GPIO14                                         |
 |    GND(9) (10) GPIO15                                         |
 | GPIO17(11) (12) GPIO18   --> [Pin 11 / BCM 17] -> 220Ω -> RED LED (+)
 | GPIO27(13) (14) GND      --> [Pin 13 / BCM 27] -> 220Ω -> GREEN LED (+)
 | ...                                   [Pin 9 or 14] -> LEDs Cathode (-)
 +---------------------------------------------------------------+
```

| Component | Pi 4B Pin | Physical Pin # | Note |
|:---|:---|:---|:---|
| **Red LED (NOT WASTE)** | **BCM 17** | **Pin 11** | Connect via 220Ω–330Ω resistor to LED Anode (+) |
| **Green LED (WASTE)** | **BCM 27** | **Pin 13** | Connect via 220Ω–330Ω resistor to LED Anode (+) |
| **LED Cathodes (-)** | **GND** | **Pin 9 or 14** | Connect short legs of both LEDs to Pi GND |
| **Pi Camera V2** | **CSI Port** | — | Between HDMI & Audio jack; blue tape faces USB ports |
| **Arduino Uno** | **USB-A Port** | — | Connect standard USB-A to USB-B cable to Arduino |
| **Pi Power** | **USB-C Port** | — | Connect dedicated 5V / 3A power bank |

> [!IMPORTANT]
> The USB cable between the Pi and Arduino provides both the serial data bus (`/dev/ttyACM0`) and power to the Arduino Uno, establishing an automatic common ground.

---

### 1.2 Arduino Uno → L298N Dual H-Bridge Motor Driver

| L298N Driver Terminal | Arduino Uno Pin | Purpose |
|:---|:---|:---|
| **ENA** | **Pin 5 (PWM)** | Left Motors Speed (PWM 0–255) |
| **ENB** | **Pin 6 (PWM)** | Right Motors Speed (PWM 0–255) |
| **IN1** | **Pin 7** | Left Motors Direction 1 |
| **IN2** | **Pin 8** | Left Motors Direction 2 |
| **IN3** | **Pin 9** | Right Motors Direction 1 |
| **IN4** | **Pin 11** | Right Motors Direction 2 |
| **12V / VCC** | **Battery (+) 6V–12V** | Motor power (external battery pack) |
| **GND** | **Arduino GND + Battery (-)** | **CRITICAL: Shared common ground** |

> [!WARNING]
> Keep the L298N 5V regulator jumper in place if battery is under 12V. Remove pins 2, 4, and 10 from motor control — they are reserved for IR line sensors.

---

### 1.3 Arduino Uno → HC-SR04 Ultrasonic Distance Sensor

| HC-SR04 Pin | Arduino Uno Pin | Note |
|:---|:---|:---|
| **VCC** | **5V** | Ultrasonic 5V rail |
| **TRIG** | **Pin 12** | Trigger pulse output |
| **ECHO** | **Pin 13** | Echo receive input |
| **GND** | **GND** | Arduino GND rail |

---

## 2. Arduino Firmware Flashing

Flash the firmware onto the Arduino Uno from your computer or Raspberry Pi:

1. Connect Arduino Uno via USB.
2. Open the Arduino IDE.
3. Open the sketch: [`arduino/robot_control/robot_control.ino`](file:///c:/Users/VAMSI%20KRISHNA%20N/Desktop/cog/arduino/robot_control/robot_control.ino)
4. Select **Board**: `Arduino Uno`, **Port**: (e.g., `/dev/ttyACM0` or `COMx`).
5. Click **Upload** ($\rightarrow$).
6. Test in Serial Monitor at **115200 baud**:
   - Arduino will print: `READY`
   - Type `Q` $\to$ Arduino replies `R,<cm>`
   - Type `S` $\to$ Arduino stops motors
7. **IMPORTANT**: Close the Arduino Serial Monitor window so the port is free for Python.

---

## 3. Raspberry Pi Environment & Project Setup

Execute these commands in the terminal on the Raspberry Pi:

### 3.1 Serial Port Permissions & System Architecture

```bash
# Verify 64-bit OS
uname -m
# Expected output: aarch64

# Grant non-root access to Arduino serial port
sudo usermod -a -G dialout $USER
```
*(If you just added the group, log out and back in or run `newgrp dialout`)*

### 3.2 Clone Repo & Setup Python Virtual Environment

```bash
# Clone the repository
git clone https://github.com/VAMSI-KRISHNA-N/cog.git ~/cog
cd ~/cog

# Create virtual environment with system site-packages access for hardware libraries
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# Upgrade pip & install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 3.3 Test Camera & Hardware Dependencies

```bash
# Verify camera detection
rpicam-hello --list-cameras
# Expected: imx219 [3280x2464 10-bit] (Pi Camera V2)

# Quick Python dependency smoke test
python3 -c "import cv2, serial, ultralytics, numpy; print('✓ Python libraries loaded successfully!')"
```

### 3.4 Download Waste Detection Model

```bash
# Download the Roboflow 10-class waste detection model
python3 download_trash_model.py --api-key YOUR_ROBOFLOW_API_KEY

# Verify checkpoint integrity
python3 download_trash_model.py --check
```

---

## 4. Run Unit & Integration Tests

Ensure software integrity before putting the robot on the floor:

```bash
cd ~/cog
source .venv/bin/activate

# 1. Run Component Tests (Bearing math, serial protocol, mock LEDs)
python3 tests/test_components.py

# 2. Run Integration Tests (Full 15-case state machine verification)
python3 tests/test_integration.py
```
Both test suites will report:
`ALL UNIT TESTS PASSED!` and `ALL INTEGRATION TESTS PASSED!`

---

## 5. Bench Testing & Calibration (Wheels Off Ground)

> [!CAUTION]
> Place the car on a pedestal or box so wheels spin freely in the air!

### 5.1 Test Direction & Preview Mode

```bash
# Run with display preview and disable power-off during testing
python3 pi/robot.py --preview --no-shutdown
```

**Check wheel rotation:**
- When command `F` is issued: wheels must drive forward toward the camera/ultrasonic side.
- When turning left (`TL`): right wheels turn forward, left wheels turn backward.
- When turning right (`TR`): left wheels turn forward, right wheels turn backward.

### 5.2 Calibrate Floor Turn & Drive Factors (Teleop Mode)

Floor friction changes how fast the robot pivots and drives. Calibrate using the interactive teleop tool:

```bash
python3 pi/robot.py --teleop
```

| Key | Action |
|:---|:---|
| `w` | Drive forward by duration burst |
| `s` | Drive backward by duration burst |
| `a` | Pivot Left by duration burst |
| `d` | Pivot Right by duration burst |
| `+` / `-` | Increase / Decrease burst duration by 50 ms |
| `u` | Query ultrasonic sensor (prints distance in cm) |
| `q` | Quit teleop mode |

**Calibration Calculations:**
1. **Turn calibration (`--turn-ms-per-deg`)**:
   - Place robot on floor.
   - Use `a` or `d` to rotate the robot $90^\circ$. Record total milliseconds.
   - Calculate: $\text{TURN\_MS\_PER\_DEG} = \frac{\text{ms taken}}{90^\circ}$ (Default: `8.0`).
2. **Drive calibration (`--forward-burst-ms`)**:
   - Use `w` to drive forward. Tune duration so each burst moves ~5–8 cm (Default: `300`).

---

## 6. Configure Systemd Auto-Start & Sudoers

Enable the robot to boot and run completely autonomously with no keyboard or monitor.

### 6.1 Create Log Directory & Service

```bash
# Ensure log folder exists
mkdir -p ~/cog/logs

# Install systemd service unit
sudo cp systemd/cog-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cog-robot.service
```

### 6.2 Passwordless Shutdown Permission

Allow the robot process to safely turn off the Pi when finished:

```bash
# Install sudoers drop-in
sudo cp systemd/cog-shutdown.sudoers /etc/sudoers.d/cog-shutdown
sudo chmod 0440 /etc/sudoers.d/cog-shutdown

# Validate syntax
sudo visudo -c
# Expected: /etc/sudoers.d/cog-shutdown: parsed OK
```

---

## 7. Autonomous Operation & Demo Day Workflow

### 7.1 Placing and Running

1. **Place the robot on the floor** in the testing arena.
2. **Scatter small floor objects** (e.g. plastic bottles, cans, paper cups).
3. **Power on the Pi** (plug in the USB-C power bank cable).
4. **Autonomous Sequence Starts**:
   - **Boot**: The systemd service starts `pi/robot.py` in headless mode.
   - **Startup (Step 1–5)**:
     1. Connects to Arduino (`/dev/ttyACM0` or `/dev/ttyUSB0`) $\to$ waits for `READY`.
     2. Loads YOLO model (`yolov8_trash.pt`).
     3. Initializes Pi Camera.
     4. **Blinks both LEDs twice** (ready confirmation).
     5. **Waits 10 seconds** (giving operators time to step back).
   - **SCAN**: Car pivots in $20^\circ$ steps until an object enters frame.
   - **ALIGN**: Car pivots to center the object in camera view.
   - **APPROACH**: Car drives forward in bursts; stops when ultrasonic $\le 18\text{ cm}$ or bounding box bottom reaches 92% of frame height.
   - **VERDICT**: Computes average detection confidence across window:
     - Mean confidence $\ge 0.50 \to$ **WASTE** $\to$ **Green LED turns ON**.
     - Mean confidence $< 0.50 \to$ **NOT WASTE** $\to$ **Red LED turns ON**.
   - **INDICATE**: Holds LED for 3 seconds, then turns off.
   - **LEAVE**: Reverses $500\text{ ms}$, turns away $120^\circ$ to clear the current object.
   - **CYCLE**: Repeats scan for the next object.
   - **DONE**: When a $180^\circ$ sweep finds no more objects:
     1. Motors immediately stop (`S`).
     2. Both LEDs blink 3 times.
     3. Alternates Green and Red LEDs for 10 seconds (**"Safe to pick up"**).
     4. Pi executes `sudo shutdown -h now`.
5. **Cut Power**: Wait until the Pi 4B green ACT activity LED stops flashing completely, then disconnect the battery pack.

---

## 8. Log Inspection & Troubleshooting

### Check Live Status or Previous Run Logs

```bash
# View live output from the systemd service
journalctl -u cog-robot.service -f

# Check continuous robot log file
tail -f ~/cog/logs/robot.log

# View full log from last run
cat ~/cog/logs/robot.log
```

### Stop or Disable Auto-Start for Manual Debugging

```bash
# Temporarily stop the service
sudo systemctl stop cog-robot.service

# Disable auto-start on boot
sudo systemctl disable cog-robot.service

# Run manually with launcher script
bash run.sh --preview --no-shutdown
```

### Troubleshooting Cheat Sheet

| Issue | Root Cause | Solution |
|:---|:---|:---|
| **Red LED blinks 5x fast on boot** | Arduino connection failed, camera missing, or model file not found | Check `cat ~/cog/logs/robot.log` for exact error. Ensure USB cable is plugged in and camera ribbon is seated. |
| **Wheels pivot opposite direction** | Motor wires reversed on L298N terminals | Swap OUT1/OUT2 or OUT3/OUT4 wires, or verify pin numbers in `robot_control.ino`. |
| **Robot doesn't stop in front of object** | Ultrasonic sensor disconnected or Echo/Trig swapped | Check Pins 12 (Trig) and 13 (Echo). Test reading with `python3 pi/robot.py --teleop` (press `u`). |
| **Object ignored during scan** | Object class not in model classes or confidence $< 0.35$ | Run `python3 pi/robot.py --preview --mock` to inspect model bounding boxes. |
| **Serial permission denied** | User not in `dialout` group or Serial Monitor open | Run `sudo usermod -a -G dialout $USER` and reboot. Ensure Arduino IDE Serial Monitor is closed. |
