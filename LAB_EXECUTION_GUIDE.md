# Master Lab Execution Guide: Phase 1 Bench Test

This guide covers the complete, step-by-step procedure to set up, flash, wire, run, debug, and calibrate the stationary waste-detection pointing system from a cold start to Phase 2 readiness.

---

## Stage 1: Physical Setup & Wiring (Bench Mode)

### 1.1 Raspberry Pi 4B Connections (Power OFF)
1. **MicroSD Card**: Insert the flashed 64GB card into the slot under the Pi.
2. **Pi Camera V2**:
   - Locate the **CAMERA** CSI port (between HDMI and audio jack, NOT the display DSI port).
   - Pull the plastic collar tabs gently upward.
   - Insert ribbon cable: **Blue tape faces the USB ports**, silver contacts face the micro-HDMI ports.
   - Push collar tabs down firmly to lock the ribbon in place.
3. **Arduino Uno Connection**: Connect a USB-A to USB-B cable from any USB port on the Pi to the Arduino Uno.
4. **Peripherals**: Connect micro-HDMI to your monitor, keyboard & mouse to USB.
5. **Power**: Plug the official USB-C power supply (5V / 3A) into the Pi.

---

### 1.2 Arduino & Sensor Wiring Pinout

> [!IMPORTANT]
> **UNCONFIRMED DEFAULT PINS**: The pins below (Servo=11, Trig=9, Echo=10) are placeholder defaults based on common bench wiring.
> **DO NOT assume they are correct for your kit's shield.** You **MUST** run **Stage 5 (`pin_probe.ino`)** first to confirm physical connections before flashing production firmware.

| Component | Wire / Pin | Arduino Uno Pin (Default) | Status | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **SG90 Servo** | Orange / Yellow | **Pin 11** *(PWM)* | *Unconfirmed default* | Servo PWM control signal |
| | Red | **5V** | Confirmed | Power |
| | Brown / Black | **GND** | Confirmed | Common ground |
| **HC-SR04** | VCC | **5V** | Confirmed | Power |
| | Trig | **Pin 9** | *Unconfirmed default* | 10µs ultrasonic trigger pulse |
| | Echo | **Pin 10** | *Unconfirmed default* | Echo pulse round-trip timing |
| | GND | **GND** | Confirmed | Common ground |

> [!TIP]
> **Power Stability**: If the servo jitters or the Arduino resets upon moving, the SG90 is pulling too much current from the Arduino's 5V rail. Connect the servo's red wire to an external 5V power source (sharing a common GND with the Arduino) or place a $100\mu\text{F}\text{--}470\mu\text{F}$ capacitor across 5V and GND.

---

## Stage 2: First Boot & OS Verification

1. Power on the Pi and log in to the desktop.
2. Open a terminal (`Ctrl + Alt + T`).
3. **Verify 64-bit Architecture**:
   ```bash
   uname -m
   ```
   - **Must output**: `aarch64`.
   - *(If it says `armv7l`, stop immediately: re-flash the card with Raspberry Pi OS 64-bit)*.
4. **Confirm Internet Access**:
   ```bash
   ping -c 3 8.8.8.8
   ```
5. **Grant Serial Port Permissions to User**:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   *(This ensures Python can access `/dev/ttyACM0` without `sudo`)*.

---

## Stage 3: Project Setup on the Pi

### 3.1 Transfer Code to Pi
- **Via Git**: Clone your repository to `~/cog`:
  ```bash
  git clone https://github.com/VAMSI-KRISHNA-N/cog.git ~/cog
  cd ~/cog
  ```
- **Or via USB Drive**: Copy the `cog/` folder to `/home/pi/cog`.

### 3.2 Create Virtual Environment & Install Dependencies
```bash
cd ~/cog

# 1. Create isolated environment
python3 -m venv .venv
source .venv/bin/activate

# 2. Upgrade pip and install requirements
pip install --upgrade pip
pip install -r requirements.txt
```
*(Installation on the Pi 4B typically takes 4–8 minutes)*.

### 3.3 Smoke Test Dependencies
```bash
python3 -c "import cv2, serial, ultralytics; print('Dependencies OK!')"
```
If this prints `Dependencies OK!`, proceed. If it fails, resolve the missing library before continuing.

### 3.4 Download & Verify the 10-Class Waste Model

The perception pipeline expects `yolov8_trash.pt` (trained on Roboflow Universe `yolov8-trash-detections`).

**Option A: Automated Download (with free API Key)**:
1. Create a free account at [https://app.roboflow.com/](https://app.roboflow.com/).
2. Copy your Private API Key from **Settings -> Roboflow API**.
3. Run:
   ```bash
   python3 download_trash_model.py --api-key YOUR_KEY
   ```

**Option B: Manual Download (No CLI Key Needed)**:
1. Visit [Roboflow Universe yolov8-trash-detections](https://universe.roboflow.com/waste-detection-spldq/yolov8-trash-detections).
2. Download weights as **YOLOv8 PyTorch format** (`best.pt`).
3. Copy/rename the file to `~/cog/yolov8_trash.pt`.
4. Verify the model file:
   ```bash
   python3 download_trash_model.py --check
   ```

> [!WARNING]
> **Placeholder Fallback Warning**: If `yolov8_trash.pt` is missing, `perception.py` falls back to `yolov8n.pt` (generic COCO weights). Standard COCO weights **cannot detect waste categories** and will misclassify items (e.g. computer mouse as apple, ceiling fan as airplane, bottle as vase). An unmissable warning will be printed in the console and shown on the preview window.

---

## Stage 4: Camera Verification

Before running YOLO, verify that the Pi Camera V2 is recognized:

1. **Check OS Camera Detection**:
   ```bash
   rpicam-hello --list-cameras
   ```
   *Should list: `imx219 [3280x2464 10-bit]` (Pi Camera V2).*

2. **Test Stream Capture**:
   - `pi/perception.py` uses native **Picamera2** by default, capturing directly at 320x320 without requiring `libcamerify`.
   - On Raspberry Pi OS Bookworm with libcamera v0.6+, running OpenCV with `libcamerify` can trigger `Assertion '!isArray_' failed`. Native Picamera2 avoids this entirely.

---

## Stage 5: Hardware Pin Discovery (`pin_probe.ino`)

**MANDATORY PRE-RUN STEP**: Before flashing `pointer_control.ino`, discover the actual pins wired to the shield.

1. Open Arduino IDE on your laptop or Pi.
2. Open `arduino/pin_probe/pin_probe.ino`.

### Step 5A: Identify the Servo Pin
1. Ensure `#define RUN_SERVO_PROBE` is uncommented in `pin_probe.ino`.
2. Upload to the Uno and open the **Serial Monitor at 115200 baud**.
3. Watch the announcements:
   ```text
   >>> Testing SERVO on Pin: 2
   >>> Testing SERVO on Pin: 3
   ...
   ```
4. Each pin runs a 2-second visual wiggle ($30^\circ \rightarrow 150^\circ \rightarrow 90^\circ$). When the physical servo wiggles, record that **Pin Number**.

### Step 5B: Identify the Ultrasonic Trig/Echo Pin Pair
1. In `pin_probe.ino`:
   - Comment out `RUN_SERVO_PROBE`.
   - Uncomment `#define RUN_ULTRASONIC_PROBE`.
   - Set `#define EXCLUDED_SERVO_PIN <pin_from_5A>` (so the servo is not jittered).
2. Upload and open Serial Monitor at 115200 baud.
3. Wave your hand $10\text{--}30\text{ cm}$ in front of the HC-SR04 sensor.
4. The probe scans candidate Trig/Echo pairs and prints valid echo returns:
   ```text
   [DETECTED ECHO] Trig=9 | Echo=10 | Distance=18.4 cm
   ```
5. Move your hand closer and further to verify which pair tracks your hand movements. Record **Trig Pin** and **Echo Pin**.

---

## Stage 6: Flash Production Firmware (`pointer_control.ino`)

1. Open `arduino/pointer_control/pointer_control.ino`.
2. Update the verified pins at the top:
   ```cpp
   #define SERVO_PIN <VERIFIED_SERVO_PIN>
   #define TRIG_PIN  <VERIFIED_TRIG_PIN>
   #define ECHO_PIN  <VERIFIED_ECHO_PIN>
   ```
3. Update the comment line:
   ```cpp
   // CONFIRMED VIA pin_probe.ino ON YYYY-MM-DD
   ```
4. Click **Upload**.
5. Briefly open the Serial Monitor (115200 baud) to confirm it prints:
   ```text
   READY
   ```
6. **CRITICAL STEP**: **Close the Arduino Serial Monitor window!**
   *(Only one process can hold the serial port at a time. If the Serial Monitor is open, `perception.py` will fail with `PermissionError`)*.

---

## Stage 7: Run the Full Live Pipeline

### 7.1 Verify Serial Port Name
```bash
ls /dev/ttyACM* /dev/ttyUSB*
```
*(Typically `/dev/ttyACM0` on the Pi)*.

### 7.2 Run Tracking & Pointing
```bash
cd ~/cog
source .venv/bin/activate

# Standard run with live preview:
python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --show

# Headless run (without monitor / X11 display):
python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --no-show
```

### 7.3 Understanding Terminal Logs

**Continuous Tracking Phase (Target Moving / Off-Center)**:
```text
[FRAME 00042] TARGET: class=plastic bottle | conf=0.86 | bearing=+14.2° (right ) | dist_est=38.4cm | servo=104° | stable=0/5
[FRAME 00043] TARGET: class=plastic bottle | conf=0.87 | bearing=+8.1°  (right ) | dist_est=37.1cm | servo=98°  | stable=0/5
```
*Servo is updated every single frame (`P,<servo_angle>`), pointing dynamically as the waste moves.*

**Stability Lock & Ultrasonic Confirmation Phase**:
```text
[FRAME 00048] TARGET: class=plastic bottle | conf=0.89 | bearing=+1.2°  (center) | dist_est=35.0cm | servo=91°  | stable=3/5
[FRAME 00049] TARGET: class=plastic bottle | conf=0.91 | bearing=+0.4°  (center) | dist_est=34.8cm | servo=90°  | stable=4/5
[FRAME 00050] TARGET: class=plastic bottle | conf=0.92 | bearing=-0.3°  (center) | dist_est=35.1cm | servo=90°  | stable=5/5

>>> CONFIRMED: plastic bottle @ 34.6cm (Stable lock at 90°)
```
*Target remained within $\pm 3^\circ$ deadband for 5 consecutive frames. Ultrasonic `Q` query fired and distance was confirmed.*

**Target Drift or Absence**:
```text
[FRAME 00055] TARGET: class=plastic bottle | conf=0.82 | bearing=-7.5°  (left  ) | dist_est=36.0cm | servo=82°  | stable=0/5
[FRAME 00060] not waste — no target (holding 82°)
```
*If the target moves outside the deadband, the stability counter resets to 0. If target is lost, servo holds its last commanded angle.*

---

## Stage 8: Calibration & Verification

### 8.1 Direction Calibration (`--servo-direction`)
1. Place a waste object to the **right** of the camera center.
2. Check terminal: bearing should be positive ($> 0^\circ$).
3. Observe physical servo:
   - **Points right toward the object**: Direction is correct (`--servo-direction 1`, default).
   - **Turns left away from the object**: Stop script (`Ctrl+C`) and re-launch with:
     ```bash
     python3 pi/perception.py --port /dev/ttyACM0 --servo-direction -1 --show
     ```

### 8.2 Mechanical Center Alignment
1. Place an object dead-center along the camera's optical axis (`bearing ≈ 0.0°`).
2. The servo receives command $90^\circ$.
3. If the pointer points slightly off-center (e.g., $5^\circ$ to one side):
   - Unscrew the SG90 plastic horn, set servo to $90^\circ$, and remount the horn so it points straight ahead.

### 8.3 Focal Length Calibration (`--focal-length`)
Estimate distance from bounding box height using calibrated pinhole optics:
1. Place a known object (e.g. 20 cm plastic bottle) at $30\text{ cm}$ and record pixel height ($h_1$).
2. Move it to $100\text{ cm}$ and record pixel height ($h_2$).
3. Run the calibration helper:
   ```bash
   python3 pi/perception.py --calibrate-focal 20.0 30.0 <h1_px> 100.0 <h2_px>
   ```
4. Copy the recommended focal length and run with:
   ```bash
   python3 pi/perception.py --focal-length <CALIBRATED_PX>
   ```

---

## Command Line Arguments Reference

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--model` | `yolov8_trash.pt` | Path to YOLO weights checkpoint |
| `--source` | `0` | Camera index (`0` for Pi Camera) or video file path |
| `--imgsz` | `320` | Resolution width/height for capture and inference |
| `--conf` | `0.25` | Confidence threshold for detection |
| `--port` | `/dev/ttyACM0` | Arduino serial port |
| `--baud` | `115200` | Arduino baud rate |
| `--mock` | `False` | Run without physical Arduino hardware |
| `--mock-dist` | `45.0` | Simulated distance in cm when in mock mode |
| `--servo-direction` | `1` | Direction multiplier: `+1` (standard) or `-1` (inverted) |
| `--fov` | `62.2` | Camera horizontal FOV in degrees (Pi Camera V2) |
| `--deadband` | `3.0` | Angular deadband in degrees for 'center' classification |
| `--stable-frames` | `5` | Consecutive centered frames required before ultrasonic read |
| `--focal-length` | `320.0` | Calibrated focal length in pixels for bbox distance estimation |
| `--min-dist` | `2.0` | Minimum valid ultrasonic distance (cm) |
| `--max-dist` | `200.0` | Maximum valid ultrasonic distance (cm) |
| `--show` / `--no-show` | `--show` | Enable/disable OpenCV preview window |
| `--calibrate-focal` | None | Run 2-point focal length calibration math and exit |

---

## Phase 2 Readiness Sign-Off Checklist

- [ ] Pi 4B runs 64-bit OS (`aarch64`) with YOLOv8 inference running at ~10–15 FPS.
- [ ] Pi Camera V2 captures frames reliably.
- [ ] Physical pins verified with `pin_probe.ino` and flashed to `pointer_control.ino`.
- [ ] `yolov8_trash.pt` downloaded and verified via `download_trash_model.py --check`.
- [ ] Continuous tracking updates servo angle live (`P,<angle>`) on every qualifying frame.
- [ ] Ultrasonic query (`Q`) fires only after target is centered for $\ge 5$ consecutive frames.
- [ ] Target drift resets stability counter and clears confirmation lock.
- [ ] Servo holds last position when target is lost.
- [ ] Calibration values recorded:
  - `SERVO_PIN`: ______
  - `TRIG_PIN`: ______
  - `ECHO_PIN`: ______
  - `SERVO_DIRECTION`: `+1` or `-1`
  - `FOCAL_LENGTH_PX`: ______

**You are now fully ready for Phase 2 (Chassis Locomotion & State Machine)!**
