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

| Component | Wire / Pin | Arduino Uno Pin | Purpose |
| :--- | :--- | :--- | :--- |
| **SG90 Servo** | Orange / Yellow | **Pin 11** *(PWM)* | Servo PWM signal |
| | Red | **5V** | Power |
| | Brown / Black | **GND** | Ground |
| **HC-SR04** | VCC | **5V** | Power |
| | Trig | **Pin 9** | 10µs trigger pulse |
| | Echo | **Pin 10** | Echo pulse timing |
| | GND | **GND** | Ground |

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
- **Via USB Drive**: Copy the `cog/` folder from your laptop to `/home/pi/cog` (or your user's home directory).
- **Or via Git**: Clone your repository to `~/cog`.

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

---

## Stage 4: Camera Verification

Before running YOLO, verify that the Pi Camera V2 is recognized by the OS and OpenCV:

1. **Check OS Camera Detection**:
   ```bash
   rpicam-hello --list-cameras
   ```
   *Should list: `imx219 [3280x2464 10-bit]` (Pi Camera V2).*

2. **Test OpenCV Capture via `libcamerify`**:
   ```bash
   libcamerify python3 -c "import cv2; cap=cv2.VideoCapture(0); ret, f=cap.read(); print('Camera Capture Success:', ret); cap.release()"
   ```
   - If it prints `Camera Capture Success: True`, you are ready!
   - **Note**: On modern Pi OS, running with `libcamerify` bridges the libcamera stack to OpenCV seamlessly.

---

## Stage 5: Hardware Pin Discovery (`pin_probe.ino`)

If the robot's physical wiring is already connected but pin numbers are unknown:

1. Open the Arduino IDE on the Pi (or on your laptop connected to the Arduino).
2. Open `arduino/pin_probe/pin_probe.ino`.
3. **Step 5A: Find the Servo Pin**:
   - Ensure `#define RUN_SERVO_PROBE` is active.
   - Upload to the Uno and open the **Serial Monitor at 115200 baud**.
   - Watch the pin announcements on screen (`>>> Testing SERVO on Pin: X`).
   - When the physical servo wiggles ($30^\circ \rightarrow 150^\circ \rightarrow 90^\circ$), note down **Pin X**.
4. **Step 5B: Find the Ultrasonic Pins**:
   - In `pin_probe.ino`, comment out `RUN_SERVO_PROBE` and uncomment `#define RUN_ULTRASONIC_PROBE`.
   - Set `#define EXCLUDED_SERVO_PIN X` (using the pin found in 5A).
   - Upload and wave your hand $10\text{--}30\text{ cm}$ in front of the HC-SR04.
   - The Serial Monitor will print:
     ```text
     [DETECTED ECHO] Trig=9 | Echo=10 | Distance=18.4 cm
     ```
   - Note down **Trig Pin** and **Echo Pin**.

---

## Stage 6: Flash Production Firmware (`pointer_control.ino`)

1. Open `arduino/pointer_control/pointer_control.ino`.
2. Confirm or update the pin numbers at the top:
   ```cpp
   #define TRIG_PIN 9
   #define ECHO_PIN 10
   #define SERVO_PIN 11
   ```
3. Click **Upload**.
4. Briefly open the Serial Monitor (115200 baud) to confirm it prints:
   ```text
   READY
   ```
5. **CRITICAL STEP**: **Close the Arduino Serial Monitor window!**
   *(Only one program can hold the serial port at a time. If the Serial Monitor is open, `perception.py` will fail with `PermissionError`)*.

---

## Stage 7: Run the Full Live Pipeline

1. In your Pi terminal, ensure the virtual environment is active:
   ```bash
   cd ~/cog
   source .venv/bin/activate
   ```
2. Verify Arduino serial port name:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```
   *(Usually `/dev/ttyACM0`)*.
3. Launch the perception and pointing pipeline:
   ```bash
   libcamerify python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --show
   ```

4. **Verify Terminal Logs**:
   ```text
   [SERIAL] Connecting to Arduino on /dev/ttyACM0 at 115200 baud...
   [SERIAL RX] READY
   [SERIAL] Handshake successful! Arduino is ready.
   ...
   [FRAME 00045] CONFIRMED TARGET: class=bottle (conf=0.84) | bearing=+12.2° | servo_angle=102° | distance=38.4 cm -> POINTED
   ```

---

## Stage 8: Servo Calibration & Verification (Phase 2 Ready)

Follow this 3-point calibration protocol to finalize your baseline:

```
          [Left Target]           [Center Target]           [Right Target]
          x < Center              x ≈ Center                x > Center
          Bearing: -15° to -25°   Bearing: -2° to +2°       Bearing: +15° to +25°
          Servo: < 90°            Servo: ≈ 90°              Servo: > 90°
```

### 8.1 Direction Calibration
1. Place a waste object to the **right** of the camera center.
2. Check terminal: bearing should be positive ($> 0^\circ$).
3. Observe physical servo:
   - **If the servo points right toward the object**: Direction is correct (`--servo-direction 1`).
   - **If the servo turns left away from the object**: Stop script (`Ctrl+C`) and re-launch with inverted direction:
     ```bash
     libcamerify python3 pi/perception.py --port /dev/ttyACM0 --servo-direction -1 --show
     ```

### 8.2 Center Alignment Calibration
1. Place an object directly along the camera's center axis.
2. When the bounding box center aligns with the camera crosshair (`bearing ≈ 0.0°`), the servo should command $90^\circ$.
3. If the physical pointer is misaligned by a slight angle (e.g. pointing $5^\circ$ off-center):
   - Unscrew the SG90 plastic horn, set the servo to $90^\circ$, and remount the horn so it points dead ahead.

### 8.3 Ultrasonic Distance Fusion Check
1. Place an object at $40\text{ cm}$:
   - Servo must point at the object.
   - Status bar displays: `CONFIRMED: <class> @ 40.0cm | Angle: X°`.
2. Move the object beyond $200\text{ cm}$ (or block the sensor):
   - Terminal prints: `UNCONFIRMED TARGET: ... OUT OF RANGE / TIMEOUT -> SKIPPED`.
   - Servo remains stationary (pointing suppressed).

---

## Phase 2 Readiness Sign-Off Checklist

- [ ] Pi 4B runs 64-bit OS (`aarch64`) with YOLOv8 inference running at ~10–15 FPS.
- [ ] Pi Camera V2 captures frames via `libcamerify`.
- [ ] Arduino connects, handshakes `READY`, and parses `P,<angle>` and `Q`.
- [ ] Ultrasonic reading accurately confirms objects between $2\text{ cm}$ and $200\text{ cm}$.
- [ ] Servo accurately tracks left, center, and right positions.
- [ ] Calibration values recorded:
  - `SERVO_PIN`: ______
  - `TRIG_PIN`: ______
  - `ECHO_PIN`: ______
  - `SERVO_DIRECTION`: `+1` or `-1`
  - Serial Port: `/dev/ttyACM0` @ 115200 baud.

**You are now fully ready for Phase 2 (Driving / Navigation)!**
