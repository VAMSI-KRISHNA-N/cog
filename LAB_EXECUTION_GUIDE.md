# Lab Execution Guide: Autonomous Waste Robot

Step-by-step procedure to set up, flash, wire, calibrate, and deploy the autonomous waste-detection robot from a cold start.

---

## Stage 1: Physical Setup & Wiring (Power OFF)

### 1.1 Raspberry Pi 4B

1. **MicroSD Card**: Insert the flashed 64GB card (Raspberry Pi OS 64-bit).
2. **Pi Camera V2**: Insert into the **CAMERA** CSI port (between HDMI and audio jack). Blue tape faces USB ports.
3. **LEDs**:
   - **Red LED**: Connect BCM GPIO **17** (physical pin **11**) → resistor (220Ω–330Ω) → LED anode → LED cathode → GND.
   - **Green LED**: Connect BCM GPIO **27** (physical pin **13**) → resistor (220Ω–330Ω) → LED anode → LED cathode → GND.
4. **Arduino Uno**: Connect USB-A (Pi) to USB-B (Arduino). This provides both serial communication and power to the Arduino.
5. **Power**: Connect the Pi to its own USB power bank (5V / 3A minimum). The Arduino is powered through the USB cable from the Pi.

> [!IMPORTANT]
> The Pi and Arduino share a common ground through the USB cable. No additional ground wire is needed between them.

### 1.2 Arduino Uno → L298N Motor Driver

| L298N Pin | Arduino Pin | Notes |
|:--|:--|:--|
| ENA | **5** (PWM) | Left motors speed |
| ENB | **6** (PWM) | Right motors speed |
| IN1 | **7** | Left motor direction A |
| IN2 | **8** | Left motor direction B |
| IN3 | **9** | Right motor direction A |
| IN4 | **11** | Right motor direction B |
| 12V / VCC | External battery pack | Motor power (6V–12V) |
| GND | Arduino GND + Battery GND | Common ground |

### 1.3 Arduino Uno → HC-SR04 Ultrasonic

| HC-SR04 | Arduino Pin |
|:--|:--|
| VCC | 5V |
| TRIG | **12** |
| ECHO | **13** |
| GND | GND |

### 1.4 Reserved Pins

Pins **2**, **4**, **10** are reserved for IR line sensors. **Do not use these pins.**

---

## Stage 2: First Boot & OS Verification

1. Power on the Pi and open a terminal.
2. **Verify 64-bit OS**:
   ```bash
   uname -m   # Must output: aarch64
   ```
3. **Internet access**:
   ```bash
   ping -c 3 8.8.8.8
   ```
4. **Serial port permissions**:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Log out and back in for this to take effect.

---

## Stage 3: Project Setup

### 3.1 Clone Repository

```bash
git clone https://github.com/VAMSI-KRISHNA-N/cog.git ~/cog
cd ~/cog
```

### 3.2 Virtual Environment & Dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 3.3 Smoke Test

```bash
python3 -c "import cv2, serial, ultralytics; print('Dependencies OK!')"
```

### 3.4 Download Waste Model

```bash
# Automated:
python3 download_trash_model.py --api-key YOUR_ROBOFLOW_KEY

# Verify:
python3 download_trash_model.py --check
```

> [!WARNING]
> If `yolov8_trash.pt` is missing, the robot falls back to `yolov8n.pt` with a loud warning. **Test your demo objects against the model before the final demo** — "cup" may not be one of the model's 10 classes. The 10 classes are: `paper`, `can`, `cardboard`, `plastic`, `plastic bag`, `plastic bottle`, `battery`, `glass bottle`, `pop tab`, `drink carton`.

---

## Stage 4: Flash Arduino Firmware

1. Open `arduino/robot_control/robot_control.ino` in the Arduino IDE.
2. Verify the pin constants at the top match your wiring (they should — pins are from lab sketches).
3. Click **Upload**.
4. Open Serial Monitor (115200 baud) and verify it prints `READY`.
5. **CLOSE the Serial Monitor** before running the Pi script (only one process can hold the serial port).

---

## Stage 5: Camera Verification

```bash
rpicam-hello --list-cameras
# Should list: imx219 [3280x2464 10-bit] (Pi Camera V2)
```

---

## Stage 6: Bench Testing (wheels off the ground!)

> [!CAUTION]
> Always test with wheels **off the ground** first! Place the car on a box or hold it up.

### 6.1 Direction Verification

```bash
cd ~/cog && source .venv/bin/activate

# Run with preview to see what the camera sees:
python3 pi/robot.py --preview --no-shutdown
```

**Verify**:
- `F` commands spin the wheels **toward the camera/ultrasonic** side (physical forward).
- Pivot left and right turn the correct way.
- If reversed: the direction mapping in the firmware needs swapping (unlikely — these values are from verified sketches).

### 6.2 Motor Calibration with Teleop

```bash
python3 pi/robot.py --teleop --preview
```

**Keys**: `w`=forward, `s`=backward, `a`=left, `d`=right, `q`=quit, `u`=ultrasonic query, `+`/`-`=adjust duration.

**Calibrate on the real floor**:

1. **TURN_MS_PER_DEG**: Place the car on the floor. Tap `a` or `d` and measure how many degrees it turns per keypress. Adjust with `+`/`-` until a known angle is achieved. Calculate: `ms_per_keypress / degrees_turned`.
2. **DRIVE_MS_PER_CM**: Tap `w` and measure how many cm the car moves. Calculate: `ms_per_keypress / cm_moved`.

> [!TIP]
> **Redo calibration when batteries are low.** Timed moves drift as motor battery voltage sags. Fresh batteries will make the car faster — you'll over-shoot if using old calibration values.

### 6.3 Model Object Test

Place your demo objects (the same ones you'll use in the demo) in front of the camera and run:

```bash
python3 pi/robot.py --mock --preview
```

Verify that each object is detected and classified. If an object isn't detected, it's not in the model's training set — the car will never drive to it.

---

## Stage 7: Install Auto-Start Service

```bash
# Create log directory
mkdir -p ~/cog/logs

# Install systemd service
sudo cp systemd/cog-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cog-robot.service

# Install sudoers for passwordless shutdown
sudo cp systemd/cog-shutdown.sudoers /etc/sudoers.d/cog-shutdown
sudo chmod 0440 /etc/sudoers.d/cog-shutdown
sudo visudo -c  # Validate syntax
```

---

## Stage 8: Full Autonomous Test

1. Disable the service temporarily:
   ```bash
   sudo systemctl stop cog-robot.service
   ```
2. Place 2–3 demo objects on the floor.
3. Run manually with `--no-shutdown` first:
   ```bash
   python3 pi/robot.py --no-shutdown
   ```
4. Watch the car scan, drive to objects, light LEDs, and reach DONE.
5. When satisfied, enable the service and reboot:
   ```bash
   sudo systemctl enable cog-robot.service
   sudo reboot
   ```
6. Watch the car start by itself after boot.

---

## Stage 9: Demo Day

1. Disable the service if you need to make adjustments:
   ```bash
   sudo systemctl disable cog-robot.service
   ```
2. Place the car on the floor, scatter the demo objects.
3. Power on the Pi (connect the power bank).
4. The robot starts automatically:
   - Blinks both LEDs twice → "ready".
   - Waits 10 seconds for people to step away.
   - Scans, drives, classifies, lights LEDs.
   - When done: blinks, alternates LEDs, shuts down.
5. **Wait for the Pi's green ACT LED to stop blinking before cutting power.**

---

## Reading Logs

```bash
# Live journal:
journalctl -u cog-robot.service -f

# Log file:
tail -f ~/cog/logs/robot.log

# Last run:
cat ~/cog/logs/robot.log
```

---

## Disabling Auto-Start for Development

```bash
sudo systemctl disable cog-robot.service
sudo systemctl stop cog-robot.service
```

Then run the robot manually with `--preview` and `--no-shutdown` during development.

---

## Troubleshooting

| Symptom | Fix |
|:--|:--|
| Red LED blinks rapidly on boot | Arduino not found, model missing, or camera failed. Check `robot.log`. |
| Car drives the wrong way | Direction mapping is swapped. Re-check `robot_control.ino` pin assignments. |
| Car overshoots or undershoots | Recalibrate `--turn-ms-per-deg` and `--forward-burst-ms` with `--teleop`. |
| Objects not detected | Object may not be in the model's 10 classes. Test with `--preview`. |
| Serial permission denied | Run `sudo usermod -a -G dialout pi` and reboot. Or close Arduino Serial Monitor. |
| Service won't start | Check `journalctl -u cog-robot.service -e` for errors. |
