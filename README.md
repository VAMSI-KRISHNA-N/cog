# Waste-Detection Pointing System — Phase 1 (Stationary)

Phase 1 stationary perception and pointing system. A camera captures images, runs YOLO inference to detect objects, calculates their bearing angle, confirms target presence using an HC-SR04 ultrasonic sensor, and commands an SG90 micro servo via an Arduino Uno to point directly at the detected waste item.

---

## 1. Hardware Pinout & Wiring

| Component | Pin / Wire | Arduino Uno Connection | Notes |
| :--- | :--- | :--- | :--- |
| **SG90 Servo** | Orange / Yellow (Signal) | **Pin 11 (PWM)** | Servo control pulse |
| | Red (VCC) | **5V** (or external 5V) | External 5V recommended if servo causes brownout |
| | Brown / Black (GND) | **GND** | Common ground with Arduino |
| **HC-SR04** | VCC | **5V** | Power |
| | Trig | **Pin 9** | 10µs trigger pulse |
| | Echo | **Pin 10** | Round-trip echo duration |
| | GND | **GND** | Common ground |
| **Pi / Laptop** | USB-A to USB-B Cable | **Arduino USB Port** | Supplies power & serial communication (115200 baud) |

---

## 2. Serial Communication Protocol

- **Baud Rate**: `115200`
- **Line Ending**: `\n`

| Direction | Command / Message | Description |
| :--- | :--- | :--- |
| Arduino $\rightarrow$ Pi | `READY` | Startup handshake sent when Arduino finishes initialization. |
| Pi $\rightarrow$ Arduino | `Q` | Query ultrasonic distance. |
| Arduino $\rightarrow$ Pi | `R,<distance_cm>` | Reply with measured distance in cm (e.g. `R,45.2`). Returns `R,-1` on timeout or out-of-range (<2cm or >200cm). |
| Pi $\rightarrow$ Arduino | `P,<angle>` | Point servo to specified integer angle `[0, 180]` (e.g. `P,105`). |

---

## 3. Tonight's Acceptance Test (Laptop Webcam + Mock Mode)

You can run and test the complete pipeline on your laptop tonight without any hardware connected.

### Step 1: Install Dependencies
```bash
pip install -r requirements.txt
```

### Step 2: Run in Mock Mode with Webcam
```bash
python pi/perception.py --mock --source 0 --show
```

### Step 3: Verify Expected Behavior
1. **Target Detection**: Point your webcam at any common object (cell phone, bottle, cup, person).
2. **Bearing Angle**:
   - Move the object to the **left**: bearing is negative, servo angle decreases below $90^\circ$.
   - Move the object to the **center**: bearing is near $0.0^\circ$, servo angle is $\approx 90^\circ$.
   - Move the object to the **right**: bearing is positive, servo angle increases above $90^\circ$.
3. **Mock Serial Outputs**:
   - Watch the console print:
     ```text
     [MOCK SERIAL TX] Q (Query distance)
     [MOCK SERIAL RX] R,45.0
     [MOCK SERIAL TX] P,102 (Point servo to 102°)
     [FRAME 00042] CONFIRMED TARGET: class=bottle (conf=0.88) | bearing=+12.4° | servo_angle=102° | distance=45.0 cm -> POINTED
     ```
4. **Test Implausible Distance Fusion**:
   ```bash
   python pi/perception.py --mock --mock-dist 250.0 --source 0 --show
   ```
   The detection will be marked `UNCONFIRMED` and no `P,<angle>` command will be sent.

---

## 4. Tomorrow's Lab Deployment (Raspberry Pi + Arduino)

### Step 1: Upload Arduino Firmware
1. Open `arduino/pointer_control/pointer_control.ino` in the Arduino IDE.
2. Select Board: **Arduino Uno** and your USB Serial Port.
3. Click **Upload**.
4. (Optional) Open the Serial Monitor at **115200 baud** to confirm it prints `READY`.

### Step 2: Connect to Raspberry Pi 4B
1. Plug the Arduino Uno into any USB port on the Raspberry Pi 4B.
2. Connect the Pi Camera V2 to the CSI port.
3. Check the serial port name on the Pi (usually `/dev/ttyACM0` or `/dev/ttyUSB0`):
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```

### Step 3: Run Real Perception Pipeline on Pi
```bash
# In headless mode (via SSH):
python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --no-show

# Or with HDMI monitor / VNC GUI:
python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --show
```

### Step 4: Physical Servo Calibration
Depending on how the SG90 horn and sensor mount are mechanically oriented:
- If the servo points in the opposite direction of the object, invert the `--servo-direction` multiplier:
  ```bash
  python3 pi/perception.py --port /dev/ttyACM0 --servo-direction -1
  ```
- Adjust the horn alignment so $90^\circ$ corresponds to straight ahead (parallel with camera center line).

---

## 5. Swapping In a Custom Waste Model
When your custom waste classification model is trained, simply pass its checkpoint path:
```bash
python3 pi/perception.py --model path/to/waste_model.pt
```
The script dynamically retrieves class labels from the model without any hardcoded class assumptions.

---

## 6. Hardware Pin Discovery Tool (`pin_probe.ino`)

If you arrive at the lab and the pin wiring on the robot is unknown or unverified:

1. Open `arduino/pin_probe/pin_probe.ino` in the Arduino IDE.
2. **Find the Servo Pin**:
   - Keep `#define RUN_SERVO_PROBE` uncommented.
   - Open Serial Monitor at **115200 baud**.
   - Watch the pin number printed on screen; when the physical servo wiggles ($30^\circ \rightarrow 150^\circ \rightarrow 90^\circ$), note that pin number.
3. **Find the Ultrasonic Sensor Pins**:
   - In `pin_probe.ino`, comment out `RUN_SERVO_PROBE` and uncomment `#define RUN_ULTRASONIC_PROBE`.
   - Set `#define EXCLUDED_SERVO_PIN <your_servo_pin>`.
   - Upload and wave your hand $10\text{--}30\text{ cm}$ in front of the HC-SR04 sensor.
   - The Serial Monitor will print:
     ```text
     [DETECTED ECHO] Trig=9 | Echo=10 | Distance=18.4 cm
     ```
4. Update `pointer_control.ino` with the discovered pin numbers!

