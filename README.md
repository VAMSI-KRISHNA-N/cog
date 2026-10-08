# Waste Robot — Phase 1 (Stationary Continuous Detect, Track & Point)

Full implementation of Phase 1 stationary perception and continuous pointing system for the waste collection robot.

---

## 1. System Architecture & Core Flow

```
Camera (Pi Camera V2)
      │
      ▼
YOLOv8 Waste Detector (10 Classes)
      │
      ▼
Pick Highest-Confidence Target
      │
      ├────────────────────────────────────────────────────────┐
      ▼                                                        ▼
Calculate Bearing & Direction                          BBox Height Distance Estimate
(Deadband: ±3.0° -> left / right / center)             dist = (real_height * f_px) / h_px
      │                                                        │
      ▼                                                        ▼
Servo Angle Mapping (90° + DIR * bearing)            Log live distance estimate
      │
      ▼
Send P,<angle> EVERY Qualifying Frame (Continuous Tracking)
      │
      ▼
Is Target Centered in Deadband?
      ├── YES ──► stable_counter += 1
      │             └──► stable_counter >= 5 frames?
      │                    └──► Trigger Ultrasonic Q -> R,<cm>
      │                         └──► Confirm: "CONFIRMED: <class> @ <dist>cm"
      │
      └── NO (Target Drifted / Lost)
            └──► Reset stable_counter to 0; unconfirm lock
                 If no target: Servo holds last commanded angle
```

---

## 2. 10 Waste Categories (Roboflow Universe yolov8-trash-detections)

The system detects and retains the actual class names for all 10 waste categories:
- `plastic bottle` (~20 cm real height)
- `can` (~12 cm real height)
- `glass bottle` (~25 cm real height)
- `drink carton` (~15 cm real height)
- `battery` (~5 cm real height)
- `cardboard` (irregular height $\rightarrow$ distance returned as `unknown`)
- `paper` (irregular height $\rightarrow$ distance returned as `unknown`)
- `plastic bag` (irregular height $\rightarrow$ distance returned as `unknown`)
- `plastic` (irregular height $\rightarrow$ distance returned as `unknown`)
- `pop tab` (irregular height $\rightarrow$ distance returned as `unknown`)

---

## 3. Distance Estimation & Focal Length Calibration

The camera estimates distance based on pinhole optics:
$$\text{focal\_length\_px} = \frac{\text{pixel\_height} \times \text{distance\_cm}}{\text{real\_height\_cm}}$$
$$\text{distance\_estimate} = \frac{\text{real\_height\_cm} \times \text{focal\_length\_px}}{\text{current\_pixel\_height}}$$

### Calibrating in the Lab:
Place a reference object (e.g. a 20cm plastic bottle) at 30cm and 100cm, record pixel heights from the bbox, and run the calibration tool:
```bash
python3 pi/perception.py --calibrate-focal 20.0 30.0 <h1_px> 100.0 <h2_px>
```
The script will compute the average focal length and verify agreement within 15–20%.

---

## 4. Hardware Pinout & Wiring

> [!IMPORTANT]
> **UNCONFIRMED DEFAULT PINS**: The pins below are placeholder defaults.
> **DO NOT assume they are correct for your kit's shield.** You **MUST** run `arduino/pin_probe/pin_probe.ino` first to confirm physical connections before flashing `arduino/pointer_control/pointer_control.ino`.

| Component | Pin / Wire | Arduino Uno Connection | Status | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **SG90 Servo** | Orange / Yellow (Signal) | **Pin 11 (PWM)** | *Unconfirmed default* | Servo control pulse |
| | Red (VCC) | **5V** | Confirmed | Power |
| | Brown / Black (GND) | **GND** | Confirmed | Common ground with Arduino |
| **HC-SR04** | VCC | **5V** | Confirmed | Power |
| | Trig | **Pin 9** | *Unconfirmed default* | 10µs trigger pulse |
| | Echo | **Pin 10** | *Unconfirmed default* | Round-trip echo duration |
| | GND | **GND** | Confirmed | Common ground |
| **Pi / Laptop** | USB-A to USB-B Cable | **Arduino USB Port** | Confirmed | Power & serial communication (115200 baud) |

---

## 5. Serial Communication Protocol

- **Baud Rate**: `115200` | **Line Ending**: `\n`

| Direction | Command / Message | Trigger | Description |
| :--- | :--- | :--- | :--- |
| Arduino $\rightarrow$ Pi | `READY` | Boot | Startup handshake when Uno finishes setup. |
| Pi $\rightarrow$ Arduino | `P,<angle>` | **Every qualifying frame** | Continuous tracking servo update (`[0, 180]`). |
| Pi $\rightarrow$ Arduino | `Q` | **Stability lock ($\ge 5$ stable frames)** | Rare, triggered ultrasonic query. |
| Arduino $\rightarrow$ Pi | `R,<cm>` | Response to `Q` | Measured distance in cm (`R,-1` on timeout). |

---

## 6. How to Run

### Automatic Launcher:
```bash
# Pull latest code
git pull

# Launch tracker (auto-detects Arduino or runs Mock mode)
./run.sh
```

### Manual CLI Run:
```bash
# Hardware mode with Pi Camera V2
python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --show

# Headless mode (without display)
python3 pi/perception.py --port /dev/ttyACM0 --baud 115200 --no-show

# Mock mode (webcam / test bench without Arduino)
python3 pi/perception.py --mock --show

# Inverted servo mounting calibration:
python3 pi/perception.py --port /dev/ttyACM0 --servo-direction -1 --show
```

### Full CLI Options Reference:
| Option | Default | Description |
| :--- | :--- | :--- |
| `--model` | `yolov8_trash.pt` | Model weights checkpoint |
| `--source` | `0` | Camera index or video path |
| `--imgsz` | `320` | Capture and inference resolution |
| `--conf` | `0.25` | Confidence cutoff |
| `--port` | `/dev/ttyACM0` | Arduino serial port |
| `--baud` | `115200` | Baud rate |
| `--mock` | `False` | Run without physical serial hardware |
| `--mock-dist` | `45.0` | Simulated ultrasonic distance |
| `--servo-direction` | `1` | Servo polarity: `+1` or `-1` |
| `--deadband` | `3.0` | Angular deadband for center categorization (±deg) |
| `--stable-frames` | `5` | Frames required to trigger ultrasonic query |
| `--focal-length` | `320.0` | Calibrated focal length in pixels |
| `--calibrate-focal` | None | Run 2-point focal calibration math |

---

## 7. Model Download & Custom Weights

To download the pre-trained Roboflow Universe `yolov8-trash-detections` weights:
```bash
# Automatic download via API:
python3 download_trash_model.py --api-key YOUR_ROBOFLOW_API_KEY

# Check if model is valid:
python3 download_trash_model.py --check
```

*(If no API key is provided, running `python3 download_trash_model.py` provides manual download instructions from Roboflow Universe).*

> [!WARNING]
> **Fallback Warning**: If `yolov8_trash.pt` is not present, `perception.py` falls back to `yolov8n.pt` (generic COCO weights). It will display a loud warning because COCO cannot detect trash items properly (e.g. mouse as apple, fan as airplane, bottle as vase).
