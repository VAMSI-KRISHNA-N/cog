// ============================================================================
// robot_control.ino — Autonomous 4WD Waste-Detection Robot Firmware
//
// Hardware:
//   - Arduino Uno
//   - L298N dual H-bridge motor driver (4WD chassis)
//   - HC-SR04 ultrasonic distance sensor
//   - Powered via USB from Raspberry Pi 4B (shared ground)
//
// VERIFIED PIN CONFIGURATION (from working lab sketches):
// ============================================================================

// --- L298N Motor Driver Pins ---
const int ENA = 5;   // PWM speed control — left motors
const int ENB = 6;   // PWM speed control — right motors
const int IN1 = 7;   // Left motor direction A
const int IN2 = 8;   // Left motor direction B
const int IN3 = 9;   // Right motor direction A
const int IN4 = 11;  // Right motor direction B

// --- HC-SR04 Ultrasonic Sensor ---
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// --- IR Line Sensors (RESERVED — do NOT use these pins) ---
// Pin 2:  IR sensor (reserved)
// Pin 4:  IR sensor (reserved)
// Pin 10: IR sensor (reserved)

// --- Default Motor Speed (0–255 PWM) ---
int motorSpeed = 220;

// --- Ultrasonic Measurement Bounds (cm) ---
const float MIN_DISTANCE_CM = 2.0;
const float MAX_DISTANCE_CM = 200.0;

// --- pulseIn timeout: 30000 µs ≈ 515 cm round-trip ---
const unsigned long PULSE_TIMEOUT_US = 30000UL;

// --- Motion Timing (non-blocking) ---
const unsigned int MAX_MOTION_MS = 1500;  // Clamp all motion durations
bool motionActive = false;
unsigned long motionEndTime = 0;

// --- Serial Input Buffer ---
String inputBuffer = "";

// ============================================================================
// DIRECTION FUNCTIONS
//
// IMPORTANT: The lab sketch function names are MISLEADING.
//   Lab sketch "back()"    = Physical FORWARD  (toward camera/ultrasonic)
//   Lab sketch "forward()" = Physical BACKWARD  (away from camera)
// The functions below are named by PHYSICAL direction (what the robot
// actually does on the floor), NOT the lab sketch names.
// ============================================================================

void driveForward() {
  // Physical FORWARD: moves toward the camera/ultrasonic side
  // (Lab sketch calls this "back()")
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void driveBackward() {
  // Physical BACKWARD: moves away from the camera/ultrasonic side
  // (Lab sketch calls this "forward()")
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void pivotLeft() {
  // Pivot LEFT in place: left wheels backward, right wheels forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void pivotRight() {
  // Pivot RIGHT in place: left wheels forward, right wheels backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  motionActive = false;
}

// ============================================================================
// ULTRASONIC READING
// ============================================================================

float readUltrasonicDistanceCm() {
  // Ensure trigger pin is low
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10 µs high pulse to trigger ultrasonic burst
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure round-trip echo duration in microseconds
  unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);

  if (durationUs == 0) {
    return -1.0;  // Timeout / no echo received
  }

  // Speed of sound in air at ~20 °C ≈ 0.0343 cm/µs
  // Distance = (duration × speed_of_sound) / 2
  float distanceCm = (durationUs * 0.0343) / 2.0;
  return distanceCm;
}

// ============================================================================
// COMMAND PROCESSING
//
// Protocol (newline-terminated):
//   F,<ms>    Drive forward for <ms> milliseconds (clamp ≤ 1500)
//   B,<ms>    Drive backward for <ms> milliseconds
//   TL,<ms>   Pivot left for <ms> milliseconds
//   TR,<ms>   Pivot right for <ms> milliseconds
//   S         Stop immediately
//   V,<0-255> Set motor PWM speed
//   Q         Query ultrasonic distance → replies R,<cm> or R,-1
//
// Motion commands reply "A\n" after completion (or override).
// ============================================================================

void startTimedMotion(unsigned int ms) {
  if (ms > MAX_MOTION_MS) ms = MAX_MOTION_MS;
  if (ms == 0) {
    stopMotors();
    return;
  }
  motionActive = true;
  motionEndTime = millis() + ms;
}

void processCommand(const String &cmd) {
  // --- Stop (highest priority) ---
  if (cmd == "S") {
    stopMotors();
    // No reply for S
    return;
  }

  // --- Any new command overrides a running motion ---
  if (motionActive) {
    stopMotors();
  }

  // --- Forward ---
  if (cmd.startsWith("F,")) {
    unsigned int ms = (unsigned int)cmd.substring(2).toInt();
    driveForward();
    startTimedMotion(ms);
  }
  // --- Backward ---
  else if (cmd.startsWith("B,")) {
    unsigned int ms = (unsigned int)cmd.substring(2).toInt();
    driveBackward();
    startTimedMotion(ms);
  }
  // --- Turn Left ---
  else if (cmd.startsWith("TL,")) {
    unsigned int ms = (unsigned int)cmd.substring(3).toInt();
    pivotLeft();
    startTimedMotion(ms);
  }
  // --- Turn Right ---
  else if (cmd.startsWith("TR,")) {
    unsigned int ms = (unsigned int)cmd.substring(3).toInt();
    pivotRight();
    startTimedMotion(ms);
  }
  // --- Set Speed ---
  else if (cmd.startsWith("V,")) {
    int spd = cmd.substring(2).toInt();
    if (spd < 0) spd = 0;
    if (spd > 255) spd = 255;
    motorSpeed = spd;
    // No reply for V
  }
  // --- Ultrasonic Query ---
  else if (cmd == "Q") {
    float distance = readUltrasonicDistanceCm();
    if (distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM) {
      Serial.print("R,");
      Serial.println(distance, 1);
    } else {
      Serial.println("R,-1");
    }
    // Q does not set motionActive, no A reply
  }
  // Unknown commands are safely ignored
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for serial port (needed for native USB boards)
  }

  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Ensure motors are stopped on boot
  stopMotors();

  // Startup handshake
  Serial.println("READY");
}

void loop() {
  // --- Check for motion timeout (non-blocking) ---
  if (motionActive && millis() >= motionEndTime) {
    stopMotors();
    Serial.println("A");  // Acknowledge completion
  }

  // --- Read serial commands ---
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += inChar;
      // Prevent buffer overflow from malformed serial floods
      if (inputBuffer.length() > 32) {
        inputBuffer = "";
      }
    }
  }
}
