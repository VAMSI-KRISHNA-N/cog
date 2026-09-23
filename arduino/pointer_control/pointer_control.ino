/*
 * pointer_control.ino - Phase 1 Stationary Waste-Detection Pointing System
 *
 * Hardware:
 *   - Arduino Uno
 *   - Tower Pro SG90 Micro Servo (PWM Pin 11)
 *   - HC-SR04 Ultrasonic Distance Sensor (Trig Pin 9, Echo Pin 10)
 *
 * Protocol:
 *   - Startup: Arduino prints "READY\n" once initialized.
 *   - P,<angle> -> Moves servo to <angle> (clamped between 0 and 180 degrees).
 *   - Q         -> Triggers an HC-SR04 distance reading.
 *                  Replies "R,<distance_cm>\n" if valid (2 to 200 cm),
 *                  or "R,-1\n" if timed out or out of valid range.
 *
 * Baud Rate: 115200 bps
 */

#include <Servo.h>

// Pin definitions
#define TRIG_PIN 9
#define ECHO_PIN 10
#define SERVO_PIN 11

// Ultrasonic measurement bounds (cm)
#define MIN_DISTANCE_CM 2.0
#define MAX_DISTANCE_CM 200.0

// Timeout for pulseIn: 25000 microseconds corresponds to ~430 cm (round trip)
#define PULSE_TIMEOUT_US 25000UL

Servo pointerServo;
String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB boards if used)
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pointerServo.attach(SERVO_PIN);
  pointerServo.write(90); // Default to centered position (90 deg)
  delay(500);             // Settle servo

  // Startup handshake
  Serial.println("READY");
}

void loop() {
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

void processCommand(const String &cmd) {
  if (cmd.startsWith("P,")) {
    // Command: P,<angle>
    String angleStr = cmd.substring(2);
    int targetAngle = angleStr.toInt();

    // Clamp angle to safe SG90 mechanical bounds [0, 180]
    if (targetAngle < 0) targetAngle = 0;
    if (targetAngle > 180) targetAngle = 180;

    pointerServo.write(targetAngle);
  }
  else if (cmd == "Q") {
    // Command: Q (Query Ultrasonic Distance)
    float distance = readUltrasonicDistanceCm();

    if (distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM) {
      Serial.print("R,");
      Serial.println(distance, 1);
    } else {
      Serial.println("R,-1");
    }
  }
  // Unknown commands are safely ignored
}

float readUltrasonicDistanceCm() {
  // Ensure trigger pin is low
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10us high pulse to trigger ultrasonic burst
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure round-trip echo duration in microseconds
  unsigned long durationUs = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);

  if (durationUs == 0) {
    return -1.0; // Timeout / no echo received
  }

  // Speed of sound in air at ~20°C is ~0.0343 cm/us
  // Distance = (duration * speed of sound) / 2
  float distanceCm = (durationUs * 0.0343) / 2.0;
  return distanceCm;
}
