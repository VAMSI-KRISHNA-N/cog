/*
 * pin_probe.ino - Hardware Pin Diagnostic & Discovery Tool
 * 
 * Purpose:
 *   Quickly discover and verify physical pin connections on an Arduino Uno
 *   when wiring is unknown or needs verification.
 * 
 * Features:
 *   1. SERVO PROBE: Sweeps a servo on pins 2–13 in sequence with pin announcements
 *      and a 2-second visual pause between pins.
 *   2. ULTRASONIC PROBE: Loops through candidate Trig/Echo pin pairs (2–13,
 *      excluding the identified servo pin), sending trigger pulses and reading
 *      echoes so the real sensor pair can be identified by waving your hand.
 * 
 * Baud Rate: 115200 bps
 */

#include <Servo.h>

// ============================================================================
// MODE SELECTION: Uncomment ONE mode below to run that diagnostic
// ============================================================================
#define RUN_SERVO_PROBE
// #define RUN_ULTRASONIC_PROBE

// ============================================================================
// CONFIGURATION
// ============================================================================
// When running the Ultrasonic probe, set this to the pin your servo was found on
// so it is excluded from ultrasonic testing (prevents jittering the servo).
#define EXCLUDED_SERVO_PIN 11

// Ultrasonic filter: When true, only prints pairs that receive a realistic echo
// (2cm to 250cm). This prevents flooding the Serial Monitor with 100+ lines of timeouts.
#define ONLY_PRINT_SANE_ECHOES true

#define PULSE_TIMEOUT_US 25000UL // ~4.2 meters max range

// ============================================================================
// IMPLEMENTATION
// ============================================================================
Servo probeServo;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port
  }
  delay(1000);

  Serial.println("==================================================");
  Serial.println("       ARDUINO PIN PROBE DIAGNOSTIC TOOL          ");
  Serial.println("==================================================");

#if defined(RUN_SERVO_PROBE)
  Serial.println("[MODE] SERVO PIN PROBE ACTIVE");
  Serial.println("Instructions:");
  Serial.println(" - Watch the Serial Monitor for the pin number.");
  Serial.println(" - The servo will wiggle (30° -> 150° -> 90°) when its pin is reached.");
  Serial.println(" - There is a 2-second pause between pins.");
  Serial.println("==================================================\n");

#elif defined(RUN_ULTRASONIC_PROBE)
  Serial.println("[MODE] ULTRASONIC PIN DISCOVERY ACTIVE");
  Serial.print("Excluding Servo Pin: ");
  Serial.println(EXCLUDED_SERVO_PIN);
  Serial.println("Instructions:");
  Serial.println(" - Wave your hand 10–30 cm in front of the HC-SR04 sensor.");
  Serial.println(" - Look for consistent 'Trig=X Echo=Y Distance=Zcm' lines.");
  Serial.println(" - As you move your hand closer/further, that specific pair will track it.");
  Serial.println("==================================================\n");
#else
  Serial.println("[ERROR] No probe mode selected!");
  Serial.println("Please uncomment either #define RUN_SERVO_PROBE or #define RUN_ULTRASONIC_PROBE.");
#endif
}

void loop() {
#if defined(RUN_SERVO_PROBE)
  runServoProbe();
#elif defined(RUN_ULTRASONIC_PROBE)
  runUltrasonicProbe();
#endif
}

// ----------------------------------------------------------------------------
// Mode 1: Servo Pin Sweep (Pins 2 to 13)
// ----------------------------------------------------------------------------
void runServoProbe() {
  for (int pin = 2; pin <= 13; pin++) {
    Serial.print(">>> Testing SERVO on Pin: ");
    Serial.println(pin);

    // Attach servo to candidate pin
    probeServo.attach(pin);

    // Perform distinct waggle so movement is unmistakable
    probeServo.write(30);
    delay(400);
    probeServo.write(150);
    delay(400);
    probeServo.write(90);
    delay(300);

    // Detach so this pin returns to neutral and doesn't interfere with next pins
    probeServo.detach();

    Serial.println("    [Pause 2s] If servo moved, your servo pin is the one above.\n");
    delay(2000);
  }

  Serial.println("--------------------------------------------------");
  Serial.println("Completed pins 2–13 sweep. Restarting in 3 seconds...");
  Serial.println("--------------------------------------------------\n");
  delay(3000);
}

// ----------------------------------------------------------------------------
// Mode 2: Ultrasonic Candidate Discovery (Pins 2 to 13)
// ----------------------------------------------------------------------------
void runUltrasonicProbe() {
  bool foundAnyEcho = false;

  for (int trig = 2; trig <= 13; trig++) {
    if (trig == EXCLUDED_SERVO_PIN) continue;

    for (int echo = 2; echo <= 13; echo++) {
      if (echo == EXCLUDED_SERVO_PIN || echo == trig) continue;

      // Configure candidate pin roles
      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);

      // Send 10µs trigger pulse
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);

      // Measure echo duration
      unsigned long duration = pulseIn(echo, HIGH, PULSE_TIMEOUT_US);

      if (duration > 0) {
        float distanceCm = (duration * 0.0343) / 2.0;

        // Check if distance is in sane physical ultrasonic range
        if (distanceCm >= 2.0 && distanceCm <= 250.0) {
          foundAnyEcho = true;
          Serial.print("[DETECTED ECHO] Trig=");
          Serial.print(trig);
          Serial.print(" | Echo=");
          Serial.print(echo);
          Serial.print(" | Distance=");
          Serial.print(distanceCm, 1);
          Serial.println(" cm");
        }
      } else {
#if !ONLY_PRINT_SANE_ECHOES
        Serial.print("Trig=");
        Serial.print(trig);
        Serial.print(" Echo=");
        Serial.print(echo);
        Serial.println(" Distance=-1 (No echo)");
#endif
      }

      // Small settling delay between pin permutations
      delay(15);
    }
  }

  // Periodic heartbeat if no obstacle is detected
  if (!foundAnyEcho) {
    Serial.println("[Scanning candidate pin pairs... Wave hand in front of sensor]");
  }

  delay(200);
}
