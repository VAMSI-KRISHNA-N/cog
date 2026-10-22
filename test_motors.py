#!/usr/bin/env python3
"""
test_motors.py - Quick Hardware Diagnostic for 4WD Motors and Ultrasonic Sensor.
Tests direct communication with Arduino Uno and L298N driver.
"""

import sys
import time
import serial
import glob

def find_arduino():
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    return candidates[0] if candidates else None

def main():
    print("=" * 60)
    print("   COG ROBOT — DIRECT MOTOR & SENSOR DIAGNOSTIC")
    print("=" * 60)

    port = find_arduino()
    if not port:
        print("[ERROR] Arduino not found on /dev/ttyACM* or /dev/ttyUSB*!")
        print("        Is the Arduino USB cable plugged into the Pi?")
        sys.exit(1)

    print(f"[1/4] Connecting to Arduino on {port} (115200 baud)...")
    try:
        ser = serial.Serial(port, 115200, timeout=1.0)
        time.sleep(2.0)  # Wait for Arduino boot
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        sys.exit(1)

    print("[2/4] Testing Ultrasonic Sensor...")
    ser.write(b"Q\n")
    time.sleep(0.1)
    reply = ser.readline().decode("utf-8", errors="ignore").strip()
    print(f"      Arduino Reply to 'Q': '{reply}'")

    print("\n[3/4] Setting Motor Speed to FULL (PWM = 220)...")
    ser.write(b"V,220\n")
    time.sleep(0.1)

    print("\n[4/4] TESTING WHEEL MOTION (Wheels should be elevated!):")
    commands = [
        ("F,1000", "FORWARD (1 second)"),
        ("B,1000", "BACKWARD (1 second)"),
        ("TL,800", "PIVOT LEFT (0.8 second)"),
        ("TR,800", "PIVOT RIGHT (0.8 second)"),
    ]

    for cmd, desc in commands:
        print(f"\n   >>> Sending {desc}: {cmd}")
        ser.write(f"{cmd}\n".encode())
        t0 = time.time()
        ack = ""
        while time.time() - t0 < 2.0:
            if ser.in_waiting > 0:
                ack = ser.readline().decode("utf-8", errors="ignore").strip()
                if ack == "A":
                    break
            time.sleep(0.05)
        print(f"       Arduino Acknowledged: '{ack}' (Finished)")
        time.sleep(0.5)

    ser.write(b"S\n")
    ser.close()
    print("\n" + "=" * 60)
    print("Test finished!")
    print("If wheels did NOT turn, check:")
    print("  1. Is the motor battery pack connected and switched ON?")
    print("  2. Is L298N GND connected to Arduino GND (common ground)?")
    print("  3. Are L298N ENA & ENB connected to Arduino Pins 5 & 6?")
    print("=" * 60)

if __name__ == "__main__":
    main()
