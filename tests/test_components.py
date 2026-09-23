"""
test_components.py - Unit test suite for Phase 1 perception math and serial interface.
"""

import sys
import os

# Add pi directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pi")))

from serial_interface import ArduinoSerialInterface
from geometry import calculate_bearing_and_angle, HORIZONTAL_FOV


def test_bearing_math():
    print("[TEST] Testing bearing and servo angle math...")
    img_width = 320.0
    fov = HORIZONTAL_FOV  # 62.2 degrees

    # 1. Target exactly in center (x = 160)
    bearing, angle = calculate_bearing_and_angle(160.0, img_width, fov, servo_direction=1)
    assert abs(bearing - 0.0) < 1e-4, f"Expected 0.0 bearing, got {bearing}"
    assert angle == 90, f"Expected 90 degree servo angle, got {angle}"
    print(f"  Center (x=160): bearing={bearing:.2f}°, servo_angle={angle}° (PASS)")

    # 2. Target on left edge (x = 0)
    bearing, angle = calculate_bearing_and_angle(0.0, img_width, fov, servo_direction=1)
    expected_bearing = -0.5 * fov  # -31.1
    assert abs(bearing - expected_bearing) < 1e-4
    assert angle == round(90.0 - 31.1)  # 59
    print(f"  Left edge (x=0): bearing={bearing:.2f}°, servo_angle={angle}° (PASS)")

    # 3. Target on right edge (x = 320)
    bearing, angle = calculate_bearing_and_angle(320.0, img_width, fov, servo_direction=1)
    expected_bearing = 0.5 * fov  # +31.1
    assert abs(bearing - expected_bearing) < 1e-4
    assert angle == round(90.0 + 31.1)  # 121
    print(f"  Right edge (x=320): bearing={bearing:.2f}°, servo_angle={angle}° (PASS)")

    # 4. Inverted servo direction (-1)
    bearing, angle_inv = calculate_bearing_and_angle(320.0, img_width, fov, servo_direction=-1)
    assert angle_inv == round(90.0 - 31.1)  # 59
    print(f"  Inverted direction on right edge: bearing={bearing:.2f}°, servo_angle={angle_inv}° (PASS)")

    # 5. Clamping check with exaggerated FOV
    _, clamped_low = calculate_bearing_and_angle(0.0, img_width, 250.0, servo_direction=1)
    assert clamped_low == 0, f"Expected clamp to 0, got {clamped_low}"
    _, clamped_high = calculate_bearing_and_angle(320.0, img_width, 250.0, servo_direction=1)
    assert clamped_high == 180, f"Expected clamp to 180, got {clamped_high}"
    print(f"  Mechanical clamping [0, 180] verified (PASS)")


def test_mock_serial():
    print("\n[TEST] Testing ArduinoSerialInterface in mock mode...")
    serial_iface = ArduinoSerialInterface(mock=True, mock_distance=42.5)

    assert serial_iface.connect() is True
    assert serial_iface.send_point(110) is True
    dist = serial_iface.query_distance()
    assert dist == 42.5, f"Expected mock distance 42.5, got {dist}"

    # Angle clamping test
    assert serial_iface.send_point(200) is True  # Should clamp to 180
    assert serial_iface.send_point(-50) is True  # Should clamp to 0

    serial_iface.close()
    print("  Mock serial interface verified (PASS)")


if __name__ == "__main__":
    test_bearing_math()
    test_mock_serial()
    print("\nALL UNIT TESTS PASSED!")
