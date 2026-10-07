"""
test_components.py - Unit test suite for Phase 1 geometry, distance estimation, and mock serial interface.
"""

import sys
import os

# Add pi directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pi")))

from serial_interface import ArduinoSerialInterface
from geometry import (
    calculate_bearing_and_angle,
    determine_direction,
    estimate_distance_from_bbox,
    calibrate_focal_length,
    HORIZONTAL_FOV,
    DEADBAND_DEG,
    FOCAL_LENGTH_PX,
)


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

    # 5. Mechanical clamping check
    _, clamped_low = calculate_bearing_and_angle(0.0, img_width, 250.0, servo_direction=1)
    assert clamped_low == 0, f"Expected clamp to 0, got {clamped_low}"
    _, clamped_high = calculate_bearing_and_angle(320.0, img_width, 250.0, servo_direction=1)
    assert clamped_high == 180, f"Expected clamp to 180, got {clamped_high}"
    print("  Mechanical clamping [0, 180] verified (PASS)")


def test_direction_deadband():
    print("\n[TEST] Testing direction categorization with deadband...")
    deadband = DEADBAND_DEG  # 3.0°

    assert determine_direction(0.0, deadband) == "center"
    assert determine_direction(2.5, deadband) == "center"
    assert determine_direction(-2.9, deadband) == "center"
    assert determine_direction(3.1, deadband) == "right"
    assert determine_direction(15.0, deadband) == "right"
    assert determine_direction(-3.5, deadband) == "left"
    assert determine_direction(-25.0, deadband) == "left"
    print("  Direction deadband (left/right/center) verified (PASS)")


def test_distance_estimation():
    print("\n[TEST] Testing bounding-box distance estimation...")
    # Plastic bottle real height = 20.0 cm, FOCAL_LENGTH_PX = 320.0
    # Expected distance for pixel_height = 100: (20.0 * 320.0) / 100 = 64.0 cm
    dist_bottle = estimate_distance_from_bbox("plastic bottle", 100.0, focal_length_px=320.0)
    assert dist_bottle is not None
    assert abs(dist_bottle - 64.0) < 1e-4, f"Expected 64.0, got {dist_bottle}"

    # Can real height = 12.0 cm. pixel_height = 80: (12 * 320) / 80 = 48.0 cm
    dist_can = estimate_distance_from_bbox("can", 80.0, focal_length_px=320.0)
    assert dist_can is not None
    assert abs(dist_can - 48.0) < 1e-4

    # Irregular classes with no consistent height should return None
    assert estimate_distance_from_bbox("cardboard", 100.0) is None
    assert estimate_distance_from_bbox("paper", 120.0) is None
    assert estimate_distance_from_bbox("plastic bag", 150.0) is None
    assert estimate_distance_from_bbox("pop tab", 30.0) is None
    print("  Distance estimation for 3D and irregular waste items verified (PASS)")


def test_focal_calibration_helper():
    print("\n[TEST] Testing focal length calibration math...")
    # Reference bottle (20cm) measured at:
    # d1 = 30cm -> pixel height should be (20 * 320) / 30 = 213.33 px
    # d2 = 100cm -> pixel height should be (20 * 320) / 100 = 64.0 px
    f1, f2, avg_f, diff = calibrate_focal_length(
        measured_pixel_height_1=213.333,
        distance_cm_1=30.0,
        real_height_cm_1=20.0,
        measured_pixel_height_2=64.0,
        distance_cm_2=100.0,
        real_height_cm_2=20.0,
    )
    assert abs(avg_f - 320.0) < 0.1
    assert diff < 1.0
    print(f"  Focal calibration derived: avg={avg_f:.1f}px, diff={diff:.2f}% (PASS)")


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
    test_direction_deadband()
    test_distance_estimation()
    test_focal_calibration_helper()
    test_mock_serial()
    print("\nALL UNIT TESTS PASSED!")
