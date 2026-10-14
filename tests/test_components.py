"""
test_components.py - Unit tests for the autonomous waste robot's geometry,
serial interface, and LED controller.
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
    bearing_to_turn_ms,
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
    print(f"  Center (x=160): bearing={bearing:.2f}°, angle={angle}° (PASS)")

    # 2. Target on left edge (x = 0) -> negative bearing
    bearing, angle = calculate_bearing_and_angle(0.0, img_width, fov, servo_direction=1)
    expected_bearing = -0.5 * fov  # -31.1
    assert abs(bearing - expected_bearing) < 1e-4
    print(f"  Left edge (x=0): bearing={bearing:.2f}° (PASS)")

    # 3. Target on right edge (x = 320) -> positive bearing
    bearing, angle = calculate_bearing_and_angle(320.0, img_width, fov, servo_direction=1)
    expected_bearing = 0.5 * fov  # +31.1
    assert abs(bearing - expected_bearing) < 1e-4
    print(f"  Right edge (x=320): bearing={bearing:.2f}° (PASS)")

    # 4. Mechanical clamping check
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
    dist_bottle = estimate_distance_from_bbox("plastic bottle", 100.0, focal_length_px=320.0)
    assert dist_bottle is not None
    assert abs(dist_bottle - 64.0) < 1e-4, f"Expected 64.0, got {dist_bottle}"

    # Can real height = 12.0 cm. pixel_height = 80: (12 * 320) / 80 = 48.0 cm
    dist_can = estimate_distance_from_bbox("can", 80.0, focal_length_px=320.0)
    assert dist_can is not None
    assert abs(dist_can - 48.0) < 1e-4

    # Irregular classes return None
    assert estimate_distance_from_bbox("cardboard", 100.0) is None
    assert estimate_distance_from_bbox("paper", 120.0) is None
    assert estimate_distance_from_bbox("plastic bag", 150.0) is None
    assert estimate_distance_from_bbox("pop tab", 30.0) is None
    print("  Distance estimation for 3D and irregular waste items verified (PASS)")


def test_focal_calibration_helper():
    print("\n[TEST] Testing focal length calibration math...")
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


def test_bearing_to_turn_ms():
    print("\n[TEST] Testing bearing_to_turn_ms conversion...")

    # 10 degrees at 8 ms/deg = 80 ms
    assert bearing_to_turn_ms(10.0, 8.0) == 80
    # Negative bearing -> uses abs, still 80
    assert bearing_to_turn_ms(-10.0, 8.0) == 80
    # 0 degrees -> 0 ms
    assert bearing_to_turn_ms(0.0, 8.0) == 0
    # Fractional result rounds
    assert bearing_to_turn_ms(7.5, 8.0) == 60
    print("  bearing_to_turn_ms conversion verified (PASS)")


def test_mock_serial_motion_commands():
    print("\n[TEST] Testing ArduinoSerialInterface mock motion commands...")
    iface = ArduinoSerialInterface(mock=True, mock_distance=80.0)
    assert iface.connect() is True

    # Forward
    assert iface.send_forward(300) is True
    # Backward
    assert iface.send_backward(200) is True
    # Turn left
    assert iface.send_turn_left(160) is True
    # Turn right
    assert iface.send_turn_right(160) is True
    # Stop
    assert iface.send_stop() is True
    # Speed
    assert iface.send_speed(200) is True
    # Distance
    dist = iface.query_distance()
    assert dist > 0

    # Verify commands were recorded
    cmds = iface.get_mock_commands()
    assert "F,300" in cmds
    assert "B,200" in cmds
    assert "TL,160" in cmds
    assert "TR,160" in cmds
    assert "S" in cmds
    assert "V,200" in cmds

    iface.close()
    print("  Mock serial motion commands verified (PASS)")


def test_mock_distance_shrinks_on_forward():
    print("\n[TEST] Testing mock distance shrinks on forward bursts...")
    iface = ArduinoSerialInterface(mock=True, mock_distance=80.0)
    iface.connect()

    d0 = iface.query_distance()
    assert d0 == 80.0

    iface.send_forward(300)  # Should shrink by ~5 cm
    d1 = iface.query_distance()
    assert d1 < d0, f"Expected distance to shrink after forward, got {d1} >= {d0}"

    iface.send_forward(300)
    d2 = iface.query_distance()
    assert d2 < d1, f"Expected continued shrink, got {d2} >= {d1}"

    # Backward should NOT shrink
    iface.send_backward(300)
    d3 = iface.query_distance()
    assert d3 == d2, f"Backward should not change distance, got {d3} != {d2}"

    iface.close()
    print("  Mock distance shrink on forward verified (PASS)")


def test_no_send_point_method():
    print("\n[TEST] Verifying send_point is removed from serial interface...")
    iface = ArduinoSerialInterface(mock=True)
    assert not hasattr(iface, "send_point"), "send_point should be removed (no servo)"
    iface.close()
    print("  send_point method correctly absent (PASS)")


def test_led_controller_mock():
    print("\n[TEST] Testing LEDController in mock mode...")
    # Import from robot module
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pi")))
    from robot import LEDController

    leds = LEDController(red_pin=17, green_pin=27, mock=True)
    leds.on("red")
    leds.on("green")
    leds.off()
    leds.blink("red", times=2, on_s=0.05, off_s=0.05)
    leds.blink_both(times=1, on_s=0.05, off_s=0.05)
    leds.cleanup()
    print("  LEDController mock operations verified (PASS)")


if __name__ == "__main__":
    test_bearing_math()
    test_direction_deadband()
    test_distance_estimation()
    test_focal_calibration_helper()
    test_bearing_to_turn_ms()
    test_mock_serial_motion_commands()
    test_mock_distance_shrinks_on_forward()
    test_no_send_point_method()
    test_led_controller_mock()
    print("\nALL UNIT TESTS PASSED!")
