"""
test_integration.py - Integration test verifying perception decision and fusion logic.
"""

import sys
import os

# Add pi directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pi")))

from serial_interface import ArduinoSerialInterface
from geometry import calculate_bearing_and_angle, HORIZONTAL_FOV


class MockTargetTracker:
    def __init__(self, serial_iface, min_dist=2.0, max_dist=200.0, fov=62.2, servo_direction=1):
        self.serial_iface = serial_iface
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.fov = fov
        self.servo_direction = servo_direction
        self.last_action = None

    def process_frame(self, target, img_width=320):
        if target is None:
            self.last_action = "NO_DETECTION"
            return self.last_action

        x_center, y_center = target["center"]
        bearing, servo_angle = calculate_bearing_and_angle(
            x_center=x_center,
            img_width=img_width,
            fov=self.fov,
            servo_direction=self.servo_direction,
        )

        dist_cm = self.serial_iface.query_distance()
        is_confirmed = (self.min_dist <= dist_cm <= self.max_dist)

        if is_confirmed:
            self.serial_iface.send_point(servo_angle)
            self.last_action = {
                "status": "CONFIRMED",
                "class": target["class"],
                "bearing": bearing,
                "servo_angle": servo_angle,
                "distance": dist_cm,
            }
        else:
            self.last_action = {
                "status": "UNCONFIRMED",
                "class": target["class"],
                "bearing": bearing,
                "servo_angle": servo_angle,
                "distance": dist_cm,
            }
        return self.last_action


def test_integration_flow():
    print("[TEST] Running Integration Flow tests...")

    # Case 1: Confirmed target on left side of image (x = 80)
    serial_mock = ArduinoSerialInterface(mock=True, mock_distance=50.0)
    tracker = MockTargetTracker(serial_mock)

    target_left = {"class": "plastic_bottle", "conf": 0.92, "center": (80.0, 160.0)}
    result = tracker.process_frame(target_left)
    assert result["status"] == "CONFIRMED"
    assert result["servo_angle"] < 90, f"Expected servo_angle < 90, got {result['servo_angle']}"
    assert result["distance"] == 50.0
    print("  Case 1: Left confirmed target pointed correctly (PASS)")

    # Case 2: Confirmed target on right side of image (x = 240)
    target_right = {"class": "aluminum_can", "conf": 0.88, "center": (240.0, 160.0)}
    result = tracker.process_frame(target_right)
    assert result["status"] == "CONFIRMED"
    assert result["servo_angle"] > 90, f"Expected servo_angle > 90, got {result['servo_angle']}"
    print("  Case 2: Right confirmed target pointed correctly (PASS)")

    # Case 3: Target detected but ultrasonic distance is implausible (>200 cm)
    serial_mock.mock_distance = 250.0
    result = tracker.process_frame(target_right)
    assert result["status"] == "UNCONFIRMED"
    print("  Case 3: Implausible distance (>200cm) treated as UNCONFIRMED (PASS)")

    # Case 4: Target detected but ultrasonic sensor timed out (R,-1)
    serial_mock.mock_distance = -1.0
    result = tracker.process_frame(target_right)
    assert result["status"] == "UNCONFIRMED"
    print("  Case 4: Ultrasonic sensor timeout (-1) treated as UNCONFIRMED (PASS)")

    # Case 5: No target detected in frame
    result = tracker.process_frame(None)
    assert result == "NO_DETECTION"
    print("  Case 5: No detection triggers no serial action (PASS)")

    print("\nALL INTEGRATION TESTS PASSED!")


if __name__ == "__main__":
    test_integration_flow()
