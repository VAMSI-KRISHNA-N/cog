"""
test_integration.py - Integration test verifying continuous tracking and stability-gated ultrasonic confirmation.
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
    DEADBAND_DEG,
    FOCAL_LENGTH_PX,
)


class MockContinuousTracker:
    def __init__(
        self,
        serial_iface,
        stable_threshold=5,
        deadband=DEADBAND_DEG,
        fov=62.2,
        servo_direction=1,
        min_dist=2.0,
        max_dist=200.0,
    ):
        self.serial_iface = serial_iface
        self.stable_threshold = stable_threshold
        self.deadband = deadband
        self.fov = fov
        self.servo_direction = servo_direction
        self.min_dist = min_dist
        self.max_dist = max_dist

        # Tracker state
        self.last_servo_angle = 90
        self.consecutive_stable_frames = 0
        self.is_confirmed = False
        self.last_confirmed_dist = None
        self.servo_commands_sent = []
        self.ultrasonic_queries_count = 0

    def process_frame(self, target, img_width=320):
        if target is None:
            self.consecutive_stable_frames = 0
            self.is_confirmed = False
            self.last_confirmed_dist = None
            return {
                "status": "not waste — no target",
                "holding_angle": self.last_servo_angle,
                "servo_sent": None,
                "confirmed": False,
            }

        x_center, _ = target["center"]
        bearing, servo_angle = calculate_bearing_and_angle(
            x_center=x_center,
            img_width=img_width,
            fov=self.fov,
            servo_direction=self.servo_direction,
        )
        direction = determine_direction(bearing, self.deadband)
        dist_est = estimate_distance_from_bbox(target["class"], target.get("pixel_height", 50))

        # Continuous tracking: Servo sends update on every qualifying frame
        self.serial_iface.send_point(servo_angle)
        self.servo_commands_sent.append(servo_angle)
        self.last_servo_angle = servo_angle

        # Stability counter and gated ultrasonic confirmation
        if direction == "center":
            self.consecutive_stable_frames += 1
            if self.consecutive_stable_frames >= self.stable_threshold and not self.is_confirmed:
                self.ultrasonic_queries_count += 1
                dist_cm = self.serial_iface.query_distance()
                if self.min_dist <= dist_cm <= self.max_dist:
                    self.is_confirmed = True
                    self.last_confirmed_dist = dist_cm
        else:
            self.consecutive_stable_frames = 0
            self.is_confirmed = False
            self.last_confirmed_dist = None

        return {
            "status": "TRACKING" if not self.is_confirmed else "CONFIRMED",
            "class": target["class"],
            "bearing": bearing,
            "direction": direction,
            "servo_sent": servo_angle,
            "stable_count": self.consecutive_stable_frames,
            "confirmed": self.is_confirmed,
            "confirmed_dist": self.last_confirmed_dist,
            "dist_est": dist_est,
        }


def test_continuous_tracking_flow():
    print("[TEST] Running Continuous Tracking & Stability-Gated Confirmation tests...")

    serial_mock = ArduinoSerialInterface(mock=True, mock_distance=42.0)
    tracker = MockContinuousTracker(serial_mock, stable_threshold=5, deadband=3.0)

    # 1. Target detected on Left (x = 80 -> bearing ~ -15.5°)
    target_left = {"class": "plastic bottle", "conf": 0.88, "center": (80.0, 160.0), "pixel_height": 100.0}
    res1 = tracker.process_frame(target_left)
    assert res1["direction"] == "left"
    assert res1["servo_sent"] < 90
    assert res1["stable_count"] == 0
    assert tracker.ultrasonic_queries_count == 0, "Ultrasonic must NOT fire while moving off-center"
    print("  Case 1: Left target tracked continuously, ultrasonic suppressed (PASS)")

    # 2. Target moves to Right (x = 240 -> bearing ~ +15.5°)
    target_right = {"class": "can", "conf": 0.90, "center": (240.0, 160.0), "pixel_height": 80.0}
    res2 = tracker.process_frame(target_right)
    assert res2["direction"] == "right"
    assert res2["servo_sent"] > 90
    assert res2["stable_count"] == 0
    assert tracker.ultrasonic_queries_count == 0
    print("  Case 2: Right target tracked continuously, ultrasonic suppressed (PASS)")

    # 3. Target centers and holds steady (x = 160 -> bearing 0.0°)
    target_center = {"class": "plastic bottle", "conf": 0.94, "center": (160.0, 160.0), "pixel_height": 150.0}

    # Frames 1 through 4: stability counter increments, but NO ultrasonic query yet
    for f in range(1, 5):
        res = tracker.process_frame(target_center)
        assert res["direction"] == "center"
        assert res["stable_count"] == f
        assert res["confirmed"] is False
        assert tracker.ultrasonic_queries_count == 0

    print("  Case 3: Frames 1-4 increment stability counter without polling ultrasonic (PASS)")

    # Frame 5: Reaches stable_threshold (5) -> triggers ONE ultrasonic confirmation!
    res_stable = tracker.process_frame(target_center)
    assert res_stable["stable_count"] == 5
    assert res_stable["confirmed"] is True
    assert res_stable["confirmed_dist"] == 42.0
    assert tracker.ultrasonic_queries_count == 1
    print("  Case 4: Frame 5 triggers ultrasonic confirmation query on steady lock (PASS)")

    # Frame 6 & 7: Stays steady. Must NOT trigger repeated ultrasonic queries!
    tracker.process_frame(target_center)
    tracker.process_frame(target_center)
    assert tracker.ultrasonic_queries_count == 1, "Ultrasonic must not poll repeatedly once confirmed"
    print("  Case 5: Subsequent steady frames do not re-poll ultrasonic (PASS)")

    # 4. Target shifts out of deadband (moves to x = 200, bearing ~ +7.8°)
    target_drift = {"class": "plastic bottle", "conf": 0.90, "center": (200.0, 160.0), "pixel_height": 150.0}
    res_drift = tracker.process_frame(target_drift)
    assert res_drift["direction"] == "right"
    assert res_drift["stable_count"] == 0
    assert res_drift["confirmed"] is False
    print("  Case 6: Target drift resets stability counter and unconfirms lock (PASS)")

    # 5. Target lost (None)
    res_lost = tracker.process_frame(None)
    assert res_lost["status"] == "not waste — no target"
    assert res_lost["holding_angle"] == tracker.last_servo_angle
    assert res_lost["servo_sent"] is None, "Must not send neutral angle on missed target"
    print("  Case 7: Missed frame holds last servo position without snapping to neutral (PASS)")

    print("\nALL CONTINUOUS TRACKING INTEGRATION TESTS PASSED!")


if __name__ == "__main__":
    test_continuous_tracking_flow()
