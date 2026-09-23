"""
geometry.py - Geometric and angular transformations for the waste pointing system.
"""

# Camera & Mechanical Defaults
HORIZONTAL_FOV = 62.2       # Pi Camera V2 horizontal field of view in degrees
SERVO_DIRECTION = 1         # Multiplier: +1 or -1 for physical servo orientation


def calculate_bearing_and_angle(
    x_center: float,
    img_width: float,
    fov: float = HORIZONTAL_FOV,
    servo_direction: int = SERVO_DIRECTION,
) -> tuple[float, int]:
    """
    Computes bearing angle relative to optical center and corresponding servo angle.

    Formula:
        normalized_offset = (x_center / img_width) - 0.5
        bearing = normalized_offset * fov
        servo_angle = 90 + (SERVO_DIRECTION * bearing), clamped to [0, 180]

    Args:
        x_center: Horizontal pixel coordinate of target bounding box center.
        img_width: Width of image frame in pixels.
        fov: Camera horizontal field of view in degrees.
        servo_direction: Direction multiplier (+1 or -1) to accommodate physical mounting.

    Returns:
        tuple (bearing_deg, servo_angle_deg)
    """
    if img_width <= 0:
        raise ValueError("Image width must be positive.")

    normalized_offset = (x_center / float(img_width)) - 0.5
    bearing = normalized_offset * float(fov)

    raw_servo_angle = 90.0 + (servo_direction * bearing)
    servo_angle = int(round(raw_servo_angle))
    clamped_servo_angle = max(0, min(180, servo_angle))

    return bearing, clamped_servo_angle
