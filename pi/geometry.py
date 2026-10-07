"""
geometry.py - Geometric, angular, and distance transformations for the waste pointing system.
"""

# Camera & Mechanical Defaults
HORIZONTAL_FOV = 62.2       # Pi Camera V2 horizontal field of view in degrees
SERVO_DIRECTION = 1         # Multiplier: +1 or -1 for physical servo orientation
DEADBAND_DEG = 3.0          # Angular deadband for "center" stability classification in degrees

# Camera Focal Length Calibration (Pinhole model for 320x320 frame)
# Formula: focal_length_px = (pixel_height * distance_cm) / real_height_cm
# Measured across two known distances (e.g. 30cm and 100cm) and averaged.
# Default baseline placeholder for Pi Camera V2:
FOCAL_LENGTH_PX = 320.0

# Approximate real heights in cm for structured 3D waste objects.
# Irregular/flat objects without consistent vertical profile (paper, cardboard,
# plastic bag, pop tab, plastic) return None / 'unknown'.
CLASS_REAL_HEIGHTS_CM = {
    "plastic bottle": 20.0,
    "can": 12.0,
    "glass bottle": 25.0,
    "drink carton": 15.0,
    "battery": 5.0,
}


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


def determine_direction(bearing: float, deadband: float = DEADBAND_DEG) -> str:
    """
    Categorizes the bearing angle into "left", "right", or "center"
    based on the specified deadband threshold.
    """
    if bearing < -deadband:
        return "left"
    elif bearing > deadband:
        return "right"
    else:
        return "center"


def estimate_distance_from_bbox(
    class_name: str,
    pixel_height: float,
    focal_length_px: float = FOCAL_LENGTH_PX,
) -> float | None:
    """
    Estimates distance in cm using pinhole camera model based on bounding box pixel height.

    Formula:
        distance_estimate = (real_height * focal_length_px) / pixel_height

    For flat/irregular classes with no consistent height (cardboard, plastic bag,
    paper, pop tab, plastic), returns None.
    """
    if pixel_height <= 0 or focal_length_px <= 0:
        return None

    normalized_class = class_name.lower().strip()
    real_height_cm = CLASS_REAL_HEIGHTS_CM.get(normalized_class)

    if real_height_cm is None:
        return None

    return (real_height_cm * focal_length_px) / float(pixel_height)


def calibrate_focal_length(
    measured_pixel_height_1: float,
    distance_cm_1: float,
    real_height_cm_1: float,
    measured_pixel_height_2: float,
    distance_cm_2: float,
    real_height_cm_2: float,
) -> tuple[float, float, float, float]:
    """
    Calibrates camera focal length in pixels using a reference object at two known distances.

    Formula:
        focal_length_px = (pixel_height * distance_cm) / real_height_cm

    Returns:
        tuple: (f1, f2, average_f, difference_percentage)
    """
    f1 = (measured_pixel_height_1 * distance_cm_1) / real_height_cm_1
    f2 = (measured_pixel_height_2 * distance_cm_2) / real_height_cm_2
    avg = (f1 + f2) / 2.0
    diff_pct = abs(f1 - f2) / avg * 100.0 if avg > 0 else 0.0

    return f1, f2, avg, diff_pct
