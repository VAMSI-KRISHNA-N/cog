"""
generate_test_video.py - Generates a synthetic test video for perception pipeline validation.
"""

import cv2
import numpy as np

def generate_synthetic_video(output_path="tests/test_feed.mp4", num_frames=30, width=320, height=320):
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, 10.0, (width, height))

    for i in range(num_frames):
        frame = np.ones((height, width, 3), dtype=np.uint8) * 240
        # Draw a moving dark blue square/box simulating an object moving from left (x=40) to right (x=280)
        center_x = int(40 + (240.0 * i / (num_frames - 1)))
        center_y = height // 2
        box_w, box_h = 50, 70

        cv2.rectangle(
            frame,
            (center_x - box_w // 2, center_y - box_h // 2),
            (center_x + box_w // 2, center_y + box_h // 2),
            (180, 50, 50),
            -1
        )
        # Add label
        cv2.putText(frame, "Target", (center_x - 20, center_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        out.write(frame)

    out.release()
    print(f"Synthetic video generated at {output_path}")

if __name__ == "__main__":
    generate_synthetic_video()
