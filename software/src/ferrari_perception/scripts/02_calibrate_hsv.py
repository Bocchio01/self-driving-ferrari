#!/usr/bin/env python3
"""Step 2: tune the HSV thresholding on the warped bird's-eye view.

This isolates the line (e.g., a white or yellow strip) from the floor.
Use the trackbars to adjust the Min and Max bounds for Hue, Saturation,
and Value until the target line is mostly white and the background is black.

Controls:
  s            save the current hsv_threshold to calibration.yaml
  q            quit

Usage:
  python3 02_calibrate_hsv.py --source 0
  python3 02_calibrate_hsv.py --source /path/to/reference_photo.jpg
"""

import argparse
import cv2

from calib_utils import (
    FrameSource,
    load_config,
    save_config,
    warp_frame,
    threshold_line,
    DEFAULT_CONFIG_PATH,
)


def empty_callback(_):
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", default="0", help="camera index, video path, or image path"
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    args = parser.parse_args()

    config = load_config(args.config)
    src = FrameSource(args.source)

    window_name = "02 - HSV Calibration"
    cv2.namedWindow(window_name)

    # Load initial thresholds
    hsv = config["hsv_threshold"]

    cv2.createTrackbar("H Min", window_name, hsv[0], 179, empty_callback)
    cv2.createTrackbar("H Max", window_name, hsv[1], 179, empty_callback)
    cv2.createTrackbar("S Min", window_name, hsv[2], 255, empty_callback)
    cv2.createTrackbar("S Max", window_name, hsv[3], 255, empty_callback)
    cv2.createTrackbar("V Min", window_name, hsv[4], 255, empty_callback)
    cv2.createTrackbar("V Max", window_name, hsv[5], 255, empty_callback)

    print(__doc__)

    while True:
        frame = src.read()
        warped = warp_frame(frame, config)

        # Read current trackbar positions
        current_hsv = [
            cv2.getTrackbarPos("H Min", window_name),
            cv2.getTrackbarPos("H Max", window_name),
            cv2.getTrackbarPos("S Min", window_name),
            cv2.getTrackbarPos("S Max", window_name),
            cv2.getTrackbarPos("V Min", window_name),
            cv2.getTrackbarPos("V Max", window_name),
        ]

        # Apply threshold
        binary = threshold_line(warped, current_hsv)

        # Display results (side-by-side)
        bgr_binary = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        combined = cv2.hconcat([warped, bgr_binary])

        cv2.imshow(window_name, combined)
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s"):
            config["hsv_threshold"] = current_hsv
            save_config(config, args.config)
            print(f"Saved HSV thresholds: {current_hsv}")

    src.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
