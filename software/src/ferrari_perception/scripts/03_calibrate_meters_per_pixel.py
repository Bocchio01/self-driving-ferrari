#!/usr/bin/env python3
"""Step 3: calculate meters per pixel on the warped view.

Click exactly two points on the warped image that represent a known distance
in the real world.

Controls:
  left-click   place a point (up to 2)
  r            reset points
  x            calculate and save meters_per_pixel_x (horizontal distance)
  y            calculate and save meters_per_pixel_y (vertical distance)
  q            quit

Usage:
  python3 03_calibrate_meters_per_pixel.py --source 0
"""

import argparse
import math
import cv2
import numpy as np

from calib_utils import FrameSource, load_config, save_config, warp_frame

points = []


def on_mouse(event, x, y, flags, _param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
        points.append((x, y))


def draw_overlay(frame):
    overlay = frame.copy()
    for i, p in enumerate(points):
        cv2.circle(overlay, p, 5, (0, 0, 255), -1)
    if len(points) == 2:
        cv2.line(overlay, points[0], points[1], (0, 255, 255), 2)
    return overlay


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", default="0", help="camera index, video path, or image path"
    )
    parser.add_argument("--config", default="calibration.yaml")
    args = parser.parse_args()

    config = load_config(args.config)
    src = FrameSource(args.source)

    window = "03 - meters per pixel (click 2 points)"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    print(__doc__)

    while True:
        frame = src.read()
        warped = warp_frame(frame, config)

        cv2.imshow(window, draw_overlay(warped))
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("r"):
            points.clear()
        elif key in [ord("x"), ord("y")]:
            if len(points) != 2:
                print("Need exactly 2 points placed.")
                continue

            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
            pixel_dist = abs(dx) if key == ord("x") else abs(dy)

            if pixel_dist == 0:
                print("Error: Distance is 0 pixels along that axis.")
                continue

            axis = "X" if key == ord("x") else "Y"
            try:
                meters = float(
                    input(
                        f"Enter real-world {axis} distance in METERS between these points: "
                    )
                )
                mpp = meters / pixel_dist

                if key == ord("x"):
                    config["meters_per_pixel_x"] = float(round(mpp, 6))
                else:
                    config["meters_per_pixel_y"] = float(round(mpp, 6))

                save_config(config, args.config)
                print(f"Calculated meters_per_pixel_{axis.lower()}: {mpp:.6f}")
                points.clear()
            except ValueError:
                print("Invalid input. Please enter a number.")

    src.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
