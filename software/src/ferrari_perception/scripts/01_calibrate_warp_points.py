#!/usr/bin/env python3
"""Step 1: pick the bird's-eye warp source trapezoid.

Click the four corners of a rectangle of floor, IN THIS ORDER:
  1. top-left     2. top-right
  4. bottom-left  3. bottom-right

The "top" points are the further-away, narrower end of the trapezoid (where
parallel lines appear to converge toward the horizon in your cockpit view);
the "bottom" points are the near corners, closest to the bumper.

Controls:
  left-click   place the next point (up to 4)
  r            reset points on the current frame
  f            grab a fresh frame from the source (also resets points)
  p            preview the resulting bird's-eye warp (needs 4 points)
  s            save warp_src_points_norm to calibration.yaml
  q            quit

Usage:
  python3 01_calibrate_warp_points.py --source 0
  python3 01_calibrate_warp_points.py --source /path/to/reference_photo.jpg
"""

import argparse

import cv2
import numpy as np

from calib_utils import DEFAULT_CONFIG_PATH, FrameSource, load_config, save_config

points = []
LABELS = ["TL", "TR", "BR", "BL"]


def on_mouse(event, x, y, flags, _param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((x, y))


def draw_overlay(frame):
    overlay = frame.copy()
    for i, p in enumerate(points):
        cv2.circle(overlay, p, 6, (0, 255, 0), -1)
        cv2.putText(
            overlay,
            f"{i + 1}:{LABELS[i]}",
            (p[0] + 8, p[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    if len(points) > 1:
        cv2.polylines(
            overlay,
            [np.array(points, dtype=np.int32)],
            isClosed=len(points) == 4,
            color=(0, 200, 255),
            thickness=2,
        )
    cv2.putText(
        overlay,
        f"points placed: {len(points)}/4",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    return overlay


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", default="0", help="camera index, video path, or image path"
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    args = parser.parse_args()

    config = load_config(args.config)
    src = FrameSource(args.source)
    frame = src.read()

    window = "01 - warp points (click TL, TR, BR, BL)"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    print(__doc__)

    while True:
        cv2.imshow(window, draw_overlay(frame))
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("r"):
            points.clear()
        elif key == ord("f"):
            frame = src.read()
            points.clear()
        elif key == ord("p"):
            if len(points) != 4:
                print("Need exactly 4 points before previewing.")
                continue
            h, w = frame.shape[:2]
            src_pts = np.float32(points)
            ww, wh = config["warped_width"], config["warped_height"]
            dst_norm = config["warp_dst_points_norm"]
            dst_pts = np.float32(
                [[dst_norm[2 * i] * ww, dst_norm[2 * i + 1] * wh] for i in range(4)]
            )
            matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            warped = cv2.warpPerspective(frame, matrix, (ww, wh))
            cv2.imshow("bird's-eye preview", warped)
        elif key == ord("s"):
            if len(points) != 4:
                print("Need exactly 4 points before saving.")
                continue
            h, w = frame.shape[:2]
            norm = []
            for x, y in points:
                norm.extend([x / w, y / h])
            config["warp_src_points_norm"] = norm
            save_config(config, args.config)
            print("warp_src_points_norm:", [round(v, 4) for v in norm])

    src.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
