#!/usr/bin/env python3
"""Step 4: Sanity check the entire pipeline.

Reads frames, warps them, thresholds HSV, isolates the largest component,
and calculates the line centers using sliding windows.

Controls:
  q            quit

Usage:
  python3 04_preview_full_pipeline.py --source 0
"""

import argparse
import cv2
import numpy as np

from calib_utils import (
    FrameSource,
    keep_best_line_component,
    load_config,
    warp_frame,
    threshold_line,
    find_line_centers,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", default="0", help="camera index, video path, or image path"
    )
    parser.add_argument("--config", default="calibration.yaml")
    args = parser.parse_args()

    config = load_config(args.config)
    src = FrameSource(args.source)

    window = "04 - Full Pipeline Preview"

    print(__doc__)

    while True:
        frame = src.read()

        # 1. Warp perspective
        warped = warp_frame(frame, config)

        # 2. Threshold HSV
        binary = threshold_line(warped, config["hsv_threshold"])

        # 3. Filter noise
        clean_binary = keep_best_line_component(binary)

        # 4. Find line centers
        centers = find_line_centers(binary, config["num_windows"])

        # Draw the pipeline results on the warped frame
        output_vis = warped.copy()

        # Draw sliding window horizontal lines for context
        h = output_vis.shape[0]
        window_height = h // config["num_windows"]
        for i in range(1, config["num_windows"]):
            y_line = h - (i * window_height)
            cv2.line(
                output_vis,
                (0, y_line),
                (output_vis.shape[1], y_line),
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        # Draw the calculated centers
        for i in range(len(centers) - 1):
            pt1 = (int(centers[i][0]), int(centers[i][1]))
            pt2 = (int(centers[i + 1][0]), int(centers[i + 1][1]))
            cv2.line(output_vis, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(output_vis, pt1, 4, (0, 255, 255), -1)

        if len(centers) > 0:
            last_pt = (int(centers[-1][0]), int(centers[-1][1]))
            cv2.circle(output_vis, last_pt, 4, (0, 255, 255), -1)

        # Show final binary mask side-by-side with drawn targets
        bgr_clean = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        combined = cv2.hconcat([bgr_clean, output_vis])

        cv2.imshow(window, combined)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break

    src.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
