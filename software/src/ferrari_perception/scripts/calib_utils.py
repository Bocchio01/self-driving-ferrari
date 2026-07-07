"""Shared helpers for the ferrari_perception camera calibration scripts.

These scripts run standalone (no ROS2 needed) against a webcam, a video
file, or a single still image, so you can iterate on calibration without
rebuilding/relaunching the perception stack.

All scripts read from and write to a shared `calibration.yaml`, formatted
as a ROS2 params file so you can pass it straight to line_detector_node
with `--params-file calibration.yaml`.

Suggested run order:
  1. python3 01_calibrate_warp_points.py
  2. python3 02_calibrate_hsv.py
  3. python3 03_calibrate_meters_per_pixel.py
  4. python3 04_preview_full_pipeline.py   (sanity check everything together)

Requirements: pip install opencv-python numpy pyyaml
"""

from pathlib import Path

import cv2
import numpy as np
import yaml

DEFAULT_CONFIG_PATH = Path("..", "config", "calibration.yaml")
NODE_NAMESPACE = "line_detector_node"

DEFAULT_CONFIG = {
    "warp_src_points_norm": [0.35, 0.55, 0.65, 0.55, 1.00, 1.00, 0.00, 1.00],
    "warp_dst_points_norm": [0.20, 0.0, 0.80, 0.0, 0.80, 1.0, 0.20, 1.0],
    "warped_width": 400,
    "warped_height": 600,
    "hsv_threshold": [0, 179, 0, 255, 200, 255],
    "meters_per_pixel_x": 0.003,
    "meters_per_pixel_y": 0.004,
    "num_windows": 10,
}


def load_config(path=DEFAULT_CONFIG_PATH):
    """Load calibration.yaml if it exists, falling back to defaults for any
    missing keys. Understands both a plain flat YAML and a ROS2 params file
    (with the `line_detector: ros__parameters:` wrapper)."""
    path = Path(path)
    config = dict(DEFAULT_CONFIG)
    if path.exists():
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        if NODE_NAMESPACE in data:
            data = data[NODE_NAMESPACE]["ros__parameters"]
        config.update(data)
    return config


def save_config(config, path=DEFAULT_CONFIG_PATH):
    doc = {NODE_NAMESPACE: {"ros__parameters": config}}
    with open(path, "w") as f:
        yaml.dump(doc, f, default_flow_style=None, sort_keys=False)
    print(f"\nSaved calibration to {Path(path).resolve()}\n")


class FrameSource:
    """Reads frames from a webcam/video device or a video file. If a still
    image path is given instead, repeats that same frame on every read() —
    handy for tuning against a fixed reference photo."""

    IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}

    def __init__(self, source):
        self._static_frame = None
        self._cap = None
        suffix = Path(str(source)).suffix.lower()
        if suffix in self.IMAGE_EXTS:
            frame = cv2.imread(str(source))
            if frame is None:
                raise RuntimeError(f"Could not read image: {source}")
            self._static_frame = frame
        else:
            try:
                source = int(source)
            except ValueError:
                pass
            self._cap = cv2.VideoCapture(source)
            if not self._cap.isOpened():
                raise RuntimeError(f"Could not open video source: {source}")

    def read(self):
        if self._static_frame is not None:
            return self._static_frame.copy()
        ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError("Failed to read frame from source")
        return frame

    def release(self):
        if self._cap is not None:
            self._cap.release()


def compute_warp_matrix(config, frame_size):
    """frame_size is (width, height) of the RAW frame being warped."""
    w, h = frame_size
    src_norm = config["warp_src_points_norm"]
    src = np.float32([[src_norm[2 * i] * w, src_norm[2 * i + 1] * h] for i in range(4)])

    ww, wh = config["warped_width"], config["warped_height"]
    dst_norm = config["warp_dst_points_norm"]
    dst = np.float32(
        [[dst_norm[2 * i] * ww, dst_norm[2 * i + 1] * wh] for i in range(4)]
    )

    matrix = cv2.getPerspectiveTransform(src, dst)
    return matrix, (ww, wh)


def warp_frame(frame, config):
    matrix, size = compute_warp_matrix(config, (frame.shape[1], frame.shape[0]))
    return cv2.warpPerspective(frame, matrix, size)


def threshold_line(frame_bgr, hsv_threshold):
    """Mirrors LineDetectorNode::colorThreshold."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = (hsv_threshold[0], hsv_threshold[2], hsv_threshold[4])
    upper = (hsv_threshold[1], hsv_threshold[3], hsv_threshold[5])
    binary = cv2.inRange(hsv, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary


def keep_best_line_component(binary, min_area=50):
    """
    Replaces keep_largest_component.
    Keeps the component that best represents a line close to the camera by
    scoring based on aspect ratio (linearity) and lowest Y-coordinate (proximity).
    """
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary, connectivity=8
    )

    # If there's only background, return the mask as-is
    if num_labels <= 1:
        return binary

    img_height = binary.shape[0]
    best_label = 0
    best_score = -1

    # Start at 1 to skip the background (label 0)
    for label in range(1, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]

        # 1. Filter out tiny specks of noise so they don't skew the logic
        if area < min_area:
            continue

        # 2. Calculate Proximity (0.0 to 1.0)
        # CC_STAT_TOP + CC_STAT_HEIGHT gives the bottom-most Y coordinate of the blob
        bottom_y = stats[label, cv2.CC_STAT_TOP] + stats[label, cv2.CC_STAT_HEIGHT]
        proximity = bottom_y / img_height

        # 3. Calculate Linearity (Aspect Ratio)
        # We isolate the blob and find its contour to fit a rotated bounding box.
        # This handles diagonal lines perfectly, unlike standard bounding boxes.
        component_mask = (labels == label).astype(np.uint8)
        contours, _ = cv2.findContours(
            component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            continue

        # minAreaRect returns: (center(x, y), (width, height), angle of rotation)
        _, (rect_w, rect_h), _ = cv2.minAreaRect(contours[0])

        # Avoid division by zero
        if min(rect_w, rect_h) == 0:
            aspect_ratio = 1.0
        else:
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)

        # 4. Calculate Final Score
        # We multiply them so a component needs to be reasonably line-like AND close.
        # Note: We square the proximity (proximity ** 2) to heavily bias the robot
        # toward lines immediately in front of it, rather than long lines far away.
        score = aspect_ratio * (proximity**2)

        if score > best_score:
            best_score = score
            best_label = label

    # If everything was filtered out (e.g., it was all tiny noise), return a blank mask
    if best_label == 0:
        return np.zeros_like(binary)

    # Isolate and return the winning component
    return np.where(labels == best_label, 255, 0).astype(np.uint8)


def find_line_centers(binary, num_windows):
    """Mirrors LineDetectorNode::findLineCenter (bottom-to-top horizontal bands)."""
    h = binary.shape[0]
    window_height = h // num_windows
    points = []
    current_y = h
    for _ in range(num_windows):
        y_start = max(0, current_y - window_height)
        y_end = current_y
        band = binary[y_start:y_end, :]
        ys, xs = np.nonzero(band)
        if len(xs) > 0:
            mean_x = float(np.mean(xs))
            mean_y = float(np.mean(ys)) + y_start
            points.append((mean_x, mean_y))
        current_y -= window_height
    return points
