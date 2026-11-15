#!/usr/bin/env python3

import argparse
import math
import time

import numpy as np
import rclpy
from ferrari_vehicle.msg import OdomLite
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

HALL = (1.75, -4.5, 15.5, 21.0)
BOXES = [(-4.5, -3.0, 1.0, 1.0), (8.0, -8.0, 1.2, 1.2), (1.0, -13.5, 1.5, 1.0), (7.0, 3.5, 1.0, 1.0), (-4.0, -10.0, 1.0, 1.6)]


def rectangle_points(cx, cy, w, h, step=0.01):
    corners = [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2), (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]
    points = []
    for i in range(4):
        (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % 4]
        count = int(math.hypot(x2 - x1, y2 - y1) / step) + 1
        points += [(x1 + (x2 - x1) * k / count, y1 + (y2 - y1) * k / count) for k in range(count + 1)]
    return np.array(points)


def min_distances(sources, targets, chunk=500):
    result = []
    for i in range(0, len(sources), chunk):
        block = sources[i : i + chunk]
        result.append(np.sqrt(((block[:, None, :] - targets[None, :, :]) ** 2).sum(-1)).min(1))
    return np.concatenate(result)


def main():
    parser = argparse.ArgumentParser(description="Compare a SLAM output against the twin ground truth")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--map-topic", default="/mapping/map")
    parser.add_argument("--sim-time", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    overrides = [Parameter("use_sim_time", value=True)] if args.sim_time else []
    node = Node("evaluate_slam", parameter_overrides=overrides)
    buffer = Buffer()
    TransformListener(buffer, node)

    truth = {}

    def on_truth(msg):
        truth["pose"] = (msg.pose.position.x, msg.pose.position.y, Time.from_msg(msg.header.stamp))

    node.create_subscription(OdomLite, "/vehicle/ground_truth", on_truth, 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, args.map_topic, lambda m: truth.__setitem__("map", m), latched)

    rows = []
    start = time.time()
    last = 0.0
    while time.time() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last < 0.2 or "pose" not in truth:
            continue
        last = time.time()
        tx, ty, stamp = truth["pose"]
        try:
            slam = buffer.lookup_transform("map", "base_link", stamp).transform.translation
            odom = buffer.lookup_transform("odom", "base_link", stamp).transform.translation
        except Exception:
            continue
        rows.append((math.hypot(slam.x - tx, slam.y - ty), math.hypot(odom.x - tx, odom.y - ty)))

    if rows:
        slam_err = np.array([r[0] for r in rows])
        odom_err = np.array([r[1] for r in rows])
        print(f"samples {len(rows)}")
        print(f"SLAM  error [m]: mean {slam_err.mean():.3f} max {slam_err.max():.3f} final {slam_err[-1]:.3f}")
        print(f"ODOM  error [m]: mean {odom_err.mean():.3f} max {odom_err.max():.3f} final {odom_err[-1]:.3f}")

    grid = truth.get("map")
    if grid is not None:
        w, h, res = grid.info.width, grid.info.height, grid.info.resolution
        data = np.array(grid.data, dtype=np.int8).reshape(h, w)
        iy, ix = np.where(data >= 65)
        cells = np.stack([grid.info.origin.position.x + (ix + 0.5) * res, grid.info.origin.position.y + (iy + 0.5) * res], 1)
        walls = np.vstack([rectangle_points(*HALL)] + [rectangle_points(*b) for b in BOXES])
        to_wall = min_distances(cells, walls)
        coverage = min_distances(walls, cells)
        print(
            f"map: {len(cells)} occupied cells, distance to true wall median {np.median(to_wall) * 100:.1f} cm, "
            f"p95 {np.percentile(to_wall, 95) * 100:.1f} cm, within 10 cm {100 * (to_wall < 0.1).mean():.1f}%, "
            f"wall coverage {100 * (coverage < 0.1).mean():.1f}%"
        )

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
