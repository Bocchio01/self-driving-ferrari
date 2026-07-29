#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import csv
import os


class MockGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("mock_global_planner_node")
        CSV_BASE_PATH = "/home/bocchio/Documents/github/self-driving-ferrari/software/src/ferrari_planning/scripts/f1tenth_racetracks"

        self.scale = self.declare_parameter("scale", 45.0).value
        track_name = self.declare_parameter("track_name", "Monza").value

        self.poses = []
        self.publisher_ = self.create_publisher(Path, "global_path", 10)

        self.get_logger().info(f"Loading track from {track_name}...")
        self.load_track(f"{CSV_BASE_PATH}/{track_name}/{track_name}_raceline.csv")

        # Set up a timer to publish the path every 5 seconds
        self.create_timer(5.0, self.publish_path)

    def load_track(self, file_path):
        """
        Loads the F1Tenth raceline format:
        s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
        """

        def create_pose(x, y, theta):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            # Convert yaw to quaternion
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)

            return pose

        if not os.path.exists(file_path):
            self.get_logger().error(f"Track file not found: {file_path}")
            return

        with open(file_path, mode="r") as file:
            reader = csv.reader(file, delimiter=";")

            for row in reader:
                if not row or row[0].startswith("#"):
                    continue

                try:
                    x = float(row[1]) / self.scale
                    y = float(row[2]) / self.scale
                    theta = float(row[3])

                    pose = create_pose(x, y, theta)
                    self.poses.append(pose)

                except (ValueError, IndexError) as e:
                    self.get_logger().warning(f"Skipping malformed row: {row}")
                    continue

    def publish_path(self):
        """Constructs and latches the Path message."""
        if not self.poses:
            self.get_logger().warning("No path generated to publish!")
            return

        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self.poses

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockGlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
