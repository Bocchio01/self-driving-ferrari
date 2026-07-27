#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# Import the new custom trajectory message
from ferrari_planning.msg import Trajectory

import matplotlib.pyplot as plt
import numpy as np
import math
from collections import deque


class ControllerVisualizer(Node):
    def __init__(self):
        super().__init__("controller_visualizer")

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.traj_sub = self.create_subscription(
            Trajectory, "local_trajectory", self.trajectory_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped, "ackermann_cmd", self.cmd_callback, 10
        )

        # Stored vehicle state
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0

        # Time tracking for the two different frequencies
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.cmd_time = deque(maxlen=500)
        self.err_time = deque(maxlen=500)

        # Control efforts
        self.vel = deque(maxlen=500)
        self.steer = deque(maxlen=500)

        # Tracking errors (Frenet frame and Yaw)
        self.err_lon = deque(maxlen=500)
        self.err_lat = deque(maxlen=500)
        self.err_yaw = deque(maxlen=500)

        # Matplotlib setup (2x2 grid for clear separation of units)
        plt.ion()
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle("Controller Performance", fontsize=16)

        # 1. Frenet Error Plot (Longitudinal & Lateral)
        self.ax_err_frenet = self.axs[0, 0]
        (self.plot_err_lon,) = self.ax_err_frenet.plot(
            [], [], "r-", label="Longitudinal"
        )
        (self.plot_err_lat,) = self.ax_err_frenet.plot([], [], "b-", label="Lateral")
        self.ax_err_frenet.set_title("Position Error (Frenet)")
        self.ax_err_frenet.set_ylabel("Error [m]")
        self.ax_err_frenet.grid(True)
        self.ax_err_frenet.legend()

        # 2. Yaw Error Plot
        self.ax_err_yaw = self.axs[0, 1]
        (self.plot_err_yaw,) = self.ax_err_yaw.plot([], [], "g-", label="Yaw Error")
        self.ax_err_yaw.set_title("Heading Error")
        self.ax_err_yaw.set_ylabel("Error [rad]")
        self.ax_err_yaw.grid(True)
        self.ax_err_yaw.legend()

        # 3. Commanded Speed
        self.ax_cmd_vel = self.axs[1, 0]
        (self.plot_vel,) = self.ax_cmd_vel.plot([], [], "m-", label="Speed")
        self.ax_cmd_vel.set_title("Control Effort: Speed")
        self.ax_cmd_vel.set_xlabel("Time [s]")
        self.ax_cmd_vel.set_ylabel("Velocity [m/s]")
        self.ax_cmd_vel.grid(True)
        self.ax_cmd_vel.legend()

        # 4. Commanded Steering
        self.ax_cmd_steer = self.axs[1, 1]
        (self.plot_steer,) = self.ax_cmd_steer.plot([], [], "c-", label="Steering")
        self.ax_cmd_steer.set_title("Control Effort: Steering")
        self.ax_cmd_steer.set_xlabel("Time [s]")
        self.ax_cmd_steer.set_ylabel("Angle [rad]")
        self.ax_cmd_steer.grid(True)
        self.ax_cmd_steer.legend()

        plt.tight_layout()

        self.timer = self.create_timer(0.5, self.update_plot)

    # ----------------------------
    # ROS callbacks
    # ----------------------------

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.car_yaw = math.atan2(siny, cosy)

    def trajectory_callback(self, msg):
        # We only compute error if the trajectory has points
        if len(msg.points) > 0:
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.err_time.append(current_time)

            # Use the first point in the trajectory as the current reference
            ref_pose = msg.points[0].pose

            # Global Position Error (Reference - Actual)
            dx = ref_pose.position.x - self.car_x
            dy = ref_pose.position.y - self.car_y

            # Reference heading (yaw_ref)
            q = ref_pose.orientation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw_ref = math.atan2(siny, cosy)

            # Transform to Frenet Frame using a rotation matrix based on the path's heading
            # Positive lateral error = Reference is to the left of the car
            # Positive longitudinal error = Reference is ahead of the car
            lon_error = dx * math.cos(yaw_ref) + dy * math.sin(yaw_ref)
            lat_error = -dx * math.sin(yaw_ref) + dy * math.cos(yaw_ref)

            self.err_lon.append(lon_error)
            self.err_lat.append(lat_error)

            # Normalize yaw error to [-pi, pi]
            dyaw = yaw_ref - self.car_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            self.err_yaw.append(dyaw)

    def cmd_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.cmd_time.append(current_time)

        self.vel.append(msg.drive.speed)
        self.steer.append(msg.drive.steering_angle)

    # ----------------------------
    # Plot update
    # ----------------------------

    def update_plot(self):
        # Errors (Only plot if we have data)
        if self.err_time:
            t_err = list(self.err_time)
            self.plot_err_lon.set_data(t_err, list(self.err_lon))
            self.plot_err_lat.set_data(t_err, list(self.err_lat))
            self.plot_err_yaw.set_data(t_err, list(self.err_yaw))

        # Controls (Only plot if we have data)
        if self.cmd_time:
            t_cmd = list(self.cmd_time)
            self.plot_vel.set_data(t_cmd, list(self.vel))
            self.plot_steer.set_data(t_cmd, list(self.steer))

        # Autoscale all axes
        for row in self.axs:
            for ax in row:
                ax.relim()
                ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
