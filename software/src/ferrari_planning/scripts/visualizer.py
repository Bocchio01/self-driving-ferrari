#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from ferrari_planning.msg import Trajectory

import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class PlannerVisualizerNode(Node):
    def __init__(self):
        super().__init__("planner_visualizer_node")

        # Data storage
        self.lock = threading.Lock()
        self.global_x, self.global_y = [], []
        self.local_x, self.local_y = [], []
        self.traj_t, self.traj_v = [], []

        # Subscribers
        self.create_subscription(Path, "global_path", self.global_path_cb, 10)
        self.create_subscription(Path, "local_path", self.local_path_cb, 10)
        self.create_subscription(Trajectory, "local_trajectory", self.trajectory_cb, 10)

        self.get_logger().info("Visualizer node started. Waiting for data...")

    def global_path_cb(self, msg):
        with self.lock:
            self.global_x = [p.pose.position.x for p in msg.poses]
            self.global_y = [p.pose.position.y for p in msg.poses]

    def local_path_cb(self, msg):
        with self.lock:
            self.local_x = [p.pose.position.x for p in msg.poses]
            self.local_y = [p.pose.position.y for p in msg.poses]

    def trajectory_cb(self, msg):
        with self.lock:
            self.traj_t = []
            self.traj_v = []
            for p in msg.points:
                # Convert builtin_interfaces/Duration to float seconds
                time_sec = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                speed = p.velocity.linear.x
                self.traj_t.append(time_sec)
                self.traj_v.append(speed)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerVisualizerNode()

    # Run ROS 2 node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Set up Matplotlib
    fig = plt.figure(figsize=(12, 6))

    # Left subplot: Track Map (X/Y)
    ax1 = fig.add_subplot(121)
    ax1.set_title("Path Visualization")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.grid(True)

    # Right subplot: Speed Profile (Time/Velocity)
    ax2 = fig.add_subplot(122)
    ax2.set_title("Trajectory Speed Profile")
    ax2.set_xlabel("Time from start (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid(True)

    def update_plot(frame):
        with node.lock:
            g_x, g_y = list(node.global_x), list(node.global_y)
            l_x, l_y = list(node.local_x), list(node.local_y)
            t_t, t_v = list(node.traj_t), list(node.traj_v)

        # Update ax1 (Spatial)
        ax1.cla()
        ax1.set_title("Path Visualization (Spatial)")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.grid(True)
        if g_x:
            ax1.plot(g_x, g_y, "k--", label="Global Path", alpha=0.5)
        if l_x:
            ax1.plot(l_x, l_y, "b-", label="Local Path", linewidth=2)
            # Plot the start of the local path with a red dot to show vehicle position
            ax1.plot(l_x[0], l_y[0], "ro", label="Vehicle")
        ax1.axis("equal")  # Keep aspect ratio 1:1 for the map
        ax1.legend(loc="upper right")

        # Update ax2 (Speed Profile)
        ax2.cla()
        ax2.set_title("Trajectory Speed Profile")
        ax2.set_xlabel("Time from start (s)")
        ax2.set_ylabel("Velocity (m/s)")
        ax2.grid(True)
        if t_t and t_v:
            ax2.plot(t_t, t_v, "r-", linewidth=2)
            ax2.fill_between(t_t, t_v, alpha=0.2, color="red")
            ax2.set_ylim(bottom=0.0)

    # Use FuncAnimation to redraw at 10 Hz (100 ms interval)
    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    plt.tight_layout()
    plt.show()  # This blocks the main thread until the window is closed

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
