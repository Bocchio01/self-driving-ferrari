#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDriveStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__("ferrari_state_publisher_node")

        self.wheel_radius = (
            self.declare_parameter("wheel_diameter", 1e-3)
            .get_parameter_value()
            .double_value
        ) / 2.0

        self.create_subscription(
            AckermannDriveStamped, "ackermann_cmd", self.cmd_callback, 10
        )
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # State variables for wheels
        self.steering_angle = 0.0
        self.speed = 0.0
        self.wheel_angle = 0.0

        # Timer to publish joint states smoothly at 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)
        self.last_time = self.get_clock().now()

    def cmd_callback(self, msg):
        self.steering_angle = msg.drive.steering_angle
        self.speed = msg.drive.speed

    def publish_joint_states(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate wheel rolling angle (omega = v / r)
        self.wheel_angle += (self.speed / self.wheel_radius) * dt

        # Keep the angle bounded between 0 and 2*PI
        self.wheel_angle = math.fmod(self.wheel_angle, 2 * math.pi)

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()

        joint_msg.name = [
            "front_wheel_right_hinge",
            "front_wheel_left_hinge",
            "front_wheel_right_rotate",
            "front_wheel_left_rotate",
            "rear_wheel_right_rotate",
            "rear_wheel_left_rotate",
        ]

        joint_msg.position = [
            self.steering_angle,
            self.steering_angle,
            self.wheel_angle,
            self.wheel_angle,
            self.wheel_angle,
            self.wheel_angle,
        ]

        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
