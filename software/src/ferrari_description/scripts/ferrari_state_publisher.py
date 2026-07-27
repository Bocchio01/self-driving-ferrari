#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from ackermann_msgs.msg import AckermannDriveStamped


class FerrariStatePublisher(Node):
    def __init__(self):
        super().__init__("ferrari_state_publisher_node")

        wheel_diameter = (
            self.declare_parameter("wheel_diameter", 1e-3)
            .get_parameter_value()
            .double_value
        )
        self.wheel_radius = wheel_diameter / 2.0

        self.steering_angle = 0.0
        self.speed = 0.0
        self.wheel_angle = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(
            AckermannDriveStamped,
            "ackermann_cmd",
            self.cmd_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

    def odom_callback(self, msg: Odometry):
        # Broadcast the transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Publish the joint states
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.wheel_angle += (self.speed / self.wheel_radius) * dt
        self.wheel_angle = math.fmod(self.wheel_angle, 2 * math.pi)

        joint_msg = JointState()
        joint_msg.header.stamp = msg.header.stamp
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

    def cmd_callback(self, msg: AckermannDriveStamped):
        self.steering_angle = msg.drive.steering_angle
        self.speed = msg.drive.speed


def main(args=None):
    rclpy.init(args=args)
    node = FerrariStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
