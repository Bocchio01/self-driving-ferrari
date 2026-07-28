#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class OdomQosBridge(Node):
    def __init__(self):
        super().__init__("odom_qos_bridge_node")

        # 1. Define the BEST_EFFORT QoS profile to match the Teensy
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 2. Define the RELIABLE QoS profile to match your controllers
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 3. Create the subscriber (listening to the Teensy)
        self.subscription = self.create_subscription(
            Odometry, "/vehicle/odom", self.odom_callback, best_effort_qos
        )

        # 4. Create the publisher (broadcasting to the controllers)
        self.publisher = self.create_publisher(Odometry, "odom", reliable_qos)

    def odom_callback(self, msg):
        # Pass the message straight through to the reliable topic
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomQosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
