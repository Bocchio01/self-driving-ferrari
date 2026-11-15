#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

HELP = """
Ferrari keyboard teleop (publishes sensor_msgs/Joy on joy)

  w / s : forward / backward
  a / d : steer left / right
  space : stop and center
  e     : toggle actuators
  q     : switch controller
  x     : quit

Keys must be held: the command returns to zero shortly after the last key event.
"""


class KeyboardJoy(Node):
    def __init__(self):
        super().__init__("keyboard_joy_node", namespace="control")

        self.steering_axis = self.declare_parameter("steering_axis", 0).value
        self.speed_axis = self.declare_parameter("speed_axis", 4).value
        self.toggle_actuators_button = self.declare_parameter(
            "toggle_actuators_button", 9
        ).value
        self.switch_controller_button = self.declare_parameter(
            "switch_controller_button", 8
        ).value
        self.speed_scale = self.declare_parameter("speed_scale", 0.5).value
        self.steering_scale = self.declare_parameter("steering_scale", 1.0).value
        self.hold_timeout = self.declare_parameter("hold_timeout_s", 0.7).value
        rate_hz = self.declare_parameter("rate_hz", 30.0).value

        self.publisher = self.create_publisher(Joy, "joy", 10)
        self.speed = 0.0
        self.steering = 0.0
        self.speed_stamp = 0.0
        self.steering_stamp = 0.0
        self.button_pulses = {}

        self.create_timer(1.0 / rate_hz, self.publish_joy)

    def handle_key(self, key):
        now = time.monotonic()
        if key == "w":
            self.speed, self.speed_stamp = self.speed_scale, now
        elif key == "s":
            self.speed, self.speed_stamp = -self.speed_scale, now
        elif key == "a":
            self.steering, self.steering_stamp = self.steering_scale, now
        elif key == "d":
            self.steering, self.steering_stamp = -self.steering_scale, now
        elif key == " ":
            self.speed = 0.0
            self.steering = 0.0
        elif key == "e":
            self.button_pulses[self.toggle_actuators_button] = now
        elif key == "q":
            self.button_pulses[self.switch_controller_button] = now

    def publish_joy(self):
        now = time.monotonic()
        if now - self.speed_stamp > self.hold_timeout:
            self.speed = 0.0
        if now - self.steering_stamp > self.hold_timeout:
            self.steering = 0.0

        size = max(self.steering_axis, self.speed_axis) + 1
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * size
        msg.axes[self.steering_axis] = float(self.steering)
        msg.axes[self.speed_axis] = float(self.speed)

        button_count = max([*self.button_pulses, 0]) + 1
        msg.buttons = [0] * button_count
        for button, stamp in list(self.button_pulses.items()):
            if now - stamp < 0.15:
                msg.buttons[button] = 1
            else:
                del self.button_pulses[button]
        self.publisher.publish(msg)


def main(args=None):
    if not sys.stdin.isatty():
        print("keyboard_joy.py needs an interactive terminal")
        return

    rclpy.init(args=args)
    node = KeyboardJoy()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == "x" or key == "\x03":
                    break
                node.handle_key(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
