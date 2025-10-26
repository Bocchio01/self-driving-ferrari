from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Il driver nativo del joystick
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}],
    )

    # 2. Il tuo nodo che traduce il Joy in AckermannDrive
    joy_teleop_node = Node(
        package="ferrari_control",
        executable="joy_teleop_node",
        name="joy_teleop_node",
        output="screen",
    )

    # 3. Il Multiplexer (Gate)
    vehicle_cmd_gate_node = Node(
        package="ferrari_control",
        executable="vehicle_cmd_gate_node",
        name="vehicle_cmd_gate_node",
        output="screen",
    )

    return LaunchDescription(
        [
            joy_node,
            joy_teleop_node,
            vehicle_cmd_gate_node,
        ]
    )
