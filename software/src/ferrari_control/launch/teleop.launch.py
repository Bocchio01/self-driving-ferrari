from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="joy",
        description='Choose teleop device: "joy" or "keyboard"',
    )
    device = LaunchConfiguration("device")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace="control",
        condition=IfCondition(PythonExpression(["'", device, "' == 'joy'"])),
        parameters=[
            {
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }
        ],
    )

    teleop_joy_node = Node(
        package="ferrari_control",
        executable="teleop_joy_node",
        name="teleop_joy_node",
        output="screen",
        namespace="control",
        remappings=[
            ("toggle_arm_actuators", "/vehicle/toggle_arm_actuators"),
        ],
        condition=IfCondition(PythonExpression(["'", device, "' == 'joy'"])),
        parameters=[
            {
                "steering_axis": 0,
                "speed_axis": 4,
                "switch_gate_mode_button": 8,
                "toggle_arm_actuators_button": 9,
                "invert_steering": False,
                "invert_speed": False,
            }
        ],
    )

    teleop_keyboard_node = Node(
        package="ferrari_control",
        executable="teleop_keyboard_node",
        name="teleop_keyboard_node",
        output="screen",
        namespace="control",
        remappings=[
            ("toggle_arm_actuators", "/vehicle/toggle_arm_actuators"),
        ],
        condition=IfCondition(PythonExpression(["'", device, "' == 'keyboard'"])),
    )

    return LaunchDescription(
        [
            device_arg,
            joy_node,
            teleop_joy_node,
            teleop_keyboard_node,
        ]
    )
