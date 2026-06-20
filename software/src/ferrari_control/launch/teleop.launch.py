from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    teleop_device_arg = DeclareLaunchArgument(
        "teleop_device",
        default_value="joy",
        description='Choose teleop device: "joy" or "keyboard"',
    )

    device_choice = LaunchConfiguration("teleop_device")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }
        ],
    )

    teleop_joy_node = Node(
        package="ferrari_control",
        executable="teleop_joy_node",
        name="teleop_joy_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", device_choice, "' == 'joy'"])),
        parameters=[
            {
                "steering_axis": 0,
                "speed_axis": 1,
                "switch_gate_mode_button": 8,
                "toggle_engage_vehicle_button": 9,
                "invert_steering": True,
                "invert_speed": False,
            }
        ],
    )

    teleop_keyboard_node = Node(
        package="ferrari_control",
        executable="teleop_keyboard_node",
        name="teleop_keyboard_node",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", device_choice, "' == 'keyboard'"])
        ),
    )

    return LaunchDescription(
        [
            teleop_device_arg,
            joy_node,
            teleop_joy_node,
            teleop_keyboard_node,
        ]
    )
