from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument("mode", default_value="all")
    mode = LaunchConfiguration("mode")

    bridge_node = Node(
        package="ferrari_dashboard",
        executable="bridge.js",
        name="ferrari_dashboard_bridge",
        output="screen",
        condition=IfCondition(PythonExpression(["'", mode, "' in ['ground_station']"])),
    )

    return LaunchDescription(
        [
            mode_arg,
            bridge_node,
        ]
    )
