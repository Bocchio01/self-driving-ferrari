from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    bridge_node = Node(
        package="ferrari_dashboard",
        executable="bridge.js",
        name="ferrari_dashboard_bridge",
        output="screen",
    )

    return LaunchDescription(
        [
            bridge_node,
        ]
    )
