from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # mpc_node = Node(
    #     package="ferrari_control",
    #     executable="mpc_follower_node",
    #     name="mpc_controller",
    #     output="screen",
    # )

    return LaunchDescription(
        [
            # mpc_node,
        ]
    )
