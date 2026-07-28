import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    vehicle_config = LaunchConfiguration("vehicle_config")

    odom_qos_bridge_node = Node(
        package="ferrari_localization",
        executable="odom_qos_bridge.py",
        name="odom_qos_bridge_node",
        namespace="localization",
        output="screen",
    )

    return LaunchDescription(
        [
            vehicle_config_arg,
            odom_qos_bridge_node,
        ]
    )
