import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    filter_arg = DeclareLaunchArgument("filter", default_value="ekf")
    filter_type = LaunchConfiguration("filter")

    config = os.path.join(
        get_package_share_directory("ferrari_localization"),
        "config",
        "localization.yaml",
    )

    localization_node = Node(
        package="ferrari_localization",
        executable="localization_node",
        name="localization_node",
        namespace="localization",
        output="screen",
        parameters=[config, {"filter": filter_type}],
        remappings=[
            ("wheel_odom", "/vehicle/odom"),
            ("imu", "/vehicle/imu"),
        ],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
    )

    return LaunchDescription(
        [
            platform_arg,
            vehicle_config_arg,
            filter_arg,
            localization_node,
        ]
    )
