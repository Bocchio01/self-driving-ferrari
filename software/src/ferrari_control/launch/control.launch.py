import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")

    pkg_dir = get_package_share_directory("ferrari_control")

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "teleop.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", platform, "' == 'ground'"])),
    )

    auto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "auto.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
    )

    control_gate_node = Node(
        package="ferrari_control",
        executable="control_gate_node",
        name="control_gate_node",
        output="screen",
        namespace="control",
        remappings=[
            ("ackermann_cmd", "/vehicle/ackermann_cmd"),
        ],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            {
                "control_rate_hz": 30.0,
                "teleop_timeout_s": 0.25,
            }
        ],
    )

    return LaunchDescription(
        [
            platform_arg,
            teleop_launch,
            auto_launch,
            control_gate_node,
        ]
    )
