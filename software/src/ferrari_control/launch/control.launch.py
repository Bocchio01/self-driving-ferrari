import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ferrari_control")

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "teleop.launch.py")
        )
    )

    # auto_control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_dir, "launch", "auto_control.launch.py")
    #     ),
    # )

    control_gate_node = Node(
        package="ferrari_control",
        executable="control_gate_node",
        name="control_gate_node",
        output="screen",
        parameters=[
            {
                "control_rate_hz": 30.0,
                "teleop_timeout_s": 0.25,
                # "auto_timeout_s": 0.5,
            }
        ],
    )

    return LaunchDescription(
        [
            teleop_launch,
            # auto_control_launch,
            control_gate_node,
        ]
    )
