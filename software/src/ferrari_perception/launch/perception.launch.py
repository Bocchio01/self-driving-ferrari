import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")

    config_params = os.path.join(
        get_package_share_directory("ferrari_perception"), "config", "calibration.yaml"
    )

    line_detector_node = Node(
        package="ferrari_perception",
        executable="line_detector_node",
        name="line_detector_node",
        namespace="perception",
        remappings=[
            ("image_rect", "/sensing/camera/image_rect"),
        ],
        parameters=[
            config_params,
            {
                "image_transport": PythonExpression(
                    ["'compressed' if '", platform, "' == 'ground' else 'raw'"]
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            platform_arg,
            line_detector_node,
        ]
    )
