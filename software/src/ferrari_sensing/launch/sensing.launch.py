from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument("mode", default_value="all")
    mode = LaunchConfiguration("mode")

    camera_node = Node(
        package="ferrari_sensing",
        executable="camera_node",
        name="camera_node",
        condition=IfCondition(PythonExpression(["'", mode, "' in ['vehicle']"])),
        parameters=[
            {
                "jpeg_quality": 30,
                "publish_rate": 15.0,
                # "camera_info_file": "config/camera_info_fallback.yaml",
                # "camera_info_file": "config/camera_info_windshield.yaml",
                # "camera_info_file": "config/camera_info.yaml",
            }
        ],
    )

    return LaunchDescription(
        [
            mode_arg,
            camera_node,
        ]
    )
