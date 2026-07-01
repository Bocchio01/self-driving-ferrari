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

    image_proc_node = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="camera",
        remappings=[
            ("image", "image_raw"),
            ("camera_info", "camera_info"),
            ("image_rect", "image_rect"),
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' in ['vehicle', 'all']"])),
    )

    return LaunchDescription(
        [
            mode_arg,
            camera_node,
            image_proc_node,
        ]
    )
