import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CAMERA_CONFIG_FILES = [
    "camera_info_windshield.yaml",
    "camera_info.yaml",
]


def generate_launch_description():

    config_params = os.path.join(
        get_package_share_directory("ferrari_sensing"), "config", CAMERA_CONFIG_FILES[1]
    )

    camera_node = Node(
        package="ferrari_sensing",
        executable="camera_node",
        name="camera_node",
        namespace="sensing/camera",
        parameters=[
            config_params,
            {
                "jpeg_quality": 30,
                "publish_rate": 15,
                "enable_manual_exposure": True,
                "exposure_time": 5000,
                "analogue_gain": 2.0,
            },
        ],
    )

    image_proc_node = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="sensing/camera",
        remappings=[
            ("image", "image_raw"),
            ("image_rect", "image_rect"),
        ],
        parameters=[{"image_transport": "compressed"}],
    )

    return LaunchDescription(
        [
            camera_node,
            image_proc_node,
        ]
    )
