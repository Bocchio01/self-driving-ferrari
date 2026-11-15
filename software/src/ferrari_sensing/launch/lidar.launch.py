from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_node = Node(
        package="ferrari_sensing",
        executable="lidar_node",
        name="lidar_node",
        namespace="sensing/lidar",
        output="screen",
        parameters=[
            {
                "port": "/dev/ttyUSB0",
                "baudrate": 230400,
                "frame_id": "lidar_link",
                "scan_bins": 450,
                "angle_offset_rad": 0.0,
            }
        ],
    )

    return LaunchDescription([lidar_node])
