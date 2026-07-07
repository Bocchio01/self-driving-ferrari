from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    bridge_node = Node(
        package="ferrari_dashboard",
        executable="bridge.js",
        name="ferrari_dashboard_bridge",
        output="screen",
        namespace="dashboard",
        remappings=[
            ("image_raw/compressed", "/sensing/camera/image_raw/compressed"),
        ],
    )

    return LaunchDescription(
        [
            bridge_node,
        ]
    )
