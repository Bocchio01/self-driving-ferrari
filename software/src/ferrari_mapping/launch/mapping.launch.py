from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    mode_arg = DeclareLaunchArgument("slam_mode", default_value="mapping")
    map_file_arg = DeclareLaunchArgument("map_file", default_value="")

    slam_node = Node(
        package="ferrari_mapping",
        executable="slam_node",
        name="slam_node",
        namespace="mapping",
        output="screen",
        parameters=[
            {
                "mode": LaunchConfiguration("slam_mode"),
                "map_file": LaunchConfiguration("map_file"),
                "map_size_m": 50.0,
                "resolution": 0.05,
            }
        ],
        remappings=[("scan", "/sensing/lidar/scan")],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
    )

    return LaunchDescription(
        [platform_arg, vehicle_config_arg, mode_arg, map_file_arg, slam_node]
    )
