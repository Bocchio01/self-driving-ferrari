import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    vehicle_config = LaunchConfiguration("vehicle_config")

    mock_global_planner_node = Node(
        package="ferrari_planning",
        executable="mock_global_planner.py",
        name="mock_global_planner_node",
        namespace="planning",
        output="screen",
        remappings=[
            ("odom", "/vehicle/odom"),
        ],
    )

    local_planner_node = Node(
        package="ferrari_planning",
        executable="local_planner_node",
        name="local_planner_node",
        namespace="planning",
        output="screen",
        remappings=[
            ("odom", "/vehicle/odom"),
        ],
        parameters=[
            {
                "lookahead_distance": 5.0,
                "control_rate_hz": 10.0,
            },
        ],
    )

    trajectory_generator_node = Node(
        package="ferrari_planning",
        executable="trajectory_generator_node",
        name="trajectory_generator_node",
        namespace="planning",
        output="screen",
        parameters=[
            vehicle_config,
        ],
    )

    visualizer_node = Node(
        package="ferrari_planning",
        executable="visualizer.py",
        name="visualizer_node",
        namespace="planning",
        output="screen",
    )

    return LaunchDescription(
        [
            vehicle_config_arg,
            mock_global_planner_node,
            local_planner_node,
            trajectory_generator_node,
            # visualizer_node,
        ]
    )
