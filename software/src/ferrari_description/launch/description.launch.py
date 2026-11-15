import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    vehicle_config = LaunchConfiguration("vehicle_config")
    is_onboard = IfCondition(PythonExpression(["'", platform, "' == 'onboard'"]))
    is_ground = IfCondition(PythonExpression(["'", platform, "' == 'ground'"]))

    pkg_path = get_package_share_directory("ferrari_description")

    vehicle_config_file = os.path.join(pkg_path, "config", "ferrari_params.yaml")
    xacro_file = os.path.join(pkg_path, "urdf", "ferrari.urdf.xacro")
    rviz_config_file = os.path.join(pkg_path, "rviz", "display.rviz")

    with open(vehicle_config_file, "r") as f:
        yaml_config = yaml.safe_load(f)

    params = yaml_config["/**"]["ros__parameters"]
    xacro_mappings = {
        "wheelbase": str(params["wheelbase"]),
        "track_width": str(params["track_width"]),
        "wheel_diameter": str(params["wheel_diameter"]),
        "wheel_thickness": str(params["wheel_thickness"]),
    }

    robot_description_config = xacro.process_file(xacro_file, mappings=xacro_mappings)
    robot_description = robot_description_config.toprettyxml(indent="  ")

    rqt_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="description",
        parameters=[{"robot_description": robot_description}],
        condition=is_onboard,
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        namespace="description",
        parameters=[{"robot_description": robot_description}],
    )

    ferrari_state_publisher_node = Node(
        package="ferrari_description",
        executable="ferrari_state_publisher.py",
        name="ferrari_state_publisher_node",
        namespace="description",
        parameters=[vehicle_config],
        remappings=[
            ("ackermann_cmd", "/vehicle/ackermann_cmd"),
        ],
        condition=is_ground,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace="description",
        arguments=["-d", rviz_config_file],
        condition=is_ground,
    )

    return LaunchDescription(
        [
            platform_arg,
            vehicle_config_arg,
            # rqt_node,
            robot_state_publisher_node,
            # joint_state_publisher_gui_node,
            ferrari_state_publisher_node,
            rviz_node,
        ]
    )
