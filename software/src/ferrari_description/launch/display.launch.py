import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Get the path to the package
    pkg_path = get_package_share_directory("ferrari_description")

    # Process the Xacro file to get the robot description
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")

    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, "rviz", "display.rviz")

    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # Node: Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Node: RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
