import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Get the path to the package
    pkg_path = get_package_share_directory("ferrari_description")
    # pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Process the Xacro file to get the robot description
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toprettyxml(indent="  ")

    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_path, "rviz", "display.rviz")

    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # simulation_bridge_node = Node(
    #     package="ferrari_description",
    #     executable="ackermann_to_gazebo",
    #     name="ackermann_to_gazebo",
    #     # parameters=[{"use_sim_time": True}],
    #     output="screen",
    # )

    # # Avvia Modern Gazebo (gz sim)
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
    #     ),
    #     launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    # )

    # # Spawna il veicolo
    # spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=["-topic", "robot_description", "-name", "ferrari"],
    #     output="screen",
    # )

    # # IL BRIDGE: Modern Gazebo richiede questo nodo per far comunicare /cmd_vel
    # bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    #         "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #     ],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
            # gazebo,
            # simulation_bridge_node,
            # spawn_entity,
            # bridge,
        ]
    )
