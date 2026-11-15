from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    port_arg = DeclareLaunchArgument("dashboard_port", default_value="8080")

    dashboard_node = Node(
        package="ferrari_dashboard",
        executable="dashboard_server.py",
        name="dashboard_node",
        namespace="dashboard",
        output="screen",
        parameters=[{"port": LaunchConfiguration("dashboard_port")}],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'ground'"])),
    )

    return LaunchDescription([platform_arg, vehicle_config_arg, port_arg, dashboard_node])
