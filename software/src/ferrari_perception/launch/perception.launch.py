from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument("mode", default_value="all")
    mode = LaunchConfiguration("mode")

    line_detector_node = Node(
        package="ferrari_perception",
        executable="line_detector_node",
        name="line_detector_node",
        condition=IfCondition(PythonExpression(["'", mode, "' in ['vehicle', 'all']"])),
        parameters=[{}],
    )

    return LaunchDescription(
        [
            mode_arg,
            line_detector_node,
        ]
    )
