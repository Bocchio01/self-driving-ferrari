import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

DEFAULT_EXECUTABLE = "/opt/plotjuggler4/bin/plotjuggler4"
DEFAULT_LAYOUT_NAME = "ferrari.pj4.xml"


def generate_launch_description():
    default_layout = os.path.join(
        get_package_share_directory("ferrari_bringup"),
        "plotjuggler",
        DEFAULT_LAYOUT_NAME,
    )
    if not os.path.exists(default_layout):
        default_layout = ""

    executable_arg = DeclareLaunchArgument(
        "executable", default_value=DEFAULT_EXECUTABLE
    )
    layout_arg = DeclareLaunchArgument("layout", default_value=default_layout)
    executable = LaunchConfiguration("executable")
    layout = LaunchConfiguration("layout")
    has_layout = PythonExpression(["'", layout, "' != ''"])

    plotjuggler_with_layout = ExecuteProcess(
        cmd=[executable, "--layout", layout],
        name="plotjuggler",
        output="screen",
        condition=IfCondition(has_layout),
    )

    plotjuggler = ExecuteProcess(
        cmd=[executable],
        name="plotjuggler",
        output="screen",
        condition=UnlessCondition(has_layout),
    )

    return LaunchDescription(
        [executable_arg, layout_arg, plotjuggler_with_layout, plotjuggler]
    )
