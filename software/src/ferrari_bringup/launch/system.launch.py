import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="all",
        description='Deployment mode: "ground_station", "vehicle", or "all"',
    )
    mode = LaunchConfiguration("mode")

    ferrari_control_dir = get_package_share_directory("ferrari_control")
    ferrari_vehicle_dir = get_package_share_directory("ferrari_vehicle")

    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ferrari_vehicle_dir, "launch", "vehicle.launch.py")
        ),
        launch_arguments={"mode": mode}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' in ['vehicle', 'all']"])),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ferrari_control_dir, "launch", "control.launch.py")
        ),
        launch_arguments={"mode": mode}.items(),
    )

    return LaunchDescription(
        [
            mode_arg,
            vehicle_launch,
            control_launch,
        ]
    )
