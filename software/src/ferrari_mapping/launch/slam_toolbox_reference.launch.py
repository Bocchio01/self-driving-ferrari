import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("ferrari_mapping"),
        "config",
        "slam_toolbox_reference.yaml",
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={
                    "slam_params_file": params_file,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        ]
    )
