import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

sys.path.insert(
    0,
    os.path.join(get_package_share_directory("ferrari_bringup"), "launch"),
)
from utils import include_package_launches  # noqa: E402

SIM_ARGUMENTS = [
    ("sim", "true"),
    ("filter", LaunchConfiguration("filter")),
    ("odom_drift", LaunchConfiguration("odom_drift")),
    ("slam_mode", LaunchConfiguration("slam_mode")),
    ("map_file", LaunchConfiguration("map_file")),
    ("global_planner", LaunchConfiguration("global_planner")),
]

ONBOARD_PACKAGES = [
    "ferrari_control",
    "ferrari_description",
    "ferrari_localization",
    "ferrari_mapping",
    "ferrari_vehicle",
]

GROUND_PACKAGES = [
    "ferrari_control",
    "ferrari_dashboard",
    "ferrari_description",
    "ferrari_planning",
]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("filter", default_value="ekf"),
            DeclareLaunchArgument("odom_drift", default_value="false"),
            DeclareLaunchArgument("slam_mode", default_value="mapping"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("global_planner", default_value="mock"),
            *include_package_launches(
                ONBOARD_PACKAGES, "onboard", extra_arguments=SIM_ARGUMENTS
            ),
            *include_package_launches(
                GROUND_PACKAGES, "ground", extra_arguments=SIM_ARGUMENTS
            ),
        ]
    )
