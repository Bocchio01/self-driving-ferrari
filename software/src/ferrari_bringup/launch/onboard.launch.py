import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription

sys.path.insert(
    0,
    os.path.join(get_package_share_directory("ferrari_bringup"), "launch"),
)
from utils import include_package_launches  # noqa: E402

PLATFORM = "onboard"
PACKAGES = [
    # "ferrari_bringup",
    "ferrari_control",
    # "ferrari_dashboard",
    # "ferrari_description",
    # "ferrari_localization",
    # "ferrari_mapping",
    # "ferrari_perception",
    # "ferrari_planning",
    "ferrari_sensing",
    "ferrari_vehicle",
]


def generate_launch_description():
    launch_actions = include_package_launches(PACKAGES, PLATFORM)
    return LaunchDescription([*launch_actions])
