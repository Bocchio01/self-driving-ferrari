import os
from typing import Optional
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


PACKAGE_NAMES: dict[str, Optional[str]] = {
    "ferrari_bringup": None,
    # "ferrari_description": "simulation.launch.py",
    "ferrari_control": "control.launch.py",
    "ferrari_vehicle": "vehicle.launch.py",
}


def create_include_launch_description(
    package_name: str,
    launch_file: Optional[str],
) -> Optional[IncludeLaunchDescription]:

    if launch_file is not None:
        launch_file_path = os.path.join(
            get_package_share_directory(package_name),
            "launch",
            launch_file,
        )
        if os.path.exists(launch_file_path):
            return IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file_path)
            )
        else:
            print(f"Launch file {launch_file} not found in package {package_name}")

    return None


def generate_launch_description():

    include_launch_descriptions = []

    for package_name, launch_file in PACKAGE_NAMES.items():
        include_launch_description = create_include_launch_description(
            package_name,
            launch_file,
        )
        if include_launch_description is not None:
            include_launch_descriptions.append(include_launch_description)

    return LaunchDescription(include_launch_descriptions)
