import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_launch_file_path(package_name: str, launch_file_name: str) -> str:
    """Resolve and validate the path to a launch file."""

    path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file_name,
    )

    if not os.path.exists(path):
        raise FileNotFoundError(f"Launch file not found: {path}")

    return path


def include_package_launch(
    package_name: str,
    platform: str,
) -> IncludeLaunchDescription:
    """Generate an IncludeLaunchDescription for a package based on naming conventions."""

    launch_file_name = f"{package_name.split('_')[1]}.launch.py"
    launch_path = get_launch_file_path(package_name, launch_file_name)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=[("platform", platform)],
    )


def include_package_launches(
    package_names: list[str],
    platform: str,
) -> list[IncludeLaunchDescription]:
    """Generate a list of IncludeLaunchDescription for multiple packages."""

    return [
        include_package_launch(package_name, platform) for package_name in package_names
    ]
