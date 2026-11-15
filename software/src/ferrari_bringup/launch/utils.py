import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

VEHICLE_CONFIG = os.path.join(
    get_package_share_directory("ferrari_description"),
    "config",
    "ferrari_params.yaml",
)


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
    vehicle_config: str = VEHICLE_CONFIG,
    extra_arguments: list[tuple[str, str]] | None = None,
) -> GroupAction:

    launch_file_name = f"{package_name.split('_')[1]}.launch.py"
    launch_path = get_launch_file_path(package_name, launch_file_name)

    return GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments=[
                    ("platform", platform),
                    ("vehicle_config", vehicle_config),
                    *(extra_arguments or []),
                ],
            )
        ],
        scoped=True,
    )


def include_package_launches(
    package_names: list[str],
    platform: str,
    vehicle_config: str = VEHICLE_CONFIG,
    extra_arguments: list[tuple[str, str]] | None = None,
) -> list[GroupAction]:
    return [
        include_package_launch(package_name, platform, vehicle_config, extra_arguments)
        for package_name in package_names
    ]
