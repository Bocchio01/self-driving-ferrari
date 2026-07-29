from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    vehicle_config = LaunchConfiguration("vehicle_config")

    micro_ros_agent = ExecuteProcess(
        cmd=[
            "docker",
            "run",
            "-i",
            "--rm",
            "--init",
            "-v",
            "/dev:/dev",
            "-v",
            "/dev/shm:/dev/shm",
            "--privileged",
            "--net=host",
            "micro-ros-agent-ferrari:jazzy",
            "serial",
            "--dev",
            PythonExpression(
                ["'/dev/serial0' if '", platform, "' == 'onboard' else '/dev/ttyACM0'"]
            ),
            "-b",
            "1000000",
            "-v4",
        ],
        name="micro_ros_agent_docker",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
    )

    odom_proxy_node = Node(
        package="ferrari_vehicle",
        executable="odom_proxy_node",
        name="odom_proxy_node",
        namespace="vehicle",
        output="screen",
        parameters=[vehicle_config],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
    )

    ferrari_twin_node = Node(
        package="ferrari_vehicle",
        executable="ferrari_twin_node",
        name="ferrari_twin_node",
        namespace="vehicle",
        output="screen",
        parameters=[vehicle_config],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'ground'"])),
    )

    return LaunchDescription(
        [
            platform_arg,
            vehicle_config_arg,
            micro_ros_agent,
            odom_proxy_node,
            ferrari_twin_node,
        ]
    )
