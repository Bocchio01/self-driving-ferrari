from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")

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
            "microros/micro-ros-agent:jazzy",
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
    )

    return LaunchDescription(
        [
            platform_arg,
            micro_ros_agent,
        ]
    )
