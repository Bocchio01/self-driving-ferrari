from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    mode_arg = DeclareLaunchArgument("mode", default_value="all")
    mode = LaunchConfiguration("mode")

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
                [
                    "'/dev/ttyS0' if '",
                    mode,
                    "' == 'vehicle' else '/dev/ttyACM0'",
                ]
            ),
            "-v6",
        ],
        name="micro_ros_agent_docker",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            mode_arg,
            micro_ros_agent,
        ]
    )
