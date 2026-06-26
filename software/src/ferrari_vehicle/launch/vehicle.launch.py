from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

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
            "/dev/ttyACM0",
            "-v6",
        ],
        name="micro_ros_agent_docker",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            micro_ros_agent,
        ]
    )
