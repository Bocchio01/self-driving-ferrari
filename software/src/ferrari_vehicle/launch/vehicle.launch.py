from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    micro_ros_agent = ExecuteProcess(
        cmd=[
            "docker",
            "run",
            "--rm",
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
    )

    return LaunchDescription(
        [
            micro_ros_agent,
        ]
    )
