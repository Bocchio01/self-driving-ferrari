from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    vehicle_config = LaunchConfiguration("vehicle_config")
    sim_arg = DeclareLaunchArgument("sim", default_value="false")
    sim = LaunchConfiguration("sim")
    drift_arg = DeclareLaunchArgument("odom_drift", default_value="false")
    drift = LaunchConfiguration("odom_drift")

    def drift_value(value):
        return ParameterValue(
            PythonExpression([str(value), " if '", drift, "' == 'true' else 0.0"]),
            value_type=float,
        )

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
        condition=IfCondition(
            PythonExpression(["'", platform, "' == 'onboard' and '", sim, "' != 'true'"])
        ),
    )

    odom_proxy_node = Node(
        package="ferrari_vehicle",
        executable="odom_proxy_node",
        name="odom_proxy_node",
        namespace="vehicle",
        output="screen",
        parameters=[vehicle_config],
        condition=IfCondition(
            PythonExpression(["'", platform, "' == 'onboard' or '", sim, "' == 'true'"])
        ),
    )

    ferrari_twin_node = Node(
        package="ferrari_vehicle",
        executable="ferrari_twin_node",
        name="ferrari_twin_node",
        namespace="vehicle",
        output="screen",
        parameters=[
            vehicle_config,
            {
                "odom_speed_scale_error": drift_value(0.03),
                "odom_steering_offset": drift_value(0.008),
                "odom_speed_noise_std": drift_value(0.02),
                "odom_yaw_rate_noise_std": drift_value(0.02),
            },
        ],
        condition=IfCondition(PythonExpression(["'", sim, "' == 'true'"])),
    )

    lidar_simulator_node = Node(
        package="ferrari_vehicle",
        executable="lidar_simulator_node",
        name="lidar_simulator_node",
        namespace="vehicle",
        output="screen",
        remappings=[("scan", "/sensing/lidar/scan")],
        condition=IfCondition(PythonExpression(["'", sim, "' == 'true'"])),
    )

    return LaunchDescription(
        [
            platform_arg,
            vehicle_config_arg,
            sim_arg,
            drift_arg,
            micro_ros_agent,
            odom_proxy_node,
            ferrari_twin_node,
            lidar_simulator_node,
        ]
    )
