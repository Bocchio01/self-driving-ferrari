import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    platform_arg = DeclareLaunchArgument("platform")
    platform = LaunchConfiguration("platform")
    vehicle_config_arg = DeclareLaunchArgument("vehicle_config")
    vehicle_config = LaunchConfiguration("vehicle_config")

    pkg_dir = get_package_share_directory("ferrari_control")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace="control",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'ground'"])),
        parameters=[
            {
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }
        ],
    )

    joypad_controller_node = Node(
        package="ferrari_control",
        executable="joypad_controller_node",
        name="joypad_controller_node",
        output="screen",
        namespace="control",
        remappings=[
            ("toggle_actuators", "/vehicle/toggle_actuators"),
        ],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'ground'"])),
        parameters=[
            vehicle_config,
            {
                "steering_axis": 0,
                "speed_axis": 4,
                "switch_controller_button": 8,
                "toggle_actuators_button": 9,
                "invert_steering": False,
                "invert_speed": False,
            },
        ],
    )

    # States: [x, y, yaw, v_log, v_lat, w]
    # Control: [acceleration, steering_angle]
    mpc_controller_node = Node(
        package="ferrari_control",
        executable="mpc_controller_node",
        name="mpc_controller_node",
        namespace="control",
        output="screen",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            vehicle_config,
            {
                "prediction_horizon": 10,
                "control_horizon": 5,
                "Q": [100.0, 100.0, 10.0, 5.0, 5.0, 0.0],
                "QN": [100.0, 100.0, 10.0, 5.0, 5.0, 0.0],
                "R": [0.5, 1.0],
                "S": [10.0, 50.0],
                "state_min": [-1000.0, -1000.0, -3.1415, +0.0, -2.0, -3.0],
                "state_max": [+1000.0, +1000.0, +3.1415, +8.0, +2.0, +3.0],
            },
        ],
    )

    # States: [d, mu]
    # Control: [steering_angle]
    lateral_mpc_controller_node = Node(
        package="ferrari_control",
        executable="lateral_mpc_controller_node",
        name="lateral_mpc_controller_node",
        namespace="control",
        output="screen",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            vehicle_config,
            {
                "prediction_horizon": 10,
                "control_horizon": 5,
                "Q": [100.0, 10.0],
                "QN": [100.0, 30.0],
                "R": [1.0],
                "S": [0.0],  # ??
                "state_min": [-0.5, -1.57],
                "state_max": [+0.5, +1.57],
            },
        ],
    )

    lateral_stanley_controller_node = Node(
        package="ferrari_control",
        executable="lateral_stanley_controller_node",
        name="lateral_stanley_controller_node",
        namespace="control",
        output="screen",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            vehicle_config,
            {
                "k_gain": 2.0,
                "k_soft": 1.0,
            },
        ],
    )

    lateral_pure_pursuit_controller_node = Node(
        package="ferrari_control",
        executable="lateral_pure_pursuit_controller_node",
        name="lateral_pure_pursuit_controller_node",
        namespace="control",
        output="screen",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            vehicle_config,
            {
                "min_lookahead": 0.3,
                "lookahead_gain": 0.2,
            },
        ],
    )

    longitudinal_pid_controller_node = Node(
        package="ferrari_control",
        executable="longitudinal_pid_controller_node",
        name="longitudinal_pid_controller_node",
        namespace="control",
        output="screen",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            vehicle_config,
            {
                "kp": 1.0,
                "ki": 0.0,
                "kd": 0.1,
            },
        ],
    )

    controller_multiplexer_node = Node(
        package="ferrari_control",
        executable="controller_multiplexer_node",
        name="controller_multiplexer_node",
        output="screen",
        namespace="control",
        remappings=[
            ("ackermann_cmd", "/vehicle/ackermann_cmd"),
        ],
        condition=IfCondition(PythonExpression(["'", platform, "' == 'onboard'"])),
        parameters=[
            {
                "control_rate_hz": 30.0,
                "control_timeout_s": 0.25,
            },
        ],
    )

    controller_visualizer_node = Node(
        package="ferrari_control",
        executable="controller_visualizer.py",
        name="controller_visualizer_node",
        namespace="control",
        output="screen",
        condition=IfCondition(PythonExpression(["'", platform, "' == 'ground'"])),
        remappings=[
            ("odom", "/localization/odom"),
            ("local_trajectory", "/planning/local_trajectory"),
            ("ackermann_cmd", "/vehicle/ackermann_cmd"),
        ],
    )

    return LaunchDescription(
        [
            vehicle_config_arg,
            platform_arg,
            joy_node,
            joypad_controller_node,
            mpc_controller_node,
            lateral_mpc_controller_node,
            lateral_stanley_controller_node,
            lateral_pure_pursuit_controller_node,
            longitudinal_pid_controller_node,
            controller_multiplexer_node,
            controller_visualizer_node,
        ]
    )
