from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    control_algo_arg = DeclareLaunchArgument(
        "control_algorithm",
        default_value="pid",
        description='Choose the autonomous control algorithm: "pid" or "mpc"',
    )

    control_algo_choice = LaunchConfiguration("control_algorithm")

    pid_node = Node(
        package="ferrari_control",
        executable="pure_pursuit_node",
        name="pure_pursuit_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", control_algo_choice, "' == 'pid'"])
        ),
    )

    mpc_node = Node(
        package="ferrari_control",
        executable="mpc_follower_node",
        name="mpc_controller",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", control_algo_choice, "' == 'mpc'"])
        ),
    )

    return LaunchDescription(
        [
            control_algo_arg,
            pid_node,
            mpc_node,
        ]
    )
