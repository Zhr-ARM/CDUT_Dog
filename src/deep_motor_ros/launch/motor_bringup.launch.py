import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("deep_motor_ros"),
        "config",
        "four_can_real_robot.yaml",
    )

    command_topic = LaunchConfiguration("command_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=default_param_file,
                description="deep_motor_ros parameter file for the real robot",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/motor_cmd",
                description="Command topic shared by the driver and optional test node.",
            ),
            DeclareLaunchArgument(
                "launch_test_node",
                default_value="false",
                description="Launch a small sinusoidal motor test publisher.",
            ),
            DeclareLaunchArgument(
                "test_motor_count",
                default_value="12",
                description="Total motor command slots published by the test node.",
            ),
            DeclareLaunchArgument(
                "test_motor_index",
                default_value="0",
                description="Motor index to move during the test.",
            ),
            DeclareLaunchArgument(
                "test_amplitude",
                default_value="0.15",
                description="Sine motion amplitude in radians.",
            ),
            DeclareLaunchArgument(
                "test_frequency_hz",
                default_value="0.25",
                description="Sine motion frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "test_kp",
                default_value="6.0",
                description="Position gain used by the test node.",
            ),
            DeclareLaunchArgument(
                "test_kd",
                default_value="0.3",
                description="Damping gain used by the test node.",
            ),
            Node(
                package="deep_motor_ros",
                executable="motor_node",
                name="deep_motor_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("param_file"),
                    {"command_topic": command_topic},
                ],
            ),
            Node(
                package="deep_motor_ros",
                executable="motor_test_node",
                name="motor_test_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_test_node")),
                parameters=[
                    {
                        "command_topic": command_topic,
                        "motor_count": LaunchConfiguration("test_motor_count"),
                        "active_motor_index": LaunchConfiguration("test_motor_index"),
                        "amplitude": LaunchConfiguration("test_amplitude"),
                        "frequency_hz": LaunchConfiguration("test_frequency_hz"),
                        "kp": LaunchConfiguration("test_kp"),
                        "kd": LaunchConfiguration("test_kd"),
                    }
                ],
            ),
        ]
    )
