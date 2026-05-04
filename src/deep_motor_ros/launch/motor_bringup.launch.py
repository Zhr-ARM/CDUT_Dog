import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
                description="Command topic used by the motor driver.",
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
        ]
    )
