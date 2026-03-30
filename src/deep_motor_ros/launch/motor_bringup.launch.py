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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=default_param_file,
                description="deep_motor_ros parameter file for the real robot",
            ),
            Node(
                package="deep_motor_ros",
                executable="motor_node",
                name="deep_motor_node",
                output="screen",
                parameters=[LaunchConfiguration("param_file")],
            ),
        ]
    )
