import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("dog_position_control"),
        "config",
        "gait_controller_sim_trot.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=default_param_file,
                description="Quadruped gait controller parameter file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock for the gait controller.",
            ),
            Node(
                package="dog_position_control",
                executable="quadruped_gait_controller",
                name="quadruped_gait_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("param_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
