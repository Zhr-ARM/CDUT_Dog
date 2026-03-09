"""
步态仿真 Launch 文件 (position_control 包)

使用方法:
  1. 先启动 Gazebo 仿真环境:
      ros2 launch leg_model gazebo.launch.py

  2. 等待 Gazebo 和控制器完全加载后，启动步态控制器:
      ros2 launch position_control gait_sim.launch.py

  可选参数:
      ros2 launch position_control gait_sim.launch.py param_file:=/path/to/gait_controller.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_param_file = os.path.join(
        get_package_share_directory('position_control'),
        'config',
        'gait_controller.yaml'
    )

    args = [
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param_file,
            description='步态控制器参数文件路径'
        ),
    ]

    gait_controller_node = Node(
        package='position_control',
        executable='gait_controller',
        name='gait_controller',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )

    return LaunchDescription(args + [gait_controller_node])
