# -*- coding: utf-8 -*-
"""
mpc_gait_sim.launch.py
======================

作用：
1) 启动 `mpc_control` 包中的 `mpc_gait_controller` 节点；
2) 支持通过 launch 参数覆盖默认参数文件；
3) 用于 Gazebo + leg_mit_controller 仿真链路。

典型用法：
    ros2 launch mpc_control mpc_gait_sim.launch.py

自定义参数文件：
    ros2 launch mpc_control mpc_gait_sim.launch.py \
            param_file:=/absolute/path/to/your.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 从已安装包目录中定位默认参数文件。
    # 注意：这里使用 get_package_share_directory，
    # 因为 launch 运行时通常读取 install 空间而非源码目录。
    default_param_file = os.path.join(
        get_package_share_directory('mpc_control'),
        'config',
        'mpc_gait_controller.yaml'
    )

    # 声明可覆盖参数：param_file
    # 用户可在命令行传入自定义 yaml 做实验和调参。
    param_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='MPC gait controller parameter file'
    )

    # 定义要启动的 ROS2 节点
    # package/executable/name 必须与构建安装内容一致。
    node = Node(
        package='mpc_control',
        executable='mpc_gait_controller',
        name='mpc_gait_controller',
        # 输出到终端，便于查看步态切换与运行状态日志。
        output='screen',
        # 将 launch 参数 `param_file` 注入节点参数系统。
        parameters=[LaunchConfiguration('param_file')],
    )

    # 返回 LaunchDescription：先声明参数，再启动节点。
    return LaunchDescription([param_arg, node])
