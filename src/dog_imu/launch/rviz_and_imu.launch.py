# -*- coding: utf-8 -*-
"""Launch IMU driver + RViz2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dog_imu')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'wit_ros2_imu.rviz')

    type_arg = DeclareLaunchArgument(
        'type', default_value='normal',
        description='IMU protocol: normal | modbus | hmodbus | can | hcan')
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyACM0')
    baud_arg = DeclareLaunchArgument('baud', default_value='9600')
    frame_arg = DeclareLaunchArgument('frame_id', default_value='base_link')

    executable = PythonExpression([
        "'wit_' + '", LaunchConfiguration('type'), "' + '_node'"
    ])

    imu_node = Node(
        package='dog_imu',
        executable=executable,
        name='wit_imu',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([type_arg, port_arg, baud_arg, frame_arg, imu_node, rviz_node])
