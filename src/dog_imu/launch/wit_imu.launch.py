# -*- coding: utf-8 -*-
"""Launch the WIT IMU driver only.

type: normal | modbus | hmodbus | can | hcan
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='normal',
        description='IMU protocol: normal | modbus | hmodbus | can | hcan'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='Serial port path')
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='9600',
        description='Serial baud rate')
    frame_arg = DeclareLaunchArgument(
        'frame_id', default_value='base_link',
        description='IMU frame_id')

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

    return LaunchDescription([type_arg, port_arg, baud_arg, frame_arg, imu_node])
