import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dog_teleop_node = Node(
        package='dog_teleop',
        executable='dog_teleop_node',
        name='dog_teleop',
        output='screen',
        parameters=[{
            'joy_topic': '/joy',
            'cmd_topic': '/cmd_vel',
            'max_linear_vel': 1.0,
            'max_angular_vel': 1.0,
        }],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        dog_teleop_node,
    ])
