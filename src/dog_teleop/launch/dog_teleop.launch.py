import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory('dog_teleop'),
        'config',
        'dog_teleop.yaml',
    )
    param_file = LaunchConfiguration('param_file')

    dog_teleop_node = Node(
        package='dog_teleop',
        executable='dog_teleop_node',
        name='dog_teleop',
        output='screen',
        parameters=[param_file],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param_file,
            description='Dog teleop and joy_node parameter file',
        ),
        joy_node,
        dog_teleop_node,
    ])
