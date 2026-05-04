import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('arm_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    default_world = '/usr/share/gazebo-11/worlds/empty.world'

    urdf_path = os.path.join(package_share, 'urdf', 'robot_arm.urdf')
    generated_urdf_dir = os.path.join(package_share, 'config', 'description')
    generated_urdf_path = os.path.join(generated_urdf_dir, 'robot_arm.urdf')
    gazebo_launch_path = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    os.makedirs(generated_urdf_dir, exist_ok=True)

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    generate_urdf = ExecuteProcess(cmd=['xacro ', urdf_path, ' -o ', generated_urdf_path], shell=True, output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Optional Gazebo world file.',
        ),
        generate_urdf,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'robot_arm',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.02',
                        '-timeout', '300',
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
