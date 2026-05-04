import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('arm_bringup')

    urdf_path = os.path.join(package_share, 'urdf', 'robot_arm.urdf')
    generated_urdf_dir = os.path.join(package_share, 'config', 'description')
    generated_urdf_path = os.path.join(generated_urdf_dir, 'robot_arm.urdf')
    rviz_config_path = os.path.join(package_share, 'rviz', 'robot_arm.rviz')

    os.makedirs(generated_urdf_dir, exist_ok=True)

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    generate_urdf = ExecuteProcess(
        cmd=['xacro ', urdf_path, ' -o ', generated_urdf_path],
        shell=True,
        output='screen',
    )

    gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start joint_state_publisher_gui when true.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Absolute path to the RViz configuration file.',
        ),
        generate_urdf,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[generated_urdf_path],
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[generated_urdf_path],
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
