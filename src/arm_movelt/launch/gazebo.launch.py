from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    pkg_arm_model = get_package_share_directory('arm_model')
    pkg_arm_movelt = get_package_share_directory('arm_movelt')
    
    xacro_file = os.path.join(pkg_arm_model, 'urdf', 'robot_arm.gazebo.xacro')
    robot_description = Command(['xacro ', xacro_file])

    controllers_file = os.path.join(pkg_arm_model, 'config', 'ros2_gazebo_control.yaml')
    gazebo_model_paths = [os.path.dirname(pkg_arm_model)]
    if os.environ.get('GAZEBO_MODEL_PATH'):
        gazebo_model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    gazebo_client_env = {
        'GAZEBO_MODEL_DATABASE_URI': '',
        'GAZEBO_MODEL_PATH': os.pathsep.join(gazebo_model_paths),
    }

    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_movelt").to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run Gazebo without the GUI.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'pause': 'false',
            }.items()
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            additional_env=gazebo_client_env, # type: ignore
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot_arm', '-topic', 'robot_description'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster','--controller-manager','/controller_manager','--ros-args','--params-file', controllers_file],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller','--controller-manager','/controller_manager','--ros-args','--params-file', controllers_file],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_arm_movelt, 'launch', 'move_group.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_arm_movelt, 'launch', 'moveit_rviz.launch.py')
            ),
        ),
        Node(
            package='arm_control',
            executable='target_to_moveit',
            name='target_to_moveit',
            parameters=[
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        Node(
            package='arm_control',
            executable='moveit_plan_service',
            name='moveit_plan_service',
            parameters=[
            moveit_config.to_dict(),
                {'use_sim_time': True}
            ],
            output='screen'
        ),
    ])
