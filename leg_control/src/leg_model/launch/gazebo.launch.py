# File: launch/gazebo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
import re
import tempfile


def generate_launch_description():
    robot_package_dir = get_package_share_directory('leg_model')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI (gzclient). Keep false for headless servers.',
    )

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    urdf_file_path = os.path.join(robot_package_dir, 'urdf', 'leg_model.urdf')
    controllers_file = os.path.join(robot_package_dir, 'config', 'ros2_controllers.yaml')

    with open(urdf_file_path, 'r', encoding='utf-8') as f:
        urdf_text = f.read()

    gazebo_urdf_text = urdf_text.replace(
        'package://leg_model/config/ros2_controllers.yaml',
        controllers_file,
    )
    gazebo_urdf_text = gazebo_urdf_text.replace(
        'package://leg_model/',
        f'file://{robot_package_dir}/',
    )

    gazebo_urdf_text = re.sub(r"<\?xml[^>]*\?>|<!--.*?-->", "", gazebo_urdf_text, flags=re.DOTALL)
    gazebo_urdf_text = gazebo_urdf_text.strip()

    gazebo_urdf_file_path = os.path.join(tempfile.gettempdir(), 'leg_model_gazebo.urdf')
    with open(gazebo_urdf_file_path, 'w', encoding='utf-8') as f:
        f.write(gazebo_urdf_text)

    robot_description = gazebo_urdf_text
    robot_description = robot_description.replace('"', "'")
    robot_description = re.sub(r"<\?xml[^>]*\?>|<!--.*?-->", "", robot_description, flags=re.DOTALL)
    robot_description = re.sub(r"\s+", " ", robot_description).strip()

    robot_description_server_node = Node(
        package='leg_model',
        executable='robot_description_server.py',
        name='robot_description_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'robot',
            '-file', gazebo_urdf_file_path,
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster',
            '--param-file', controllers_file,
        ],
        output='screen',
    )

    mit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mit_controller',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'leg_mit_controller/MitController',
            '--param-file', controllers_file,
        ],
        output='screen',
    )

    tf_footprint_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    spawn_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                mit_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        gui_arg,
        gzserver,
        gzclient,
        robot_description_server_node,
        robot_state_publisher_node,
        spawn_entity_node,
        spawn_controllers_after_spawn,
        tf_footprint_base_node
    ])
