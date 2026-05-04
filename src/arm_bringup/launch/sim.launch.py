import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float_arg(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _arm_controller_param(name, default):
    try:
        config_path = os.path.join(
            get_package_share_directory('arm_controller'),
            'config',
            'arm_controller.yaml',
        )
        with open(config_path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        return str(data['arm_controller_node']['ros__parameters'].get(name, default))
    except (KeyError, OSError, TypeError, yaml.YAMLError):
        return str(default)


def _write_box_sdf(path, model_name, size, color, mass=1.0, static=True):
    sx, sy, sz = size
    static_text = 'true' if static else 'false'
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    with open(path, 'w', encoding='utf-8') as file:
        file.write(f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>{static_text}</static>
    <link name="link">
      <inertial>
        <mass>{mass:.6f}</mass>
        <inertia>
          <ixx>{ixx:.9f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{iyy:.9f}</iyy>
          <iyz>0</iyz>
          <izz>{izz:.9f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")


def _spawn_scene_objects(context, *args, **kwargs):
    package_share = get_package_share_directory('arm_bringup')
    scene_dir = os.path.join(package_share, 'config', 'generated_scene')
    os.makedirs(scene_dir, exist_ok=True)

    platform_height = _float_arg(context, 'platform_height_m')
    platform_size_x = _float_arg(context, 'platform_size_x_m')
    platform_size_y = _float_arg(context, 'platform_size_y_m')
    platform_x = _float_arg(context, 'platform_x_m')
    box_width = _float_arg(context, 'box_width_m')
    box_depth = _float_arg(context, 'box_depth_m')
    box_height = _float_arg(context, 'box_height_m')
    box_center_x = _float_arg(context, 'arm_to_box_distance_m')
    box_y = _float_arg(context, 'box_y_m')

    platform_sdf = os.path.join(scene_dir, 'platform.sdf')
    box_sdf = os.path.join(scene_dir, 'box.sdf')

    _write_box_sdf(
        platform_sdf,
        'arm_platform',
        (platform_size_x, platform_size_y, platform_height),
        '0.32 0.32 0.34 1',
        mass=8.0,
        static=True,
    )
    _write_box_sdf(
        box_sdf,
        'target_box',
        (box_depth, box_width, box_height),
        '0.10 0.45 0.85 1',
        mass=0.5,
        static=False,
    )

    return [
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-file', platform_sdf,
                        '-entity', 'arm_platform',
                        '-x', f'{platform_x:.6f}',
                        '-y', '0.0',
                        '-z', f'{platform_height / 2.0:.6f}',
                        '-timeout', '300',
                    ],
                    output='screen',
                ),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-file', box_sdf,
                        '-entity', 'target_box',
                        '-x', f'{box_center_x:.6f}',
                        '-y', f'{box_y:.6f}',
                        '-z', f'{box_height / 2.0:.6f}',
                        '-timeout', '300',
                    ],
                    output='screen',
                ),
            ],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('arm_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    default_world = '/usr/share/gazebo-11/worlds/empty.world'

    urdf_path = os.path.join(package_share, 'urdf', 'robot_arm.urdf')
    generated_urdf_dir = os.path.join(package_share, 'config', 'description')
    generated_urdf_path = os.path.join(generated_urdf_dir, 'robot_arm.urdf')
    rviz_config_path = os.path.join(package_share, 'rviz', 'robot_arm.rviz')
    gazebo_launch_path = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    controller_config_path = PathJoinSubstitution(
        [FindPackageShare('sim_motor_controller'), 'config', 'sim_controllers.yaml']
    )

    os.makedirs(generated_urdf_dir, exist_ok=True)

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    generate_urdf = ExecuteProcess(
        cmd=['xacro ', urdf_path, ' -o ', generated_urdf_path],
        shell=True,
        output='screen',
    )

    rviz_config = LaunchConfiguration('rviz_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_sim_motor_controller = LaunchConfiguration('launch_sim_motor_controller')
    launch_arm_controller = LaunchConfiguration('launch_arm_controller')
    launch_scene_objects = LaunchConfiguration('launch_scene_objects')
    platform_height = LaunchConfiguration('platform_height_m')
    box_width = LaunchConfiguration('box_width_m')
    box_depth = LaunchConfiguration('box_depth_m')
    box_height = LaunchConfiguration('box_height_m')
    arm_to_box_distance = LaunchConfiguration('arm_to_box_distance_m')
    box_y = LaunchConfiguration('box_y_m')
    robot_base_bottom_offset = LaunchConfiguration('robot_base_bottom_offset_m')
    robot_spawn_clearance = LaunchConfiguration('robot_spawn_clearance_m')
    controller_manager_name = LaunchConfiguration('controller_manager_name')
    sim_motor_launch_path = os.path.join(
        get_package_share_directory('sim_motor_controller'),
        'launch',
        'sim_motor_controller.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Optional Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz together with Gazebo.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Absolute path to the RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'launch_sim_motor_controller',
            default_value='true',
            description='Launch the simulated MIT motor backend and ros2_control controllers.',
        ),
        DeclareLaunchArgument(
            'launch_arm_controller',
            default_value='true',
            description='Launch the target-point IK controller that publishes MIT motor commands.',
        ),
        DeclareLaunchArgument(
            'controller_manager_name',
            default_value='controller_manager',
            description='ROS name of the controller manager inside Gazebo.',
        ),
        DeclareLaunchArgument(
            'launch_scene_objects',
            default_value='true',
            description='Spawn the platform and target box scene objects.',
        ),
        DeclareLaunchArgument(
            'platform_height_m',
            default_value='0.16',
            description='Height of the platform under the robot, in meters.',
        ),
        DeclareLaunchArgument(
            'platform_size_x_m',
            default_value='0.30',
            description='Platform depth along the robot x axis, in meters.',
        ),
        DeclareLaunchArgument(
            'platform_size_y_m',
            default_value='0.30',
            description='Platform width along the robot y axis, in meters.',
        ),
        DeclareLaunchArgument(
            'platform_x_m',
            default_value='-0.04',
            description='Platform center x position in the world, in meters.',
        ),
        DeclareLaunchArgument(
            'arm_to_box_distance_m',
            default_value=_arm_controller_param('arm_to_box_distance_m', 0.25),
            description='Forward distance from the robot base to the center of the target box.',
        ),
        DeclareLaunchArgument(
            'box_width_m',
            default_value=_arm_controller_param('box_width_m', 0.25),
            description='Target box width along y, in meters.',
        ),
        DeclareLaunchArgument(
            'box_depth_m',
            default_value=_arm_controller_param('box_depth_m', 0.25),
            description='Target box depth along x, in meters.',
        ),
        DeclareLaunchArgument(
            'box_height_m',
            default_value=_arm_controller_param('box_height_m', 0.25),
            description='Target box height, in meters.',
        ),
        DeclareLaunchArgument(
            'box_y_m',
            default_value=_arm_controller_param('box_motion_y_m', 0.0),
            description='Target box center y position in the world, in meters.',
        ),
        DeclareLaunchArgument(
            'robot_base_bottom_offset_m',
            default_value='0.006',
            description='How far the base_link origin is above the bottom of the base collision mesh.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_clearance_m',
            default_value='0.002',
            description='Small clearance above the platform when spawning the robot.',
        ),
        generate_urdf,
        OpaqueFunction(
            function=_spawn_scene_objects,
            condition=IfCondition(launch_scene_objects),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_motor_launch_path),
            condition=IfCondition(launch_sim_motor_controller),
        ),
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
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf_path],
            condition=UnlessCondition(launch_sim_motor_controller),
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'robot_arm',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z',
                        PythonExpression(
                            [platform_height, ' + ', robot_base_bottom_offset, ' + ', robot_spawn_clearance]
                        ),
                        '-timeout', '300',
                    ],
                    output='screen',
                ),
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'joint_state_broadcaster',
                        '-c',
                        controller_manager_name,
                        '-p',
                        controller_config_path,
                        '--controller-manager-timeout',
                        '60.0',
                    ],
                    condition=IfCondition(launch_sim_motor_controller),
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    output='screen',
                    arguments=[
                        'arm_position_controller',
                        '-c',
                        controller_manager_name,
                        '-p',
                        controller_config_path,
                        '--controller-manager-timeout',
                        '60.0',
                    ],
                    condition=IfCondition(launch_sim_motor_controller),
                ),
            ],
        ),
        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('arm_controller'), 'config', 'arm_controller.yaml']
                ),
                {
                    'use_sim_time': True,
                    'platform_height_m': ParameterValue(platform_height, value_type=float),
                    'box_width_m': ParameterValue(box_width, value_type=float),
                    'box_depth_m': ParameterValue(box_depth, value_type=float),
                    'box_height_m': ParameterValue(box_height, value_type=float),
                    'box_center_distance_m': ParameterValue(arm_to_box_distance, value_type=float),
                    'arm_to_box_distance_m': ParameterValue(arm_to_box_distance, value_type=float),
                    'box_motion_y_m': ParameterValue(box_y, value_type=float),
                },
            ],
            condition=IfCondition(launch_arm_controller),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
            parameters=[{'use_sim_time': True}],
        ),
    ])
