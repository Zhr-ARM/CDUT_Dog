"""
Launch gravity-compensated arm control (zero stiffness, drag-to-teach).

Usage:
  ros2 launch arm_bringup gravity_compensation.launch.py backend:=real serial_port:=/dev/ttyACM0
  ros2 launch arm_bringup gravity_compensation.launch.py backend:=sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    backend = LaunchConfiguration("backend")
    serial_port = LaunchConfiguration("serial_port")
    launch_rviz = LaunchConfiguration("launch_rviz")

    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sim_motor_controller"), "launch", "sim_motor_controller.launch.py"]
    )
    real_launch = PathJoinSubstitution(
        [FindPackageShare("real_motor_controller"), "launch", "real_motor_controller.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend",
            default_value="real",
            description="Motor backend: sim or real.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="USB-CAN serial port (real backend only).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization.",
        ),

        # ── URDF 生成 ──
        generate_urdf,

        # ── robot_state_publisher：读取 /joint_states，广播 TF ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # ⚠️ 注意：此处不启动 joint_state_publisher / joint_state_publisher_gui！
        # real_motor_controller_node 已直接将电机反馈发布到 /joint_states，
        # 同时启动 joint_state_publisher 会导致零位覆盖真实关节值，造成 RViz
        # 无法实时显示关节运动。

        # ── 电机后端（sim 或 real） ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            condition=IfCondition(PythonExpression(["'", backend, "' == 'sim'"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={"serial_port": serial_port}.items(),
            condition=IfCondition(PythonExpression(["'", backend, "' == 'real'"])),
        ),

        # ── arm_controller（重力补偿模式） ──
        Node(
            package="arm_controller",
            executable="arm_controller_node",
            name="arm_controller_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare("arm_controller"), "config", "arm_controller.yaml"]
                ),
                {"gravity_compensation.enabled": True},
            ],
        ),

        # ── RViz 可视化 ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(launch_rviz),
        ),
    ])
