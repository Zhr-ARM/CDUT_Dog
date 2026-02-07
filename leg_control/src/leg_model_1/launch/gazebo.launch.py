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
    # 获取当前包路径
    robot_package_dir = get_package_share_directory('leg_model_1')

    # 替换原本的 IncludeLaunchDescription 方式
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

    # ---------------------------------------------------------
    # 1. 准备文件路径
    # ---------------------------------------------------------
    # 获取 URDF 模型文件和控制器配置文件(YAML)在系统中的绝对路径
    urdf_file_path = os.path.join(robot_package_dir, 'urdf', 'leg_model_1.urdf')
    controllers_file = os.path.join(robot_package_dir, 'config', 'ros2_controllers.yaml')

    # ---------------------------------------------------------
    # 2. 读取原始模型
    # ---------------------------------------------------------
    # 将原始 URDF 文件的内容全部读取到内存字符串 urdf_text 中
    with open(urdf_file_path, 'r', encoding='utf-8') as f:
        urdf_text = f.read()

    # ---------------------------------------------------------
    # 3. 动态补丁 (核心逻辑)
    # ---------------------------------------------------------
    # 问题背景：URDF 文件中写的是 <parameters>package://...</parameters>
    # 兼容性修复：将 "package://" 格式的 URI 强制替换为本次运行时的绝对文件路径。
    # 如果不这样做，gazebo_ros2_control 插件可能无法找到配置文件，导致控制器无法加载。
    gazebo_urdf_text = urdf_text.replace(
        'package://leg_model_1/config/ros2_controllers.yaml',
        controllers_file,
    )

    # ---------------------------------------------------------
    # 4. 生成临时 URDF 文件
    # ---------------------------------------------------------
    # 将替换路径后的新内容写入到系统的临时文件夹中 (/tmp/...)。
    # 这个临时文件将专门传给 "spawn_entity" 节点来在 Gazebo 中生成机器人。
    gazebo_urdf_file_path = os.path.join(tempfile.gettempdir(), 'leg_model_1_gazebo.urdf')
    with open(gazebo_urdf_file_path, 'w', encoding='utf-8') as f:
        f.write(gazebo_urdf_text)

    # ---------------------------------------------------------
    # 5. 变量准备 (robot_description)
    # ---------------------------------------------------------
    # 复用已经替换过路径的文本，避免重复操作。
    # 后续代码会对其进行格式压缩处理，以便作为参数传递给 robot_state_publisher 节点。
    robot_description = gazebo_urdf_text

    # gazebo_ros2_control 内部会把 robot_description 作为命令行参数传给 rcl，
    # 多行 XML 会触发 rcl 参数解析失败。这里进行一站式压缩处理：
    # 1. 替换 " 为 ' 避免 Shell/YAML 解析错误
    # 2. 去除 XML 声明和注释
    # 3. 将所有空白字符（换行、制表符、多空格）压缩为单个空格
    robot_description = robot_description.replace('"', "'")
    robot_description = re.sub(r"<\?xml[^>]*\?>|<!--.*?-->", "", robot_description, flags=re.DOTALL)
    robot_description = re.sub(r"\s+", " ", robot_description).strip()

    robot_description_server_node = Node(
        package='leg_model_1',
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

    # 启动 Gazebo 模型生成器节点
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

    # 静态 TF 发布器
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