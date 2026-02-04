# File: launch/gazebo.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess  # 新增导入
import os

def generate_launch_description():
    # 获取当前包路径
    robot_package_dir = get_package_share_directory('leg_model_1')

    # --- 按照评论修改的部分开始 ---
    # 替换原本的 IncludeLaunchDescription 方式
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    # --- 按照评论修改的部分结束 ---

    # 加载 URDF 文件路径
    urdf_file_path = os.path.join(robot_package_dir, 'urdf', 'leg_model_1.urdf')

    # 启动 Gazebo 模型生成器节点
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=[
            '-entity', 'robot',
            '-file', urdf_file_path,
        ],
        output='screen'
    )

    # 静态 TF 发布器
    tf_footprint_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    return LaunchDescription([
        gazebo,            # 对应修改后的变量
        spawn_entity_node,
        tf_footprint_base_node
    ])