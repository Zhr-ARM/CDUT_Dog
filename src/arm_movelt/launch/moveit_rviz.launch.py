from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_movelt").to_moveit_configs()
    # 注入 use_sim_time 参数，使 rviz 使用仿真时钟而非系统时钟
    moveit_config.robot_description['use_sim_time'] = True
    return generate_moveit_rviz_launch(moveit_config)
