from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_movelt").to_moveit_configs()
    # 注入 use_sim_time 参数
    moveit_config.robot_description['use_sim_time'] = True
    return generate_rsp_launch(moveit_config)
