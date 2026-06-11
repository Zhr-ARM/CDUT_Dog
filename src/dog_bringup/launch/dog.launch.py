import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory("dog_bringup")
    teleop_pkg = get_package_share_directory("dog_teleop")
    motor_param_file = os.path.join(
        get_package_share_directory("deep_motor_ros"),
        "config",
        "four_can_real_robot.yaml",
    )
    gait_param_file = os.path.join(
        get_package_share_directory("dog_position_control"),
        "config",
        "gait_controller_real_stand.yaml",
    )
    teleop_param_file = os.path.join(teleop_pkg, "config", "dog_teleop.yaml")

    dog_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, "launch", "real.launch.py")),
        launch_arguments={
            "motor_param_file": LaunchConfiguration("motor_param_file"),
            "gait_param_file": LaunchConfiguration("gait_param_file"),
            "launch_simulation": LaunchConfiguration("launch_simulation"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_teleop": LaunchConfiguration("launch_teleop"),
            "teleop_param_file": LaunchConfiguration("teleop_param_file"),
            "launch_imu": LaunchConfiguration("launch_imu"),
            "imu_type": LaunchConfiguration("imu_type"),
            "imu_port": LaunchConfiguration("imu_port"),
            "imu_baud": LaunchConfiguration("imu_baud"),
            "imu_frame_id": LaunchConfiguration("imu_frame_id"),
            "shared_command_topic": LaunchConfiguration("shared_command_topic"),
            "hardware_joint_state_topic": LaunchConfiguration("hardware_joint_state_topic"),
            "hardware_feedback_topic": LaunchConfiguration("hardware_feedback_topic"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("motor_param_file", default_value=motor_param_file),
            DeclareLaunchArgument("gait_param_file", default_value=gait_param_file),
            DeclareLaunchArgument("launch_simulation", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("launch_teleop", default_value="true"),
            DeclareLaunchArgument("teleop_param_file", default_value=teleop_param_file),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument("imu_type", default_value="normal"),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyimu"),
            DeclareLaunchArgument("imu_baud", default_value="9600"),
            DeclareLaunchArgument("imu_frame_id", default_value="base_link"),
            DeclareLaunchArgument("shared_command_topic", default_value="/motor_cmd"),
            DeclareLaunchArgument("hardware_joint_state_topic", default_value="/real/joint_states"),
            DeclareLaunchArgument("hardware_feedback_topic", default_value="/real/motor_feedback"),
            dog_stack,
        ]
    )
