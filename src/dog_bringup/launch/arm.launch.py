import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arm_movelt_pkg = get_package_share_directory("arm_movelt")

    arm_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_movelt_pkg, "launch", "real_robot.launch.py")
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("arm_serial_port"),
            "serial_baudrate": LaunchConfiguration("arm_serial_baudrate"),
            "timeout_ms": LaunchConfiguration("arm_timeout_ms"),
            "retry_count": LaunchConfiguration("arm_retry_count"),
            "use_moveit": LaunchConfiguration("arm_use_moveit"),
            "use_rviz": "false",
            "suction_relay1_port": LaunchConfiguration("suction_relay1_port"),
            "suction_relay2_port": LaunchConfiguration("suction_relay2_port"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_serial_port", default_value="/dev/arm_motor"),
            DeclareLaunchArgument("arm_serial_baudrate", default_value="921600"),
            DeclareLaunchArgument("arm_timeout_ms", default_value="10"),
            DeclareLaunchArgument("arm_retry_count", default_value="3"),
            DeclareLaunchArgument("arm_use_moveit", default_value="true"),
            DeclareLaunchArgument("suction_relay1_port", default_value="/dev/arm_relay1"),
            DeclareLaunchArgument("suction_relay2_port", default_value="/dev/arm_relay2"),
            arm_stack,
        ]
    )
