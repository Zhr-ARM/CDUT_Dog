import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("dog_bringup")
    teleop_pkg = get_package_share_directory("dog_teleop")
    imu_pkg = get_package_share_directory("dog_imu")
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
    teleop_param_file = os.path.join(
        teleop_pkg,
        "config",
        "dog_teleop.yaml",
    )

    launch_simulation = LaunchConfiguration("launch_simulation")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_teleop = LaunchConfiguration("launch_teleop")
    launch_imu = LaunchConfiguration("launch_imu")
    shared_command_topic = LaunchConfiguration("shared_command_topic")
    hardware_joint_state_topic = LaunchConfiguration("hardware_joint_state_topic")
    hardware_feedback_topic = LaunchConfiguration("hardware_feedback_topic")

    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "launch_gait_controller": "false",
            "gait_param_file": LaunchConfiguration("gait_param_file"),
            "gait_use_sim_time": "true",
        }.items(),
        condition=IfCondition(launch_simulation),
    )

    teleop_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, "launch", "dog_teleop.launch.py")
        ),
        launch_arguments={
            "param_file": LaunchConfiguration("teleop_param_file"),
        }.items(),
        condition=IfCondition(launch_teleop),
    )

    imu_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg, "launch", "wit_imu.launch.py")
        ),
        launch_arguments={
            "type": LaunchConfiguration("imu_type"),
            "port": LaunchConfiguration("imu_port"),
            "baud": LaunchConfiguration("imu_baud"),
            "frame_id": LaunchConfiguration("imu_frame_id"),
        }.items(),
        condition=IfCondition(launch_imu),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motor_param_file",
                default_value=motor_param_file,
                description="Motor driver parameter file",
            ),
            DeclareLaunchArgument(
                "gait_param_file",
                default_value=gait_param_file,
                description="Quadruped gait controller parameter file",
            ),
            DeclareLaunchArgument(
                "launch_simulation",
                default_value="false",
                description="Launch Gazebo together with the real robot control stack.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz when the Gazebo stack is enabled.",
            ),
            DeclareLaunchArgument(
                "launch_teleop",
                default_value="true",
                description="Launch joystick teleop with the real robot control stack.",
            ),
            DeclareLaunchArgument(
                "teleop_param_file",
                default_value=teleop_param_file,
                description="Dog teleop and joy_node parameter file.",
            ),
            DeclareLaunchArgument(
                "launch_imu",
                default_value="true",
                description="Launch the WIT IMU driver with the real robot control stack.",
            ),
            DeclareLaunchArgument(
                "imu_type",
                default_value="normal",
                description="IMU protocol: normal | modbus | hmodbus | can | hcan.",
            ),
            DeclareLaunchArgument(
                "imu_port",
                default_value="/dev/ttyACM0",
                description="IMU serial port path.",
            ),
            DeclareLaunchArgument(
                "imu_baud",
                default_value="9600",
                description="IMU serial baud rate.",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="base_link",
                description="IMU message frame_id.",
            ),
            DeclareLaunchArgument(
                "shared_command_topic",
                default_value="/motor_cmd",
                description="Shared MIT command topic consumed by both Gazebo and hardware.",
            ),
            DeclareLaunchArgument(
                "hardware_joint_state_topic",
                default_value="/real/joint_states",
                description="Real robot joint state topic used by the gait controller.",
            ),
            DeclareLaunchArgument(
                "hardware_feedback_topic",
                default_value="/real/motor_feedback",
                description="Real robot feedback topic.",
            ),
            sim_stack,
            teleop_stack,
            imu_stack,
            Node(
                package="deep_motor_ros",
                executable="motor_node",
                name="deep_motor_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("motor_param_file"),
                    {
                        "command_topic": shared_command_topic,
                        "joint_state_topic": hardware_joint_state_topic,
                        "feedback_topic": hardware_feedback_topic,
                    },
                ],
            ),
            Node(
                package="dog_position_control",
                executable="quadruped_gait_controller",
                name="quadruped_gait_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("gait_param_file"),
                    {
                        "use_sim_time": False,
                        "command_topic": shared_command_topic,
                        "joint_state_topic": hardware_joint_state_topic,
                    },
                ],
            ),
        ]
    )
