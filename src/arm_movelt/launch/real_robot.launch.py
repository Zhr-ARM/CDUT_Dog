from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    pkg_arm_control = get_package_share_directory("arm_control")
    pkg_arm_model = get_package_share_directory("arm_model")
    pkg_arm_movelt = get_package_share_directory("arm_movelt")
    pkg_suction = get_package_share_directory("suction_controller")

    xacro_file = os.path.join(pkg_arm_model, "urdf", "robot_arm.real.xacro")
    controllers_file = os.path.join(pkg_arm_movelt, "config", "ros2_controllers.yaml")
    box_motion_file = os.path.join(pkg_arm_control, "config", "box_motion.yaml")
    suction_config_file = os.path.join(pkg_suction, "config", "suction_controller.yaml")

    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    timeout_ms = LaunchConfiguration("timeout_ms")
    retry_count = LaunchConfiguration("retry_count")
    use_moveit = LaunchConfiguration("use_moveit")
    use_rviz = LaunchConfiguration("use_rviz")
    suction_relay1_port = LaunchConfiguration("suction_relay1_port")
    suction_relay2_port = LaunchConfiguration("suction_relay2_port")

    xacro_mappings = {
        "serial_port": serial_port,
        "serial_baudrate": serial_baudrate,
        "timeout_ms": timeout_ms,
        "retry_count": retry_count,
    }

    robot_description_content = Command(
        [
            "xacro ",
            xacro_file,
            " serial_port:=",
            serial_port,
            " serial_baudrate:=",
            serial_baudrate,
            " timeout_ms:=",
            timeout_ms,
            " retry_count:=",
            retry_count,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="arm_movelt")
        .robot_description(file_path=xacro_file, mappings=xacro_mappings)
        .to_moveit_configs()
    )
    trajectory_execution_parameters = {
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/arm_motor",
                description="DM-J4340 USB-CAN 适配器串口设备。",
            ),
            DeclareLaunchArgument(
                "serial_baudrate",
                default_value="921600",
                description="DM-J4340 USB-CAN 适配器串口波特率。",
            ),
            DeclareLaunchArgument(
                "timeout_ms",
                default_value="10",
                description="硬件接口等待单帧反馈的超时时间，单位 ms。",
            ),
            DeclareLaunchArgument(
                "retry_count",
                default_value="3",
                description="硬件接口读写请求的重试次数。",
            ),
            DeclareLaunchArgument(
                "use_moveit",
                default_value="true",
                description="是否启动 move_group。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="是否启动 MoveIt RViz。",
            ),
            DeclareLaunchArgument(
                "suction_relay1_port",
                default_value="/dev/arm_relay1",
                description="吸盘继电器（继电器1）串口设备路径。",
            ),
            DeclareLaunchArgument(
                "suction_relay2_port",
                default_value="/dev/arm_relay2",
                description="气阀继电器（继电器2）串口设备路径。",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_file],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "arm_controller",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_config.to_dict(), trajectory_execution_parameters],
                output="screen",
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="arm_control",
                executable="moveit_plan_service",
                name="moveit_plan_service",
                parameters=[moveit_config.to_dict(), trajectory_execution_parameters],
                output="screen",
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="arm_control",
                executable="target_to_moveit",
                name="target_to_moveit",
                output="screen",
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="arm_control",
                executable="box_motion_node",
                name="box_motion_node",
                parameters=[box_motion_file],
                output="screen",
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="suction_controller",
                executable="suction_controller_node",
                name="suction_controller_node",
                output="screen",
                parameters=[
                    suction_config_file,
                    {
                        "relay1_port": suction_relay1_port,
                        "relay2_port": suction_relay2_port,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(pkg_arm_movelt, "config", "moveit.rviz"),
                ],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
