import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    serial_port = LaunchConfiguration("serial_port")
    launch_arm_controller = LaunchConfiguration("launch_arm_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    sim_launch = PathJoinSubstitution(
        [FindPackageShare("sim_motor_controller"), "launch", "sim_motor_controller.launch.py"]
    )
    real_launch = PathJoinSubstitution(
        [
            FindPackageShare("real_motor_controller"),
            "launch",
            "real_motor_controller.launch.py",
        ]
    )

    # URDF for robot_state_publisher and RViz
    package_share = get_package_share_directory("arm_bringup")
    urdf_path = os.path.join(package_share, "urdf", "robot_arm.urdf")
    default_rviz_config = os.path.join(package_share, "rviz", "robot_arm.rviz")
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="sim",
                description="Motor backend: sim or real.",
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "launch_arm_controller",
                default_value="true",
                description="Launch the target-point IK controller that publishes MIT motor commands.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz together with the robot driver.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Absolute path to the RViz configuration file.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                condition=IfCondition(PythonExpression(["'", backend, "' == 'sim'"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
                launch_arguments={"serial_port": serial_port}.items(),
                condition=IfCondition(PythonExpression(["'", backend, "' == 'real'"])),
            ),
            Node(
                package="arm_controller",
                executable="arm_controller_node",
                name="arm_controller_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("arm_controller"), "config", "arm_controller.yaml"]
                    ),
                    {"gravity_compensation.enabled": False},
                ],
                condition=IfCondition(launch_arm_controller),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
