from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    serial_port = LaunchConfiguration("serial_port")
    launch_arm_controller = LaunchConfiguration("launch_arm_controller")

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
        ]
    )
