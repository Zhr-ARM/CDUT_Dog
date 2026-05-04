from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_enable_on_start = LaunchConfiguration("auto_enable_on_start")
    config_file = PathJoinSubstitution(
        [FindPackageShare("sim_motor_controller"), "config", "sim_motor_controller.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_enable_on_start", default_value="true"),
            Node(
                package="sim_motor_controller",
                executable="sim_motor_controller_node",
                name="sim_motor_controller_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "auto_enable_on_start": ParameterValue(
                            auto_enable_on_start, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
