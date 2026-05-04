from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    auto_set_zero_on_start = LaunchConfiguration("auto_set_zero_on_start")
    auto_enable_on_start = LaunchConfiguration("auto_enable_on_start")
    publish_robot_joint_states = LaunchConfiguration("publish_robot_joint_states")
    config_file = PathJoinSubstitution(
        [FindPackageShare("real_motor_controller"), "config", "real_motor_controller.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("auto_set_zero_on_start", default_value="true"),
            DeclareLaunchArgument("auto_enable_on_start", default_value="true"),
            DeclareLaunchArgument("publish_robot_joint_states", default_value="true"),
            Node(
                package="real_motor_controller",
                executable="real_motor_controller_node",
                name="real_motor_controller_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "serial_port": serial_port,
                        "auto_set_zero_on_start": ParameterValue(
                            auto_set_zero_on_start, value_type=bool
                        ),
                        "auto_enable_on_start": ParameterValue(
                            auto_enable_on_start, value_type=bool
                        ),
                        "publish_robot_joint_states": ParameterValue(
                            publish_robot_joint_states, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
