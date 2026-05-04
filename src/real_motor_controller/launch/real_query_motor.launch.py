from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    config_file = PathJoinSubstitution(
        [FindPackageShare("real_motor_controller"), "config", "real_query_motor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            Node(
                package="real_motor_controller",
                executable="real_query_motor_node",
                name="real_query_motor_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "serial_port": serial_port,
                    }
                ],
            ),
        ]
    )
