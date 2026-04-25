from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    config_file = PathJoinSubstitution(
        [FindPackageShare("dm_j4340_quad"), "config", "query_motor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            Node(
                package="dm_j4340_quad",
                executable="query_motor_node",
                name="query_motor_node",
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
