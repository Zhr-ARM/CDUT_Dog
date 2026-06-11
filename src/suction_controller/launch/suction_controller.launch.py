from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "suction_relay1_port",
                default_value="/dev/arm_relay1",
                description="Serial port for suction cup relay (relay1).",
            ),
            DeclareLaunchArgument(
                "suction_relay2_port",
                default_value="/dev/arm_relay2",
                description="Serial port for air valve relay (relay2).",
            ),
            Node(
                package="suction_controller",
                executable="suction_controller_node",
                name="suction_controller_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("suction_controller"), "config", "suction_controller.yaml"]
                    ),
                    {
                        "relay1_port": LaunchConfiguration("suction_relay1_port"),
                        "relay2_port": LaunchConfiguration("suction_relay2_port"),
                    },
                ],
            ),
        ]
    )
