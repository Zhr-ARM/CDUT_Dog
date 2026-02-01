import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    gazebo_pkg = get_package_share_directory("gazebo_ros")
    pkg = get_package_share_directory("dog_bringup")

    xacro_file = os.path.join(pkg, "urdf", "dog.xacro")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", xacro_file]), value_type=str
                )
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "dog_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.5",
        ],
        output="screen",
    )
    test_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["test_controller"],
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
    )

    load_imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
    )

    ld = LaunchDescription()

    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    ld.add_action(load_joint_state_broadcaster)
    # ld.add_action(load_effort_controller)
    ld.add_action(test_controller)
    ld.add_action(load_imu_broadcaster)

    return ld
