import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    bringup_pkg = get_package_share_directory("dog_bringup")
    gait_pkg = get_package_share_directory("dog_position_control")

    xacro_file = os.path.join(bringup_pkg, "xacro", "cdut_dog", "dog.xacro")
    urdf_output_dir = os.path.join(bringup_pkg, "config", "description")
    urdf_output_file = os.path.join(urdf_output_dir, "dog.urdf")
    world_file = os.path.join(bringup_pkg, "config", "dog.world")
    rviz_config_file = os.path.join(bringup_pkg, "config", "dog.rviz")
    default_gait_param_file = os.path.join(gait_pkg, "config", "gait_controller_sim_trot.yaml")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_gait_controller = LaunchConfiguration("launch_gait_controller")
    gait_param_file = LaunchConfiguration("gait_param_file")
    gait_use_sim_time = LaunchConfiguration("gait_use_sim_time")

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    generate_urdf = ExecuteProcess(
        cmd=[
            "mkdir -p ",
            urdf_output_dir,
            " && xacro ",
            xacro_file,
            " -o ",
            urdf_output_file,
        ],
        shell=True,
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments=[("world", world_file), ("verbose", "true")],
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
            "0.09",
            "-timeout",
            "300",
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    mit_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mit_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gait_controller = Node(
        package="dog_position_control",
        executable="quadruped_gait_controller",
        name="quadruped_gait_controller",
        output="screen",
        parameters=[
            gait_param_file,
            {"use_sim_time": ParameterValue(gait_use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(launch_gait_controller),
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz together with Gazebo.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "launch_gait_controller",
            default_value="true",
            description="Launch the quadruped gait controller inside the sim stack.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "gait_param_file",
            default_value=default_gait_param_file,
            description="Quadruped gait controller parameter file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "gait_use_sim_time",
            default_value="true",
            description="Whether the sim gait controller should use the simulation clock.",
        )
    )
    ld.add_action(generate_urdf)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(mit_controller)
    ld.add_action(gait_controller)
    return ld
