import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_pkg = get_package_share_directory("dog_bringup")

    publish_cmd_vel = LaunchConfiguration("publish_cmd_vel")
    crouch_on_arrival = LaunchConfiguration("crouch_on_arrival")
    arrival_action = LaunchConfiguration("arrival_action")
    standoff_distance = LaunchConfiguration("standoff_distance")
    linear_tolerance = LaunchConfiguration("linear_tolerance")
    yaw_tolerance = LaunchConfiguration("yaw_tolerance")
    drive_mode = LaunchConfiguration("drive_mode")
    enable_yaw_control = LaunchConfiguration("enable_yaw_control")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    target_frame_mode = LaunchConfiguration("target_frame_mode")
    lock_target = LaunchConfiguration("lock_target")
    sensor_forward_offset = LaunchConfiguration("sensor_forward_offset")
    target_timeout = LaunchConfiguration("target_timeout")
    launch_sim = LaunchConfiguration("launch_sim")

    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "launch_rviz": "true",
            "launch_gait_controller": publish_cmd_vel,
            "gait_use_sim_time": "true",
        }.items(),
        condition=IfCondition(launch_sim),
    )

    nav_preview = Node(
        package="dog_vision",
        executable="box_nav_preview_node",
        name="box_nav_preview_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_pose_topic": target_pose_topic,
                "odom_topic": "/ground_truth",
                "path_topic": "/navigation/preview_path",
                "marker_topic": "/navigation/preview_markers",
                "cmd_vel_topic": "/cmd_vel",
                "gait_action_topic": "/gait_action",
                "status_topic": "/navigation/nav_status",
                "publish_cmd_vel": ParameterValue(publish_cmd_vel, value_type=bool),
                "crouch_on_arrival": ParameterValue(crouch_on_arrival, value_type=bool),
                "arrival_action": arrival_action,
                "control_mode": "odom",
                "drive_mode": drive_mode,
                "enable_yaw_control": ParameterValue(
                    enable_yaw_control, value_type=bool
                ),
                "fixed_frame": "odom",
                "target_frame_mode": target_frame_mode,
                "lock_target": ParameterValue(lock_target, value_type=bool),
                "sensor_forward_offset": ParameterValue(
                    sensor_forward_offset, value_type=float
                ),
                "target_timeout": ParameterValue(target_timeout, value_type=float),
                "standoff_distance": ParameterValue(standoff_distance, value_type=float),
                "linear_tolerance": ParameterValue(linear_tolerance, value_type=float),
                "yaw_tolerance": ParameterValue(yaw_tolerance, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_sim",
                default_value="true",
                description="Launch Gazebo and RViz for previewing the real target path.",
            ),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/navigation/target_pose",
                description="PoseStamped topic published by the real box locator.",
            ),
            DeclareLaunchArgument(
                "target_frame_mode",
                default_value="local",
                description="local: target pose is in robot/sensor coordinates; fixed: already in odom/map; auto: infer from frame_id.",
            ),
            DeclareLaunchArgument(
                "lock_target",
                default_value="true",
                description="Lock the first real target pose into the sim frame instead of reprojecting every new local target message.",
            ),
            DeclareLaunchArgument(
                "sensor_forward_offset",
                default_value="0.22",
                description="Forward offset from robot base to the real point-cloud sensor, in meters.",
            ),
            DeclareLaunchArgument(
                "target_timeout",
                default_value="0.0",
                description="Stop publishing non-zero velocity if the selected target is stale. Default 0 disables timeout for stationary-sensor preview.",
            ),
            DeclareLaunchArgument(
                "publish_cmd_vel",
                default_value="false",
                description="false only draws the path; true also drives the Gazebo robot with /cmd_vel.",
            ),
            DeclareLaunchArgument(
                "crouch_on_arrival",
                default_value="true",
                description="Publish a gait action after reaching the stop pose.",
            ),
            DeclareLaunchArgument(
                "arrival_action",
                default_value="crouch",
                description="Gait action to publish after arrival. The controller accepts crouch/down/lie_down.",
            ),
            DeclareLaunchArgument(
                "standoff_distance",
                default_value="0.60",
                description="Distance from the selected box target point to the preview stop pose, in meters.",
            ),
            DeclareLaunchArgument(
                "linear_tolerance",
                default_value="0.08",
                description="Distance tolerance for considering the sim robot at the stop pose, in meters.",
            ),
            DeclareLaunchArgument(
                "yaw_tolerance",
                default_value="0.08",
                description="Yaw tolerance for considering the sim robot aligned at the stop pose, in radians.",
            ),
            DeclareLaunchArgument(
                "drive_mode",
                default_value="holonomic",
                description="turn_then_forward rotates before walking; holonomic also uses linear.y for lateral targets.",
            ),
            DeclareLaunchArgument(
                "enable_yaw_control",
                default_value="true",
                description="Publish angular.z from the navigation node.",
            ),
            sim_stack,
            nav_preview,
        ]
    )
