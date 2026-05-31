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

    target_row = LaunchConfiguration("target_row")
    target_col = LaunchConfiguration("target_col")
    publish_cmd_vel = LaunchConfiguration("publish_cmd_vel")
    standoff_distance = LaunchConfiguration("standoff_distance")
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

    box_locator = Node(
        package="dog_lidar",
        executable="box_grid_locator_node",
        name="box_grid_locator_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "cloud_topic": "/points",
                "debug_cloud_topic": "/navigation/box_cloud_debug",
                "marker_topic": "/boxes/markers",
                "status_topic": "/boxes/status",
                "target_pose_topic": "/navigation/target_pose",
                "target_row": ParameterValue(target_row, value_type=int),
                "target_col": ParameterValue(target_col, value_type=int),
                "z_min": -0.35,
                "z_max": 0.30,
                "self_filter_x_max": 0.60,
                "self_filter_y_abs_max": 0.60,
                "x_min": 3.0,
                "x_max": 4.0,
                "y_min": -1.0,
                "y_max": 2.0,
                "cluster_tolerance": 0.28,
                "min_cluster_size": 2,
                "voxel_leaf_size": 0.05,
                "use_face_filter": True,
                "face_width_min": 0.04,
                "face_width_max": 0.70,
                "face_height_min": 0.02,
                "face_height_max": 0.65,
                "face_depth_max": 0.45,
                "front_x_quantile": 0.10,
                "use_ground_filter": True,
                "ground_cell_size": 0.20,
                "min_height_above_ground": 0.03,
                "max_height_above_ground": 0.55,
                "use_grid_filter": True,
                "grid_row_spacing": 0.60,
                "grid_col_spacing": 0.60,
                "grid_match_tolerance": 0.30,
                "grid_min_matches": 3,
                "grid_yaw_min": -0.45,
                "grid_yaw_max": 0.45,
                "grid_yaw_step": 0.0872665,
                "use_temporal_filter": True,
                "temporal_confirm_frames": 2,
                "temporal_hold_frames": 6,
                "temporal_smoothing_alpha": 0.35,
                "temporal_reset_distance": 0.45,
                "publish_inferred_boxes": True,
                "inferred_box_z": 0.05,
                "marker_lifetime": 2.0,
                "processing_period": 0.5,
            }
        ],
    )

    nav_preview = Node(
        package="dog_vision",
        executable="box_nav_preview_node",
        name="box_nav_preview_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_pose_topic": "/navigation/target_pose",
                "odom_topic": "/ground_truth",
                "path_topic": "/navigation/preview_path",
                "marker_topic": "/navigation/preview_markers",
                "cmd_vel_topic": "/cmd_vel",
                "publish_cmd_vel": ParameterValue(publish_cmd_vel, value_type=bool),
                "fixed_frame": "odom",
                "target_frame_mode": "local",
                "sensor_forward_offset": 0.22,
                "standoff_distance": ParameterValue(standoff_distance, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_sim",
                default_value="true",
                description="Launch Gazebo and RViz before the preview nodes.",
            ),
            DeclareLaunchArgument(
                "target_row",
                default_value="1",
                description="Target row: 0 near row, 1 far row.",
            ),
            DeclareLaunchArgument(
                "target_col",
                default_value="0",
                description="Target column from left to right: 0..3.",
            ),
            DeclareLaunchArgument(
                "publish_cmd_vel",
                default_value="false",
                description="false only draws the path; true also drives the Gazebo robot with /cmd_vel.",
            ),
            DeclareLaunchArgument(
                "standoff_distance",
                default_value="0.60",
                description="Distance from the selected box target point to the preview stop pose, in meters.",
            ),
            sim_stack,
            box_locator,
            nav_preview,
        ]
    )
