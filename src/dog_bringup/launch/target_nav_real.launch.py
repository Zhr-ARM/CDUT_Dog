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
    default_gait_param_file = os.path.join(
        get_package_share_directory("dog_position_control"),
        "config",
        "gait_controller_real_stand.yaml",
    )

    perception_arg_names = [
        "target_row",
        "target_col",
        "launch_yolo",
        "launch_target_selector",
        "cloud_topic",
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "z_min",
        "z_max",
        "self_filter_x_max",
        "self_filter_y_abs_max",
        "voxel_leaf_size",
        "candidate_extraction_mode",
        "yz_cell_y_size",
        "yz_cell_z_size",
        "yz_connect_y_gap_cells",
        "yz_connect_z_gap_cells",
        "yz_connect_x_tolerance",
        "front_x_quantile",
        "front_face_depth_max",
        "profile_z_min",
        "profile_z_max",
        "profile_y_bin_size",
        "profile_min_bin_points",
        "profile_connect_gap_bins",
        "profile_connect_x_tolerance",
        "profile_min_segment_bins",
        "profile_box_width",
        "profile_candidate_x_tolerance",
        "profile_nms_y_tolerance",
        "template_search_x_min",
        "template_search_x_max",
        "template_y_half_width",
        "template_front_margin",
        "template_depth",
        "template_x_tolerance",
        "template_min_box_points",
        "template_min_total_points",
        "template_min_z_span",
        "template_upper_z_min",
        "template_min_upper_points",
        "min_face_points",
        "min_face_width_y",
        "max_face_width_y",
        "min_face_height_z",
        "max_face_height_z",
        "max_face_depth_x",
        "near_row_depth_window",
        "center_from_front_offset",
        "publish_box_center",
        "expected_cols",
        "sort_left_to_right_positive_y",
        "use_row_layout_filter",
        "row_col_spacing",
        "row_spacing_tolerance",
        "row_x_tolerance",
        "min_layout_matches",
        "infer_missing_boxes",
        "processing_period",
        "marker_lifetime",
        "status_include_faces",
    ]

    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "target_perception.launch.py")
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name in perception_arg_names
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_perception")),
    )

    real_control_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "real.launch.py")
        ),
        launch_arguments={
            "gait_param_file": LaunchConfiguration("gait_param_file"),
            "launch_simulation": "false",
            "launch_rviz": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_real_control")),
    )

    nav_node = Node(
        package="dog_vision",
        executable="box_nav_preview_node",
        name="box_target_nav_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "target_pose_topic": "/navigation/target_pose",
                "odom_topic": "/ground_truth",
                "path_topic": "/navigation/preview_path",
                "marker_topic": "/navigation/preview_markers",
                "cmd_vel_topic": "/cmd_vel",
                "gait_action_topic": "/gait_action",
                "status_topic": "/navigation/nav_status",
                "publish_cmd_vel": ParameterValue(
                    LaunchConfiguration("publish_cmd_vel"), value_type=bool
                ),
                "crouch_on_arrival": ParameterValue(
                    LaunchConfiguration("crouch_on_arrival"), value_type=bool
                ),
                "arrival_action": LaunchConfiguration("arrival_action"),
                "control_mode": LaunchConfiguration("control_mode"),
                "drive_mode": LaunchConfiguration("drive_mode"),
                "enable_yaw_control": ParameterValue(
                    LaunchConfiguration("enable_yaw_control"), value_type=bool
                ),
                "fixed_frame": "odom",
                "target_frame_mode": LaunchConfiguration("target_frame_mode"),
                "lock_target": ParameterValue(
                    LaunchConfiguration("lock_target"), value_type=bool
                ),
                "sensor_forward_offset": ParameterValue(
                    LaunchConfiguration("sensor_forward_offset"), value_type=float
                ),
                "target_timeout": ParameterValue(
                    LaunchConfiguration("target_timeout"), value_type=float
                ),
                "standoff_distance": ParameterValue(
                    LaunchConfiguration("standoff_distance"), value_type=float
                ),
                "linear_tolerance": ParameterValue(
                    LaunchConfiguration("linear_tolerance"), value_type=float
                ),
                "yaw_tolerance": ParameterValue(
                    LaunchConfiguration("yaw_tolerance"), value_type=float
                ),
                "linear_gain": ParameterValue(
                    LaunchConfiguration("linear_gain"), value_type=float
                ),
                "lateral_gain": ParameterValue(
                    LaunchConfiguration("lateral_gain"), value_type=float
                ),
                "angular_gain": ParameterValue(
                    LaunchConfiguration("angular_gain"), value_type=float
                ),
                "min_linear_speed": ParameterValue(
                    LaunchConfiguration("min_linear_speed"), value_type=float
                ),
                "max_linear_speed": ParameterValue(
                    LaunchConfiguration("max_linear_speed"), value_type=float
                ),
                "min_lateral_speed": ParameterValue(
                    LaunchConfiguration("min_lateral_speed"), value_type=float
                ),
                "max_lateral_speed": ParameterValue(
                    LaunchConfiguration("max_lateral_speed"), value_type=float
                ),
                "min_angular_speed": ParameterValue(
                    LaunchConfiguration("min_angular_speed"), value_type=float
                ),
                "max_angular_speed": ParameterValue(
                    LaunchConfiguration("max_angular_speed"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_real_control",
                default_value="false",
                description="Start real motor and gait control. Keep false until hardware is ready.",
            ),
            DeclareLaunchArgument(
                "launch_perception",
                default_value="true",
                description="Start point-cloud front-face box perception.",
            ),
            DeclareLaunchArgument(
                "gait_param_file",
                default_value=default_gait_param_file,
                description="Real gait controller parameter file.",
            ),
            DeclareLaunchArgument("target_row", default_value="0"),
            DeclareLaunchArgument("target_col", default_value="1"),
            DeclareLaunchArgument("launch_yolo", default_value="false"),
            DeclareLaunchArgument("launch_target_selector", default_value="false"),
            DeclareLaunchArgument("cloud_topic", default_value="/odin1/cloud_render"),
            DeclareLaunchArgument("x_min", default_value="0.30"),
            DeclareLaunchArgument("x_max", default_value="4.50"),
            DeclareLaunchArgument("y_min", default_value="-3.00"),
            DeclareLaunchArgument("y_max", default_value="3.00"),
            DeclareLaunchArgument("z_min", default_value="-0.32"),
            DeclareLaunchArgument("z_max", default_value="0.05"),
            DeclareLaunchArgument("self_filter_x_max", default_value="0.60"),
            DeclareLaunchArgument("self_filter_y_abs_max", default_value="0.60"),
            DeclareLaunchArgument("voxel_leaf_size", default_value="0.015"),
            DeclareLaunchArgument("candidate_extraction_mode", default_value="front_profile"),
            DeclareLaunchArgument("yz_cell_y_size", default_value="0.035"),
            DeclareLaunchArgument("yz_cell_z_size", default_value="0.035"),
            DeclareLaunchArgument("yz_connect_y_gap_cells", default_value="2"),
            DeclareLaunchArgument("yz_connect_z_gap_cells", default_value="2"),
            DeclareLaunchArgument("yz_connect_x_tolerance", default_value="0.30"),
            DeclareLaunchArgument("front_x_quantile", default_value="0.20"),
            DeclareLaunchArgument("front_face_depth_max", default_value="0.30"),
            DeclareLaunchArgument("profile_z_min", default_value="-0.24"),
            DeclareLaunchArgument("profile_z_max", default_value="0.03"),
            DeclareLaunchArgument("profile_y_bin_size", default_value="0.05"),
            DeclareLaunchArgument("profile_min_bin_points", default_value="2"),
            DeclareLaunchArgument("profile_connect_gap_bins", default_value="2"),
            DeclareLaunchArgument("profile_connect_x_tolerance", default_value="0.35"),
            DeclareLaunchArgument("profile_min_segment_bins", default_value="2"),
            DeclareLaunchArgument("profile_box_width", default_value="0.36"),
            DeclareLaunchArgument("profile_candidate_x_tolerance", default_value="0.35"),
            DeclareLaunchArgument("profile_nms_y_tolerance", default_value="0.30"),
            DeclareLaunchArgument("template_search_x_min", default_value="0.80"),
            DeclareLaunchArgument("template_search_x_max", default_value="4.50"),
            DeclareLaunchArgument("template_y_half_width", default_value="0.22"),
            DeclareLaunchArgument("template_front_margin", default_value="0.08"),
            DeclareLaunchArgument("template_depth", default_value="0.42"),
            DeclareLaunchArgument("template_x_tolerance", default_value="0.40"),
            DeclareLaunchArgument("template_min_box_points", default_value="5"),
            DeclareLaunchArgument("template_min_total_points", default_value="18"),
            DeclareLaunchArgument("template_min_z_span", default_value="0.12"),
            DeclareLaunchArgument("template_upper_z_min", default_value="-0.18"),
            DeclareLaunchArgument("template_min_upper_points", default_value="2"),
            DeclareLaunchArgument("min_face_points", default_value="3"),
            DeclareLaunchArgument("min_face_width_y", default_value="0.03"),
            DeclareLaunchArgument("max_face_width_y", default_value="0.45"),
            DeclareLaunchArgument("min_face_height_z", default_value="0.03"),
            DeclareLaunchArgument("max_face_height_z", default_value="0.35"),
            DeclareLaunchArgument("max_face_depth_x", default_value="0.45"),
            DeclareLaunchArgument("near_row_depth_window", default_value="0.45"),
            DeclareLaunchArgument("center_from_front_offset", default_value="0.125"),
            DeclareLaunchArgument("publish_box_center", default_value="true"),
            DeclareLaunchArgument("expected_cols", default_value="4"),
            DeclareLaunchArgument("sort_left_to_right_positive_y", default_value="true"),
            DeclareLaunchArgument("use_row_layout_filter", default_value="true"),
            DeclareLaunchArgument("row_col_spacing", default_value="0.85"),
            DeclareLaunchArgument("row_spacing_tolerance", default_value="0.25"),
            DeclareLaunchArgument("row_x_tolerance", default_value="0.45"),
            DeclareLaunchArgument("min_layout_matches", default_value="3"),
            DeclareLaunchArgument("infer_missing_boxes", default_value="true"),
            DeclareLaunchArgument("processing_period", default_value="0.20"),
            DeclareLaunchArgument("marker_lifetime", default_value="1.00"),
            DeclareLaunchArgument("status_include_faces", default_value="false"),
            DeclareLaunchArgument(
                "publish_cmd_vel",
                default_value="false",
                description="Publish /cmd_vel. Keep false for dry-runs.",
            ),
            DeclareLaunchArgument("crouch_on_arrival", default_value="true"),
            DeclareLaunchArgument("arrival_action", default_value="crouch"),
            DeclareLaunchArgument(
                "target_frame_mode",
                default_value="local",
                description="local for sensor-relative target poses; fixed for odom/map poses.",
            ),
            DeclareLaunchArgument(
                "control_mode",
                default_value="relative",
                description="relative uses target poses directly without odom.",
            ),
            DeclareLaunchArgument("drive_mode", default_value="holonomic"),
            DeclareLaunchArgument("enable_yaw_control", default_value="false"),
            DeclareLaunchArgument("lock_target", default_value="false"),
            DeclareLaunchArgument(
                "sensor_forward_offset",
                default_value="0.22",
                description="Forward offset from base to the point-cloud sensor, in meters.",
            ),
            DeclareLaunchArgument("target_timeout", default_value="0.8"),
            DeclareLaunchArgument("standoff_distance", default_value="0.60"),
            DeclareLaunchArgument("linear_tolerance", default_value="0.08"),
            DeclareLaunchArgument("yaw_tolerance", default_value="0.08"),
            DeclareLaunchArgument("linear_gain", default_value="0.28"),
            DeclareLaunchArgument("lateral_gain", default_value="0.28"),
            DeclareLaunchArgument("angular_gain", default_value="0.55"),
            DeclareLaunchArgument("min_linear_speed", default_value="0.03"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.08"),
            DeclareLaunchArgument("min_lateral_speed", default_value="0.03"),
            DeclareLaunchArgument("max_lateral_speed", default_value="0.07"),
            DeclareLaunchArgument("min_angular_speed", default_value="0.08"),
            DeclareLaunchArgument("max_angular_speed", default_value="0.18"),
            perception_stack,
            real_control_stack,
            nav_node,
        ]
    )
