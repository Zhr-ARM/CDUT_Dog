from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
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
            Node(
                package="dog_lidar",
                executable="box_grid_locator_node",
                name="box_grid_locator_node",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "target_row": ParameterValue(
                            LaunchConfiguration("target_row"), value_type=int
                        ),
                        "target_col": ParameterValue(
                            LaunchConfiguration("target_col"), value_type=int
                        ),
                        "target_pose_topic": "/navigation/target_pose",
                        "debug_cloud_topic": "/navigation/box_cloud_debug",
                        "marker_topic": "/boxes/markers",
                        "status_topic": "/boxes/status",
                        "x_min": ParameterValue(
                            LaunchConfiguration("x_min"), value_type=float
                        ),
                        "x_max": ParameterValue(
                            LaunchConfiguration("x_max"), value_type=float
                        ),
                        "y_min": ParameterValue(
                            LaunchConfiguration("y_min"), value_type=float
                        ),
                        "y_max": ParameterValue(
                            LaunchConfiguration("y_max"), value_type=float
                        ),
                        "z_min": ParameterValue(
                            LaunchConfiguration("z_min"), value_type=float
                        ),
                        "z_max": ParameterValue(
                            LaunchConfiguration("z_max"), value_type=float
                        ),
                        "self_filter_x_max": ParameterValue(
                            LaunchConfiguration("self_filter_x_max"), value_type=float
                        ),
                        "self_filter_y_abs_max": ParameterValue(
                            LaunchConfiguration("self_filter_y_abs_max"), value_type=float
                        ),
                        "voxel_leaf_size": ParameterValue(
                            LaunchConfiguration("voxel_leaf_size"), value_type=float
                        ),
                        "candidate_extraction_mode": LaunchConfiguration(
                            "candidate_extraction_mode"
                        ),
                        "yz_cell_y_size": ParameterValue(
                            LaunchConfiguration("yz_cell_y_size"), value_type=float
                        ),
                        "yz_cell_z_size": ParameterValue(
                            LaunchConfiguration("yz_cell_z_size"), value_type=float
                        ),
                        "yz_connect_y_gap_cells": ParameterValue(
                            LaunchConfiguration("yz_connect_y_gap_cells"), value_type=int
                        ),
                        "yz_connect_z_gap_cells": ParameterValue(
                            LaunchConfiguration("yz_connect_z_gap_cells"), value_type=int
                        ),
                        "yz_connect_x_tolerance": ParameterValue(
                            LaunchConfiguration("yz_connect_x_tolerance"),
                            value_type=float,
                        ),
                        "front_x_quantile": ParameterValue(
                            LaunchConfiguration("front_x_quantile"), value_type=float
                        ),
                        "front_face_depth_max": ParameterValue(
                            LaunchConfiguration("front_face_depth_max"), value_type=float
                        ),
                        "profile_z_min": ParameterValue(
                            LaunchConfiguration("profile_z_min"), value_type=float
                        ),
                        "profile_z_max": ParameterValue(
                            LaunchConfiguration("profile_z_max"), value_type=float
                        ),
                        "profile_y_bin_size": ParameterValue(
                            LaunchConfiguration("profile_y_bin_size"), value_type=float
                        ),
                        "profile_min_bin_points": ParameterValue(
                            LaunchConfiguration("profile_min_bin_points"), value_type=int
                        ),
                        "profile_connect_gap_bins": ParameterValue(
                            LaunchConfiguration("profile_connect_gap_bins"), value_type=int
                        ),
                        "profile_connect_x_tolerance": ParameterValue(
                            LaunchConfiguration("profile_connect_x_tolerance"),
                            value_type=float,
                        ),
                        "profile_min_segment_bins": ParameterValue(
                            LaunchConfiguration("profile_min_segment_bins"), value_type=int
                        ),
                        "profile_box_width": ParameterValue(
                            LaunchConfiguration("profile_box_width"), value_type=float
                        ),
                        "profile_candidate_x_tolerance": ParameterValue(
                            LaunchConfiguration("profile_candidate_x_tolerance"),
                            value_type=float,
                        ),
                        "profile_nms_y_tolerance": ParameterValue(
                            LaunchConfiguration("profile_nms_y_tolerance"), value_type=float
                        ),
                        "template_search_x_min": ParameterValue(
                            LaunchConfiguration("template_search_x_min"), value_type=float
                        ),
                        "template_search_x_max": ParameterValue(
                            LaunchConfiguration("template_search_x_max"), value_type=float
                        ),
                        "template_y_half_width": ParameterValue(
                            LaunchConfiguration("template_y_half_width"), value_type=float
                        ),
                        "template_front_margin": ParameterValue(
                            LaunchConfiguration("template_front_margin"), value_type=float
                        ),
                        "template_depth": ParameterValue(
                            LaunchConfiguration("template_depth"), value_type=float
                        ),
                        "template_x_tolerance": ParameterValue(
                            LaunchConfiguration("template_x_tolerance"), value_type=float
                        ),
                        "template_min_box_points": ParameterValue(
                            LaunchConfiguration("template_min_box_points"), value_type=int
                        ),
                        "template_min_total_points": ParameterValue(
                            LaunchConfiguration("template_min_total_points"), value_type=int
                        ),
                        "template_min_z_span": ParameterValue(
                            LaunchConfiguration("template_min_z_span"), value_type=float
                        ),
                        "template_upper_z_min": ParameterValue(
                            LaunchConfiguration("template_upper_z_min"), value_type=float
                        ),
                        "template_min_upper_points": ParameterValue(
                            LaunchConfiguration("template_min_upper_points"), value_type=int
                        ),
                        "min_face_points": ParameterValue(
                            LaunchConfiguration("min_face_points"), value_type=int
                        ),
                        "min_face_width_y": ParameterValue(
                            LaunchConfiguration("min_face_width_y"), value_type=float
                        ),
                        "max_face_width_y": ParameterValue(
                            LaunchConfiguration("max_face_width_y"), value_type=float
                        ),
                        "min_face_height_z": ParameterValue(
                            LaunchConfiguration("min_face_height_z"), value_type=float
                        ),
                        "max_face_height_z": ParameterValue(
                            LaunchConfiguration("max_face_height_z"), value_type=float
                        ),
                        "max_face_depth_x": ParameterValue(
                            LaunchConfiguration("max_face_depth_x"), value_type=float
                        ),
                        "near_row_depth_window": ParameterValue(
                            LaunchConfiguration("near_row_depth_window"),
                            value_type=float,
                        ),
                        "center_from_front_offset": ParameterValue(
                            LaunchConfiguration("center_from_front_offset"),
                            value_type=float,
                        ),
                        "publish_box_center": ParameterValue(
                            LaunchConfiguration("publish_box_center"), value_type=bool
                        ),
                        "expected_cols": ParameterValue(
                            LaunchConfiguration("expected_cols"), value_type=int
                        ),
                        "sort_left_to_right_positive_y": ParameterValue(
                            LaunchConfiguration("sort_left_to_right_positive_y"),
                            value_type=bool,
                        ),
                        "use_row_layout_filter": ParameterValue(
                            LaunchConfiguration("use_row_layout_filter"), value_type=bool
                        ),
                        "row_col_spacing": ParameterValue(
                            LaunchConfiguration("row_col_spacing"), value_type=float
                        ),
                        "row_spacing_tolerance": ParameterValue(
                            LaunchConfiguration("row_spacing_tolerance"), value_type=float
                        ),
                        "row_x_tolerance": ParameterValue(
                            LaunchConfiguration("row_x_tolerance"), value_type=float
                        ),
                        "min_layout_matches": ParameterValue(
                            LaunchConfiguration("min_layout_matches"), value_type=int
                        ),
                        "infer_missing_boxes": ParameterValue(
                            LaunchConfiguration("infer_missing_boxes"), value_type=bool
                        ),
                        "processing_period": ParameterValue(
                            LaunchConfiguration("processing_period"), value_type=float
                        ),
                        "marker_lifetime": ParameterValue(
                            LaunchConfiguration("marker_lifetime"), value_type=float
                        ),
                        "status_include_faces": ParameterValue(
                            LaunchConfiguration("status_include_faces"), value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="dog_vision",
                executable="target_selector_node",
                name="target_selector_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_target_selector")),
                parameters=[
                    {
                        "target_row": ParameterValue(
                            LaunchConfiguration("target_row"), value_type=int
                        ),
                        "target_col": ParameterValue(
                            LaunchConfiguration("target_col"), value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="dog_vision",
                executable="yolo_detect_node",
                name="yolo_detect_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_yolo")),
            ),
        ]
    )
