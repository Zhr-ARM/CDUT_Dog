import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def workspace_root_from_share(package_share: str) -> str:
    path = os.path.abspath(package_share)
    parts = path.split(os.sep)
    if "install" in parts:
        install_index = parts.index("install")
        return os.sep.join(parts[:install_index]) or os.sep
    return os.getcwd()


def generate_launch_description():
    bringup_pkg = get_package_share_directory("dog_bringup")
    workspace_root = workspace_root_from_share(bringup_pkg)

    dog_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, "launch", "dog.launch.py")),
        launch_arguments={
            "launch_teleop": LaunchConfiguration("launch_teleop"),
            "launch_imu": LaunchConfiguration("launch_imu"),
            "imu_type": LaunchConfiguration("imu_type"),
            "imu_port": LaunchConfiguration("imu_port"),
            "imu_baud": LaunchConfiguration("imu_baud"),
            "imu_frame_id": LaunchConfiguration("imu_frame_id"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_dog")),
    )

    arm_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_pkg, "launch", "arm.launch.py")),
        launch_arguments={
            "arm_serial_port": LaunchConfiguration("arm_serial_port"),
            "arm_serial_baudrate": LaunchConfiguration("arm_serial_baudrate"),
            "arm_timeout_ms": LaunchConfiguration("arm_timeout_ms"),
            "arm_retry_count": LaunchConfiguration("arm_retry_count"),
            "arm_use_moveit": LaunchConfiguration("arm_use_moveit"),
            "suction_relay1_port": LaunchConfiguration("suction_relay1_port"),
            "suction_relay2_port": LaunchConfiguration("suction_relay2_port"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_arm")),
    )

    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "target_perception.launch.py")
        ),
        launch_arguments={
            "target_row": LaunchConfiguration("target_row"),
            "target_col": LaunchConfiguration("target_col"),
            "launch_yolo": LaunchConfiguration("launch_yolo"),
            "launch_ocr": LaunchConfiguration("launch_ocr"),
            "launch_camera_mode_mux": LaunchConfiguration("launch_camera_mode_mux"),
            "launch_target_selector": LaunchConfiguration("launch_target_selector"),
            "camera_device": LaunchConfiguration("camera_device"),
            "vision_enabled_topic": LaunchConfiguration("vision_enabled_topic"),
            "vision_mode_topic": LaunchConfiguration("vision_mode_topic"),
            "yolo_enabled_topic": LaunchConfiguration("yolo_enabled_topic"),
            "ocr_enabled_topic": LaunchConfiguration("ocr_enabled_topic"),
            "initial_vision_mode": LaunchConfiguration("initial_vision_mode"),
            "vision_start_enabled": LaunchConfiguration("vision_start_enabled"),
            "ocr_backend": LaunchConfiguration("ocr_backend"),
            "ocr_allow_api_fallback": LaunchConfiguration("ocr_allow_api_fallback"),
            "onnx_python": LaunchConfiguration("onnx_python"),
            "api_python": LaunchConfiguration("api_python"),
            "ocr_env_file": LaunchConfiguration("ocr_env_file"),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_perception")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_dog", default_value="true"),
            DeclareLaunchArgument("launch_arm", default_value="true"),
            DeclareLaunchArgument("launch_perception", default_value="true"),
            DeclareLaunchArgument("launch_yolo", default_value="true"),
            DeclareLaunchArgument("launch_ocr", default_value="false"),
            DeclareLaunchArgument("launch_camera_mode_mux", default_value="true"),
            DeclareLaunchArgument("launch_target_selector", default_value="false"),
            DeclareLaunchArgument("target_row", default_value="0"),
            DeclareLaunchArgument("target_col", default_value="1"),
            DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
            DeclareLaunchArgument("vision_enabled_topic", default_value="/arm_vision_mode/enabled"),
            DeclareLaunchArgument("vision_mode_topic", default_value="/vision/mode"),
            DeclareLaunchArgument("yolo_enabled_topic", default_value="/vision/yolo_enabled"),
            DeclareLaunchArgument("ocr_enabled_topic", default_value="/vision/ocr_enabled"),
            DeclareLaunchArgument("initial_vision_mode", default_value="off"),
            DeclareLaunchArgument("vision_start_enabled", default_value="false"),
            DeclareLaunchArgument("ocr_backend", default_value="api"),
            DeclareLaunchArgument("ocr_allow_api_fallback", default_value="false"),
            DeclareLaunchArgument(
                "onnx_python",
                default_value=os.path.join(workspace_root, ".venv-ocr-onnx", "bin", "python"),
            ),
            DeclareLaunchArgument(
                "api_python",
                default_value=os.path.join(workspace_root, ".venv-ocr-onnx", "bin", "python"),
            ),
            DeclareLaunchArgument("ocr_env_file", default_value=os.path.join(workspace_root, ".env")),
            DeclareLaunchArgument("cloud_topic", default_value="/odin1/cloud_render"),
            DeclareLaunchArgument("launch_teleop", default_value="true"),
            DeclareLaunchArgument("launch_imu", default_value="true"),
            DeclareLaunchArgument("imu_type", default_value="normal"),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyimu"),
            DeclareLaunchArgument("imu_baud", default_value="9600"),
            DeclareLaunchArgument("imu_frame_id", default_value="base_link"),
            DeclareLaunchArgument("arm_serial_port", default_value="/dev/arm_motor"),
            DeclareLaunchArgument("arm_serial_baudrate", default_value="921600"),
            DeclareLaunchArgument("arm_timeout_ms", default_value="10"),
            DeclareLaunchArgument("arm_retry_count", default_value="3"),
            DeclareLaunchArgument("arm_use_moveit", default_value="true"),
            DeclareLaunchArgument("suction_relay1_port", default_value="/dev/arm_relay1"),
            DeclareLaunchArgument("suction_relay2_port", default_value="/dev/arm_relay2"),
            dog_stack,
            arm_stack,
            perception_stack,
        ]
    )
