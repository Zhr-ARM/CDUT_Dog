import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def workspace_root_from_share(package_share: str) -> str:
    # <workspace>/install/dog_vision/share/dog_vision -> <workspace>
    path = os.path.abspath(package_share)
    parts = path.split(os.sep)
    if 'install' in parts:
        install_index = parts.index('install')
        return os.sep.join(parts[:install_index]) or os.sep
    return os.getcwd()


def generate_launch_description():
    package_share = get_package_share_directory('dog_vision')
    workspace_root = workspace_root_from_share(package_share)
    default_model = os.path.join(package_share, 'models', 'task_2', 'best.onnx')
    default_data = os.path.join(package_share, 'config', 'task_2_data.yaml')
    default_ocr_model = os.path.join(package_share, 'models', 'ocr', 'PP-OCRv5_server_rec.onnx')
    default_ocr_dict = os.path.join(package_share, 'models', 'ocr', 'dict.txt')
    default_onnx_python = os.path.join(workspace_root, '.venv-ocr-onnx', 'bin', 'python')
    default_ocr_env = os.path.join(workspace_root, '.env')

    arguments = [
        DeclareLaunchArgument(
            'launch_camera_mode_mux',
            default_value='true',
            description='Whether to launch the yolo/ocr camera mode mux.',
        ),
        DeclareLaunchArgument(
            'launch_yolo',
            default_value='true',
            description='Whether to launch the YOLO detector node.',
        ),
        DeclareLaunchArgument(
            'launch_ocr',
            default_value='false',
            description='Whether to launch the OCR recognition node.',
        ),
        DeclareLaunchArgument(
            'vision_mode_topic',
            default_value='/vision/mode',
            description='String topic for camera mode: off, yolo, ocr.',
        ),
        DeclareLaunchArgument(
            'legacy_vision_enabled_topic',
            default_value='/arm_vision_mode/enabled',
            description='Existing Bool topic mapped to yolo/off by the mux.',
        ),
        DeclareLaunchArgument(
            'yolo_enabled_topic',
            default_value='/vision/yolo_enabled',
            description='Bool topic emitted by the mux to enable YOLO.',
        ),
        DeclareLaunchArgument(
            'ocr_enabled_topic',
            default_value='/vision/ocr_enabled',
            description='Bool topic emitted by the mux to enable OCR.',
        ),
        DeclareLaunchArgument(
            'initial_vision_mode',
            default_value='off',
            description='Initial mux mode: off, yolo, or ocr.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Path to the YOLO ONNX model file. PT/ultralytics inference is disabled.',
        ),
        DeclareLaunchArgument(
            'data',
            default_value=default_data,
            description='Path to the YOLO data.yaml file.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='auto',
            description='Inference mode: auto or onnx. PT/ultralytics inference is disabled.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video0',
            description='Camera device path.',
        ),
        DeclareLaunchArgument(
            'width',
            default_value='1920',
            description='Camera width.',
        ),
        DeclareLaunchArgument(
            'height',
            default_value='1080',
            description='Camera height.',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='Camera frame rate.',
        ),
        DeclareLaunchArgument(
            'pixel_format',
            default_value='MJPG',
            description='Camera pixel format.',
        ),
        DeclareLaunchArgument(
            'imgsz',
            default_value='640',
            description='YOLO inference image size.',
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.6',
            description='Detection confidence threshold.',
        ),
        DeclareLaunchArgument(
            'iou',
            default_value='0.45',
            description='NMS IoU threshold.',
        ),
        DeclareLaunchArgument(
            'show_image',
            default_value='false',
            description='Whether to show the annotated camera image.',
        ),
        DeclareLaunchArgument(
            'enabled_topic',
            default_value='/vision/yolo_enabled',
            description='Bool topic that enables camera capture and inference.',
        ),
        DeclareLaunchArgument(
            'start_enabled',
            default_value='false',
            description='Whether to open the camera immediately at node startup.',
        ),
        DeclareLaunchArgument(
            'ocr_backend',
            default_value='api',
            description='OCR backend: api or onnx.',
        ),
        DeclareLaunchArgument(
            'ocr_allow_api_fallback',
            default_value='false',
            description='Whether api backend may fall back to onnx after a worker failure.',
        ),
        DeclareLaunchArgument(
            'ocr_width',
            default_value='1280',
            description='OCR camera width.',
        ),
        DeclareLaunchArgument(
            'ocr_height',
            default_value='720',
            description='OCR camera height.',
        ),
        DeclareLaunchArgument(
            'ocr_filter_consensus',
            default_value='3',
            description='Consecutive-frame consensus for local OCR backends.',
        ),
        DeclareLaunchArgument(
            'onnx_python',
            default_value=default_onnx_python,
            description='Python executable for the ONNX OCR virtual environment.',
        ),
        DeclareLaunchArgument(
            'api_python',
            default_value=default_onnx_python,
            description='Python executable for the API OCR backend.',
        ),
        DeclareLaunchArgument(
            'ocr_onnx_model_path',
            default_value=default_ocr_model,
            description='Path to OCR ONNX recognition model.',
        ),
        DeclareLaunchArgument(
            'ocr_onnx_dict_path',
            default_value=default_ocr_dict,
            description='Path to OCR ONNX character dictionary.',
        ),
        DeclareLaunchArgument(
            'ocr_env_file',
            default_value=default_ocr_env,
            description='Optional env file for API OCR credentials.',
        ),
    ]

    mux_node = Node(
        package='dog_vision',
        executable='camera_mode_mux_node',
        name='camera_mode_mux_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_camera_mode_mux')),
        parameters=[{
            'mode_topic': LaunchConfiguration('vision_mode_topic'),
            'legacy_enabled_topic': LaunchConfiguration('legacy_vision_enabled_topic'),
            'yolo_enabled_topic': LaunchConfiguration('yolo_enabled_topic'),
            'ocr_enabled_topic': LaunchConfiguration('ocr_enabled_topic'),
            'initial_mode': ParameterValue(LaunchConfiguration('initial_vision_mode'), value_type=str),
        }],
    )

    yolo_node = Node(
        package='dog_vision',
        executable='yolo_detect_node',
        name='yolo_detect_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_yolo')),
        parameters=[{
            'model': LaunchConfiguration('model'),
            'data': LaunchConfiguration('data'),
            'mode': LaunchConfiguration('mode'),
            'device': LaunchConfiguration('device'),
            'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'imgsz': ParameterValue(LaunchConfiguration('imgsz'), value_type=int),
            'conf': ParameterValue(LaunchConfiguration('conf'), value_type=float),
            'iou': ParameterValue(LaunchConfiguration('iou'), value_type=float),
            'show_image': ParameterValue(LaunchConfiguration('show_image'), value_type=bool),
            'enabled_topic': LaunchConfiguration('enabled_topic'),
            'start_enabled': ParameterValue(LaunchConfiguration('start_enabled'), value_type=bool),
        }],
    )

    ocr_node = Node(
        package='dog_vision',
        executable='ocr_recognition_node',
        name='ocr_recognition_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_ocr')),
        parameters=[{
            'backend': LaunchConfiguration('ocr_backend'),
            'allow_api_fallback': ParameterValue(LaunchConfiguration('ocr_allow_api_fallback'), value_type=bool),
            'enabled_topic': LaunchConfiguration('ocr_enabled_topic'),
            'start_enabled': False,
            'device': LaunchConfiguration('device'),
            'width': ParameterValue(LaunchConfiguration('ocr_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('ocr_height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'show_image': ParameterValue(LaunchConfiguration('show_image'), value_type=bool),
            'filter_consensus': ParameterValue(LaunchConfiguration('ocr_filter_consensus'), value_type=int),
            'onnx_python': LaunchConfiguration('onnx_python'),
            'api_python': LaunchConfiguration('api_python'),
            'onnx_model_path': LaunchConfiguration('ocr_onnx_model_path'),
            'onnx_dict_path': LaunchConfiguration('ocr_onnx_dict_path'),
            'env_file': LaunchConfiguration('ocr_env_file'),
        }],
    )

    return LaunchDescription(arguments + [mux_node, yolo_node, ocr_node])
