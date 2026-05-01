import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('dog_vision')
    default_model = os.path.join(package_share, 'models', 'task_2', 'best.pt')
    default_data = os.path.join(package_share, 'config', 'task_2_data.yaml')

    arguments = [
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Path to the YOLO .pt or ONNX model file.',
        ),
        DeclareLaunchArgument(
            'data',
            default_value=default_data,
            description='Path to the YOLO data.yaml file.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='auto',
            description='Inference mode: auto, yolo, or onnx.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video2',
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
    ]

    node = Node(
        package='dog_vision',
        executable='yolo_detect_node',
        name='yolo_detect_node',
        output='screen',
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
        }],
    )

    return LaunchDescription(arguments + [node])
