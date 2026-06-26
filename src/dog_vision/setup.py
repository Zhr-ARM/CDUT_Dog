from glob import glob
from setuptools import find_packages, setup


package_name = 'dog_vision'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['.env.example']),
        ('share/' + package_name, ['OCR_ROS_INTEGRATION.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/requirements', glob('requirements/*')),
        ('share/' + package_name + '/models/task_2', glob('models/task_2/*')),
        ('share/' + package_name + '/models/ocr', glob('models/ocr/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cys',
    maintainer_email='2388995069@qq.com',
    description='YOLO-based vision node for CDUT_Dog.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detect_node = dog_vision.yolo_detect_node:main',
            'target_selector_node = dog_vision.target_selector_node:main',
            'box_nav_preview_node = dog_vision.box_nav_preview_node:main',
            'field_map_node = dog_vision.field_map_node:main',
            'voice_broadcast_node = dog_vision.voice_broadcast_node:main',
            'camera_mode_mux_node = dog_vision.camera_mode_mux_node:main',
            'ocr_recognition_node = dog_vision.ocr_recognition_node:main',
            'ocr_worker_main = dog_vision.ocr_worker_main:main',
        ],
    },
)
