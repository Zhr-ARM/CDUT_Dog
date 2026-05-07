from glob import glob
from setuptools import setup


package_name = 'dog_vision'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/models/task_2', glob('models/task_2/*')),
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
        ],
    },
)
