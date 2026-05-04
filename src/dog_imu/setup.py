from setuptools import setup
from glob import glob

package_name = 'dog_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lx',
    maintainer_email='lx1617299377520@gmail.com',
    description='WitMotion IWT603 IMU ROS 2 driver for CDUT_Dog.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wit_normal_node   = dog_imu.wit_normal_node:main',
            'wit_modbus_node   = dog_imu.wit_modbus_node:main',
            'wit_hmodbus_node  = dog_imu.wit_hmodbus_node:main',
            'wit_can_node      = dog_imu.wit_can_node:main',
            'wit_hcan_node     = dog_imu.wit_hcan_node:main',
            'wit_imu_ctrl      = dog_imu.wit_imu_ctrl:main',
            'get_imu_rpy       = dog_imu.get_imu_rpy:main',
            'wit_normal_demo   = dog_imu.wit_normal_demo:main',
            'wit_modbus_demo   = dog_imu.wit_modbus_demo:main',
            'wit_normal_ui     = dog_imu.wit_normal_ui:main',
            'wit_modbus_ui     = dog_imu.wit_modbus_ui:main',
            'convert_log       = dog_imu.convert:main',
        ],
    },
)
