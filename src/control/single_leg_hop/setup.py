from setuptools import setup
import os
from glob import glob

package_name = 'single_leg_hop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Li Xiang (Algorithm Group)',
    maintainer_email='lixiang@cdut.edu.cn',
    description='FSM hopping controller using Pinocchio',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hop_node = single_leg_hop.hop_node:main',
        ],
    },
)