from glob import glob
from setuptools import find_packages, setup


package_name = "dm_j4340_quad"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="zhr",
    maintainer_email="zhr@example.com",
    description="ROS 2 tools for DM-J4340-2EC motors over the Damiao USB-to-CAN serial adapter.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "query_motor_node = dm_j4340_quad.query_motor_node:main",
            "start_motor_node = dm_j4340_quad.start_motor_node:main",
        ],
    },
)
