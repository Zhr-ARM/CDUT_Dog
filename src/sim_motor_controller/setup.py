from glob import glob
from setuptools import find_packages, setup


package_name = "sim_motor_controller"


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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zhr",
    maintainer_email="zhr@example.com",
    description="Simulation-side MIT motor controller with the same ROS interface as the real DM-J4340 controller.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_motor_controller_node = sim_motor_controller.sim_motor_controller_node:main",
        ],
    },
)
