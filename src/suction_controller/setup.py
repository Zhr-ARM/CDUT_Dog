from glob import glob
from setuptools import find_packages, setup

package_name = "suction_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src", exclude=["test"]),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="zhr",
    maintainer_email="zhr@example.com",
    description="Serial relay controller for suction cup and air valve.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "suction_controller_node = suction_controller.suction_controller_node:main",
        ],
    },
)
