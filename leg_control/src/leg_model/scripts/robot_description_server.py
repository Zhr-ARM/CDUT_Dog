#!/usr/bin/env python3

import os
import re

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory


def load_and_sanitize_urdf(urdf_file_path: str, controllers_file: str) -> str:
    with open(urdf_file_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # Ensure gazebo_ros2_control receives a real filesystem path for params.
    # The plugin does NOT understand package:// URIs for --params-file.
    robot_description = re.sub(
        r"<parameters>\s*[^<]*ros2_controllers\.yaml\s*</parameters>",
        f"<parameters>{controllers_file}</parameters>",
        robot_description,
        count=1,
    )

    # Reduce characters that tend to break rcl param override parsing in gazebo_ros2_control
    robot_description = robot_description.replace('\r', ' ').replace('\n', ' ').replace('\t', ' ')
    while '  ' in robot_description:
        robot_description = robot_description.replace('  ', ' ')

    # Convert attribute quoting to single quotes (valid XML)
    robot_description = robot_description.replace('"', "'")

    # Drop XML declaration and comments
    robot_description = re.sub(r"<\?xml[^>]*\?>", "", robot_description)
    robot_description = re.sub(r"<!--.*?-->", "", robot_description)

    return robot_description.strip()


class RobotDescriptionServer(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_server')

        pkg_dir = get_package_share_directory('leg_model')
        urdf_file_path = os.path.join(pkg_dir, 'urdf', 'leg_model.urdf')
        controllers_file = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')

        robot_description = load_and_sanitize_urdf(urdf_file_path, controllers_file)
        self.declare_parameter('robot_description', robot_description)
        self.get_logger().info(
            f"robot_description_server ready (len={len(robot_description)}), controllers_file={controllers_file}")


def main() -> None:
    rclpy.init()
    node = RobotDescriptionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
