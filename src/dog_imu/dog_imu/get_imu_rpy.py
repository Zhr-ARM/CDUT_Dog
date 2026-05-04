#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Subscribe /wit/imu and print Roll/Pitch/Yaw (ROS 2)."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class GetImuRpy(Node):

    def __init__(self):
        super().__init__('get_imu')
        self.sub = self.create_subscription(Imu, '/wit/imu', self.callback, 10)

    def callback(self, msg):
        r, p, y = euler_from_quaternion(
            (msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w))
        rad = 180.0 / math.pi
        self.get_logger().info(
            'Roll = %8.3f, Pitch = %8.3f, Yaw = %8.3f' % (r * rad, p * rad, y * rad))


def main(args=None):
    rclpy.init(args=args)
    node = GetImuRpy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
