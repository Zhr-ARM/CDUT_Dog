#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive command publisher for /wit/cali (ROS 2)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP_TEXT = """----------------------------
0:exti cali mode
9:enter mag cali mode
h:show cmd help
e:exti sys
v:show version
b:begin recording
s:stop recording
rate:set 0.2~200Hz output  (e.g. rate100)
baud:set 4800~230400 baud  (e.g. baud115200)
rsw:set output data <time,acc,gyro,angle,mag>
----------------------------"""


class WitImuCtrl(Node):

    def __init__(self):
        super().__init__('wit_imu_ctrl')
        self.pub = self.create_publisher(String, 'wit/cali', 10)


def main(args=None):
    rclpy.init(args=args)
    node = WitImuCtrl()
    print('please input your cmd:')
    print(HELP_TEXT)
    msg = String()
    try:
        while rclpy.ok():
            try:
                line = input('input cmd:')
            except EOFError:
                break
            if 'rate' in line:
                msg.data = line
                node.pub.publish(msg); print('change ' + line)
            elif 'rsw' in line:
                msg.data = 'rsw'; node.pub.publish(msg)
            elif 'baud' in line:
                msg.data = line
                node.pub.publish(msg); print('change ' + line)
            elif 'b' in line:
                msg.data = 'begin'; node.pub.publish(msg); print('begin recording')
            elif 's' in line:
                msg.data = 'stop'; node.pub.publish(msg); print('stop recording')
            elif 'v' in line:
                msg.data = 'version'; node.pub.publish(msg); print('show sensor version')
            elif 'h' in line:
                print(HELP_TEXT)
            elif 'e' in line:
                print('exti sys'); break
            elif '0' in line:
                msg.data = 'exti'; node.pub.publish(msg); print('exti cali mode')
            elif '9' in line:
                msg.data = 'mag'; node.pub.publish(msg); print('enter mag cali mode')
            else:
                print('{} cmd not supported'.format(line))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
