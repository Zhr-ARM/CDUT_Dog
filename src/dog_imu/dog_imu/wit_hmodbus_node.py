#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WitMotion IMU - Modbus high-precision driver (ROS 2)."""
import math

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_from_euler

import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


def find_ttyUSB():
    posts = [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device]
    print('USB serial devices ({}): {}'.format(len(posts), posts))


class WitHModbusNode(Node):

    def __init__(self):
        super().__init__('wit_imu')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('slave_id', 80)
        self.declare_parameter('poll_period', 0.01)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value
        self.poll_period = self.get_parameter('poll_period').get_parameter_value().double_value

        self.get_logger().info(
            'IMU Type: HModbus Port:%s Baud:%d' % (self.port, self.baud))
        find_ttyUSB()

        try:
            self.wt_imu = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.5)
            if not self.wt_imu.isOpen():
                self.wt_imu.open()
            self.get_logger().info('\033[32mport open success...\033[0m')
        except Exception as e:
            self.get_logger().error('\033[31mport open failed: %s\033[0m' % str(e))
            raise

        self.master = modbus_rtu.RtuMaster(self.wt_imu)
        self.master.set_timeout(1)
        self.master.set_verbose(True)

        self.imu_pub = self.create_publisher(Imu, 'wit/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'wit/mag', 10)

        self.timer = self.create_timer(self.poll_period, self.poll_once)

    def poll_once(self):
        try:
            reg = self.master.execute(self.slave_id, cst.READ_HOLDING_REGISTERS, 52, 15)
        except Exception as e:
            self.get_logger().warn('read register timeout: %s' % str(e))
            return

        v = [0] * 12
        for i in range(0, 9):
            v[i] = reg[i] - 65536 if reg[i] > 32767 else reg[i]
        v[9]  = reg[9]  + reg[10] * 65536
        v[10] = reg[11] + reg[12] * 65536
        v[11] = reg[13] + reg[14] * 65536

        acceleration = [v[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
        angular_velocity = [v[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3, 6)]
        magnetometer = v[6:9]

        angle_degree = [0.0, 0.0, 0.0]
        for i in range(9, 12):
            if v[i] > 2147483647:
                v[i] = v[i] - 4294967296
            angle_degree[i - 9] = v[i] / 1000.0

        stamp = self.get_clock().now().to_msg()
        angle_radian = [a * math.pi / 180 for a in angle_degree]
        qua = quaternion_from_euler(*angle_radian)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.x = float(qua[0])
        imu_msg.orientation.y = float(qua[1])
        imu_msg.orientation.z = float(qua[2])
        imu_msg.orientation.w = float(qua[3])
        imu_msg.angular_velocity.x = float(angular_velocity[0])
        imu_msg.angular_velocity.y = float(angular_velocity[1])
        imu_msg.angular_velocity.z = float(angular_velocity[2])
        imu_msg.linear_acceleration.x = float(acceleration[0])
        imu_msg.linear_acceleration.y = float(acceleration[1])
        imu_msg.linear_acceleration.z = float(acceleration[2])
        self.imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = float(magnetometer[0])
        mag_msg.magnetic_field.y = float(magnetometer[1])
        mag_msg.magnetic_field.z = float(magnetometer[2])
        self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WitHModbusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
