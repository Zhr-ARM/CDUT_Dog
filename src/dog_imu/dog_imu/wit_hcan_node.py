#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WitMotion IMU - CAN high-precision driver (ROS 2)."""
import math
import struct
import threading
import time

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler


def find_ttyUSB():
    posts = [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device]
    print('USB serial devices ({}): {}'.format(len(posts), posts))


def hex_to_short(raw):
    return list(struct.unpack("hhh", bytearray(raw)))


def hex_to_int(raw):
    return list(struct.unpack("i", bytearray(raw)))


BAUD_LIST = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
RATE_LIST = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200]

UNLOCK_CMD              = b'\xff\xaa\x69\x88\xb5'
RESET_MAGX_OFFSET_CMD   = b'\xff\xaa\x0b\x00\x00'
RESET_MAGY_OFFSET_CMD   = b'\xff\xaa\x0c\x00\x00'
RESET_MAGZ_OFFSET_CMD   = b'\xff\xaa\x0d\x00\x00'
ENTER_MAG_CALI_CMD      = b'\xff\xaa\x01\x09\x00'
EXIT_CALI_CMD           = b'\xff\xaa\x01\x00\x00'
SAVE_PARAM_CMD          = b'\xff\xaa\x00\x00\x00'
RESET_MAG_PARAM_CMD     = b'\xff\xaa\x01\x07\x00'
SET_RSW_DEMO_CMD        = b'\xff\xaa\x02\x1f\x00'


class WitHCanNode(Node):

    def __init__(self):
        super().__init__('wit_imu')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('frame_id', 'base_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(
            'IMU Type: HCAN Port:%s Baud:%d' % (self.port, self.baud))
        find_ttyUSB()

        self.buff = {}
        self.key = 0
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.magnetometer = [0, 0, 0]
        self.angle_tip = 0

        self.cali_flag = False
        self.calibuff = []
        self.mag_offset = [0, 0, 0]
        self.mag_range = [500, 500, 500]

        try:
            self.wt_imu = serial.Serial(port=self.port, baudrate=self.baud, timeout=10)
            if not self.wt_imu.isOpen():
                self.wt_imu.open()
            self.get_logger().info('\033[32mSerial port enabled successfully...\033[0m')
        except Exception as e:
            self.get_logger().error('\033[31mFailed to open serial port: %s\033[0m' % str(e))
            raise

        self.imu_pub = self.create_publisher(Imu, 'wit/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'wit/mag', 10)
        self.cali_sub = self.create_subscription(String, 'wit/cali', self.cali_callback, 10)

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        while rclpy.ok():
            try:
                buff_count = self.wt_imu.inWaiting()
                if buff_count > 0:
                    buff_data = self.wt_imu.read(buff_count)
                    for b in buff_data:
                        self.handle_serial_data(b)
            except Exception as e:
                self.get_logger().error('IMU read exception: %s' % str(e))
                return

    def handle_serial_data(self, raw_byte):
        self.buff[self.key] = raw_byte
        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            return
        if self.key < 8:
            return

        data_buff = list(self.buff.values())

        try:
            if self.buff[1] == 0x51:
                self.acceleration = [hex_to_short(data_buff[2:8])[i] / 32768.0 * 16 * 9.8
                                     for i in range(3)]
            elif self.buff[1] == 0x52:
                self.angular_velocity = [hex_to_short(data_buff[2:8])[i] / 32768.0
                                         * 2000 * math.pi / 180 for i in range(3)]
            elif self.buff[1] == 0x53:
                temp = hex_to_int(data_buff[4:8])
                axis = self.buff[2]
                if axis == 0x01:
                    self.angle_degree[0] = float(temp[0]) / 1000.0
                    self.angle_tip += 1
                elif axis == 0x02:
                    self.angle_degree[1] = float(temp[0]) / 1000.0
                    self.angle_tip += 1
                elif axis == 0x03:
                    self.angle_degree[2] = float(temp[0]) / 1000.0
                    self.angle_tip += 1
            elif self.buff[1] == 0x54:
                self.magnetometer = hex_to_short(data_buff[2:8])
                if self.cali_flag:
                    self.calibuff.append(self.magnetometer[0:2])
        finally:
            self.buff = {}
            self.key = 0

        if self.angle_tip == 3:
            self.publish_messages()
            self.angle_tip = 0

    def publish_messages(self):
        stamp = self.get_clock().now().to_msg()
        angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
        qua = quaternion_from_euler(*angle_radian)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.x = float(qua[0])
        imu_msg.orientation.y = float(qua[1])
        imu_msg.orientation.z = float(qua[2])
        imu_msg.orientation.w = float(qua[3])
        imu_msg.angular_velocity.x = float(self.angular_velocity[0])
        imu_msg.angular_velocity.y = float(self.angular_velocity[1])
        imu_msg.angular_velocity.z = float(self.angular_velocity[2])
        imu_msg.linear_acceleration.x = float(self.acceleration[0])
        imu_msg.linear_acceleration.y = float(self.acceleration[1])
        imu_msg.linear_acceleration.z = float(self.acceleration[2])
        self.imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = float(self.magnetometer[0])
        mag_msg.magnetic_field.y = float(self.magnetometer[1])
        mag_msg.magnetic_field.z = float(self.magnetometer[2])
        self.mag_pub.publish(mag_msg)

    def cali_callback(self, data):
        cmd = data.data
        self.get_logger().info('cali cmd: %s' % cmd)
        if 'mag' in cmd:
            for c in (UNLOCK_CMD, RESET_MAGX_OFFSET_CMD, RESET_MAGY_OFFSET_CMD,
                      RESET_MAGZ_OFFSET_CMD, RESET_MAG_PARAM_CMD, ENTER_MAG_CALI_CMD):
                self.wt_imu.write(c); time.sleep(0.1)
            self.cali_flag = True
        elif 'exti' in cmd:
            self.cali_flag = False
            self.wt_imu.write(UNLOCK_CMD); time.sleep(0.1)
            self.wt_imu.write(EXIT_CALI_CMD); time.sleep(0.1)
            self.wt_imu.write(SAVE_PARAM_CMD); time.sleep(1)
        elif 'rate' in cmd:
            try:
                rate = float(cmd[4:])
                if rate in RATE_LIST:
                    val = RATE_LIST.index(rate) + 1
                    out = bytearray([0xff, 0xaa, 0x03, val, 0x00])
                    self.wt_imu.write(UNLOCK_CMD); time.sleep(0.1)
                    self.wt_imu.write(out)
            except Exception as e:
                self.get_logger().error(str(e))
        elif 'baud' in cmd:
            try:
                baud = float(cmd[4:])
                if baud in BAUD_LIST:
                    val = BAUD_LIST.index(baud) + 1
                    out = bytearray([0xff, 0xaa, 0x04, val, 0x00])
                    self.wt_imu.write(UNLOCK_CMD); time.sleep(0.1)
                    self.wt_imu.write(out); time.sleep(0.1)
                    self.wt_imu.baudrate = int(baud)
            except Exception as e:
                self.get_logger().error(str(e))
        elif 'rsw' in cmd:
            self.wt_imu.write(UNLOCK_CMD); time.sleep(0.1)
            self.wt_imu.write(SET_RSW_DEMO_CMD); time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = WitHCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
