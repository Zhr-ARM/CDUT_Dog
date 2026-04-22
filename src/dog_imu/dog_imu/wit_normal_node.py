#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WitMotion IMU - WIT standard protocol driver (ROS 2)."""
import math
import struct
import threading
import time

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler


def find_ttyUSB():
    posts = [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device]
    print('USB serial devices ({}): {}'.format(len(posts), posts))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def hex_to_short(raw):
    return list(struct.unpack("hhhh", bytearray(raw)))


def hex_to_int(raw):
    return list(struct.unpack("i", bytearray(raw)))


def hex_to_altitude(raw):
    return list(struct.unpack("h", bytearray(raw)))


BAUD_LIST = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
RATE_LIST = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 125, 200]

UNLOCK_CMD              = b'\xff\xaa\x69\x88\xb5'
RESET_MAGX_OFFSET_CMD   = b'\xff\xaa\x0b\x00\x00'
RESET_MAGY_OFFSET_CMD   = b'\xff\xaa\x0c\x00\x00'
RESET_MAGZ_OFFSET_CMD   = b'\xff\xaa\x0d\x00\x00'
ENTER_MAG_CALI_CMD      = b'\xff\xaa\x01\x09\x00'
EXIT_CALI_CMD           = b'\xff\xaa\x01\x00\x00'
SAVE_PARAM_CMD          = b'\xff\xaa\x00\x00\x00'
READ_MAG_OFFSET_CMD     = b'\xff\xaa\x27\x0b\x00'
READ_MAG_RANGE_CMD      = b'\xff\xaa\x27\x1c\x00'
RESET_MAG_PARAM_CMD     = b'\xff\xaa\x01\x07\x00'
SET_RSW_DEMO_CMD        = b'\xff\xaa\x02\x1f\x00'  # output time/acc/gyro/angle/mag


class WitNormalNode(Node):

    def __init__(self):
        super().__init__('wit_imu')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'base_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(
            'IMU Type: Normal Port:%s Baud:%d' % (self.port, self.baud))
        find_ttyUSB()

        self.buff = {}
        self.key = 0
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.magnetometer = [0, 0, 0]
        self.longitude = 0.0
        self.latitude = 0.0
        self.altitude = 0.0
        self.version = 0

        # mag-cali state
        self.cali_flag = False
        self.calibuff = []
        self.mag_offset = [0, 0, 0]
        self.mag_range = [500, 500, 500]
        self.read_reg = 0

        # raw record
        self.record_flag = False
        self.record_buff = bytearray()

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
        self.location_pub = self.create_publisher(NavSatFix, 'wit/location', 10)
        self.cali_sub = self.create_subscription(String, 'wit/cali', self.cali_callback, 10)

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    # -------- serial read loop --------
    def read_loop(self):
        while rclpy.ok():
            try:
                buff_count = self.wt_imu.inWaiting()
                if buff_count > 0:
                    buff_data = self.wt_imu.read(buff_count)
                    if self.record_flag:
                        self.record_buff += buff_data
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
        if self.key < 11:
            return

        data_buff = list(self.buff.values())
        angle_flag = False

        try:
            if self.buff[1] == 0x51:
                if check_sum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8
                                         for i in range(3)]
                else:
                    self.get_logger().warn('0x51 check failure')

            elif self.buff[1] == 0x52:
                if check_sum(data_buff[0:10], data_buff[10]):
                    self.angular_velocity = [hex_to_short(data_buff[2:10])[i] / 32768.0
                                             * 2000 * math.pi / 180 for i in range(3)]
                else:
                    self.get_logger().warn('0x52 check failure')

            elif self.buff[1] == 0x53:
                if check_sum(data_buff[0:10], data_buff[10]):
                    temp = hex_to_short(data_buff[2:10])
                    self.angle_degree = [temp[i] / 32768.0 * 180 for i in range(3)]
                    self.version = temp[3]
                    angle_flag = True
                else:
                    self.get_logger().warn('0x53 check failure')

            elif self.buff[1] == 0x54:
                if check_sum(data_buff[0:10], data_buff[10]):
                    self.magnetometer = hex_to_short(data_buff[2:10])
                    if self.cali_flag:
                        self.calibuff.append(self.magnetometer[0:2])
                else:
                    self.get_logger().warn('0x54 check failure')

            elif self.buff[1] == 0x57:
                if check_sum(data_buff[0:10], data_buff[10]):
                    lon_raw = hex_to_int(data_buff[2:6])[0]
                    lat_raw = hex_to_int(data_buff[6:10])[0]
                    self.longitude = (lon_raw // 10000000.0 * 100) + ((lon_raw % 10000000) / 10000000.0)
                    self.latitude = (lat_raw // 10000000.0 * 100) + ((lat_raw % 10000000) / 10000000.0)
                else:
                    self.get_logger().warn('0x57 check failure')

            elif self.buff[1] == 0x58:
                if check_sum(data_buff[0:10], data_buff[10]):
                    self.altitude = hex_to_altitude(data_buff[2:4])[0] / 10.0
                else:
                    self.get_logger().warn('0x58 check failure')

            elif self.buff[1] == 0x5f:
                if check_sum(data_buff[0:10], data_buff[10]):
                    readval = hex_to_short(data_buff[2:10])
                    if self.read_reg == 0x0b:
                        self.mag_offset = readval
                    else:
                        self.mag_range = readval
                    self.get_logger().info('reg read: %s' % str(readval))
                else:
                    self.get_logger().warn('0x5f check failure')
        finally:
            self.buff = {}
            self.key = 0

        if angle_flag:
            self.publish_messages()

    def publish_messages(self):
        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id

        angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
        qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
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

        loc_msg = NavSatFix()
        loc_msg.header.stamp = stamp
        loc_msg.header.frame_id = self.frame_id
        loc_msg.longitude = float(self.longitude)
        loc_msg.latitude = float(self.latitude)
        loc_msg.altitude = float(self.altitude)
        self.location_pub.publish(loc_msg)

    # -------- /wit/cali command callback --------
    def cali_callback(self, data):
        cmd = data.data
        self.get_logger().info('cali cmd: %s' % cmd)
        if 'mag' in cmd:
            for c in (UNLOCK_CMD, RESET_MAGX_OFFSET_CMD, RESET_MAGY_OFFSET_CMD,
                      RESET_MAGZ_OFFSET_CMD, RESET_MAG_PARAM_CMD, ENTER_MAG_CALI_CMD):
                self.wt_imu.write(c)
                time.sleep(0.1)
            self.cali_flag = True
            self.calibuff = []
            self.mag_offset = [0, 0, 0]
            self.mag_range = [500, 500, 500]

        elif 'exti' in cmd:
            self.cali_flag = False
            self.wt_imu.write(UNLOCK_CMD); time.sleep(0.1)
            self.wt_imu.write(EXIT_CALI_CMD); time.sleep(0.1)
            self.wt_imu.write(SAVE_PARAM_CMD); time.sleep(1)
            self.read_reg = 0x0b
            self.wt_imu.write(READ_MAG_OFFSET_CMD); time.sleep(1)
            self.read_reg = 0x1c
            self.wt_imu.write(READ_MAG_RANGE_CMD); time.sleep(1)
            datalen = len(self.calibuff)
            self.get_logger().info('cali data %d' % datalen)
            if datalen > 0:
                r = []
                for i in range(datalen):
                    tx = ((self.calibuff[i][0] - self.mag_offset[0]) * 2 / float(self.mag_range[0]))
                    ty = ((self.calibuff[i][1] - self.mag_offset[1]) * 2 / float(self.mag_range[1]))
                    r.append(abs(tx * tx + ty * ty - 1))
                r_n = float(sum(r)) / datalen
                if r_n < 0.05:
                    self.get_logger().info('mag calibration: very good')
                elif r_n < 0.1:
                    self.get_logger().info('mag calibration: good')
                else:
                    self.get_logger().warn('mag calibration: bad, please retry')

        elif 'version' in cmd:
            self.get_logger().info('sensor version: %s' % self.version)

        elif 'begin' in cmd:
            threading.Thread(target=self._record_thread, daemon=True).start()

        elif 'stop' in cmd:
            self.record_flag = False

        elif 'rate' in cmd:
            try:
                rate = float(cmd[4:])
                if rate in RATE_LIST:
                    val = RATE_LIST.index(rate) + 1
                    out = bytearray([0xff, 0xaa, 0x03, val, 0x00])
                    self.wt_imu.write(UNLOCK_CMD); time.sleep(0.1)
                    self.wt_imu.write(out)
                    self.get_logger().info('change rate to %s Hz' % rate)
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

    def _record_thread(self):
        self.record_flag = True
        self.record_buff = bytearray()
        fname = time.strftime('%Y%m%d%H%M%S', time.localtime()) + '.bin'
        self.get_logger().info('begin recording: %s' % fname)
        with open(fname, 'wb') as fd:
            while self.record_flag and rclpy.ok():
                if len(self.record_buff):
                    fd.write(bytes(self.record_buff))
                    self.record_buff = bytearray()
                else:
                    time.sleep(1)
        self.get_logger().info('stop recording')


def main(args=None):
    rclpy.init(args=args)
    node = WitNormalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
