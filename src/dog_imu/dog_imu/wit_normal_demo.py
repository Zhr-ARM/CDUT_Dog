#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Console demo for WIT standard protocol (no ROS dependency)."""
import math
import struct
import sys

import serial
import serial.tools.list_ports


def find_ttyUSB():
    posts = [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device]
    print('USB serial devices ({}): {}'.format(len(posts), posts))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def hex_to_short(raw):
    return list(struct.unpack("hhhh", bytearray(raw)))


key = 0
buff = {}
acceleration = [0.0, 0.0, 0.0]
angular_velocity = [0.0, 0.0, 0.0]
angle_degree = [0.0, 0.0, 0.0]
magnetometer = [0, 0, 0]


def handle_serial_data(raw_byte):
    global buff, key, acceleration, angular_velocity, angle_degree, magnetometer
    angle_flag = False
    buff[key] = raw_byte
    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:
        return

    data_buff = list(buff.values())
    if buff[1] == 0x51:
        if check_sum(data_buff[0:10], data_buff[10]):
            acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(3)]
        else:
            print('0x51 check failure')
    elif buff[1] == 0x52:
        if check_sum(data_buff[0:10], data_buff[10]):
            angular_velocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180
                                for i in range(3)]
        else:
            print('0x52 check failure')
    elif buff[1] == 0x53:
        if check_sum(data_buff[0:10], data_buff[10]):
            angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(3)]
            angle_flag = True
        else:
            print('0x53 check failure')
    elif buff[1] == 0x54:
        if check_sum(data_buff[0:10], data_buff[10]):
            magnetometer = hex_to_short(data_buff[2:10])
        else:
            print('0x54 check failure')
    else:
        buff = {}
        key = 0
        return

    buff = {}
    key = 0
    if angle_flag:
        print('''
    Acceleration (m/s^2):
        x: %.2f   y: %.2f   z: %.2f
    Angular velocity (rad/s):
        x: %.2f   y: %.2f   z: %.2f
    Euler angle (deg):
        x: %.2f   y: %.2f   z: %.2f
    Magnetic:
        x: %.2f   y: %.2f   z: %.2f
''' % (acceleration[0], acceleration[1], acceleration[2],
       angular_velocity[0], angular_velocity[1], angular_velocity[2],
       angle_degree[0], angle_degree[1], angle_degree[2],
       magnetometer[0], magnetometer[1], magnetometer[2]))


def main():
    find_ttyUSB()
    port = '/dev/ttyUSB0' if sys.platform.startswith('linux') else 'COM3'
    baud = 9600
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)
        if not ser.isOpen():
            ser.open()
        print('\033[32mport open success...\033[0m')
    except Exception as e:
        print(e)
        print('\033[31mport open failed\033[0m')
        return

    while True:
        try:
            n = ser.inWaiting()
        except Exception as e:
            print('exception: ' + str(e))
            return
        if n > 0:
            data = ser.read(n)
            for b in data:
                handle_serial_data(b)


if __name__ == '__main__':
    main()
