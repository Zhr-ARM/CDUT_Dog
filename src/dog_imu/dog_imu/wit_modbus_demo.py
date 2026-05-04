#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Console demo for Modbus-RTU protocol (no ROS dependency)."""
import math
import sys
import time

import serial
import serial.tools.list_ports
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


def find_ttyUSB():
    posts = [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device]
    print('USB serial devices ({}): {}'.format(len(posts), posts))


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

    master = modbus_rtu.RtuMaster(ser)
    master.set_timeout(0.1)
    master.set_verbose(True)

    while True:
        time.sleep(0.01)
        try:
            reg = master.execute(80, cst.READ_HOLDING_REGISTERS, 52, 12)
        except Exception as e:
            print(e)
            print('\033[31mread register timeout, check connection or baudrate\033[0m')
            continue

        v = [0] * 12
        for i in range(12):
            v[i] = reg[i] - 65536 if reg[i] > 32767 else reg[i]

        acceleration = [v[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
        angular_velocity = [v[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3, 6)]
        magnetometer = v[6:9]
        angle_degree = [v[i] / 32768.0 * 180 for i in range(9, 12)]

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


if __name__ == '__main__':
    main()
