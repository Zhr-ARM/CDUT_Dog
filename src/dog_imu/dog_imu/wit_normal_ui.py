#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tk UI demo for WIT standard protocol (Python 3 / Tk)."""
import math
import struct
import sys
import threading
import tkinter as tk

import serial
import serial.tools.list_ports


def find_ttyUSB():
    posts = [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device]
    print('USB serial devices ({}): {}'.format(len(posts), posts))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def hex_to_short(raw):
    return list(struct.unpack("hhhh", bytearray(raw)))


class App:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title('wit imu')
        self.window.geometry('640x360')
        frame = tk.Frame(self.window, height=345, width=625)
        frame.place(x=5, y=5)
        self.show_text = tk.Text(frame, height=700, bg='white', font=('Arial', 12))
        self.show_text.place(x=4, y=4)

        self.key = 0
        self.buff = {}
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.magnetometer = [0, 0, 0]

    def show(self, text):
        self.show_text.delete(0.0, tk.END)
        self.show_text.insert(tk.INSERT, text)

    def handle(self, raw_byte):
        self.buff[self.key] = raw_byte
        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            return
        if self.key < 11:
            return
        data = list(self.buff.values())
        flag = False
        if self.buff[1] == 0x51 and check_sum(data[0:10], data[10]):
            self.acceleration = [hex_to_short(data[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(3)]
        elif self.buff[1] == 0x52 and check_sum(data[0:10], data[10]):
            self.angular_velocity = [hex_to_short(data[2:10])[i] / 32768.0 * 2000 * math.pi / 180
                                     for i in range(3)]
        elif self.buff[1] == 0x53 and check_sum(data[0:10], data[10]):
            self.angle_degree = [hex_to_short(data[2:10])[i] / 32768.0 * 180 for i in range(3)]
            flag = True
        elif self.buff[1] == 0x54 and check_sum(data[0:10], data[10]):
            self.magnetometer = hex_to_short(data[2:10])

        self.buff = {}
        self.key = 0
        if flag:
            self.show('''
    Acceleration (m/s^2):
        x: %.2f   y: %.2f   z: %.2f
    Angular velocity (rad/s):
        x: %.2f   y: %.2f   z: %.2f
    Euler angle (deg):
        x: %.2f   y: %.2f   z: %.2f
    Magnetic:
        x: %.2f   y: %.2f   z: %.2f
''' % (self.acceleration[0], self.acceleration[1], self.acceleration[2],
       self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2],
       self.angle_degree[0], self.angle_degree[1], self.angle_degree[2],
       self.magnetometer[0], self.magnetometer[1], self.magnetometer[2]))

    def loop(self, ser):
        while True:
            try:
                n = ser.inWaiting()
            except Exception as e:
                print('exception: ' + str(e))
                self.window.quit()
                return
            if n > 0:
                for b in ser.read(n):
                    self.handle(b)


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

    app = App()
    t = threading.Thread(target=app.loop, args=(ser,), daemon=True)
    t.start()
    app.window.mainloop()


if __name__ == '__main__':
    main()
