#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert raw IMU log (binary 0x55 frames) to a TSV text file."""
import struct
import sys
import time


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def hex_to_short(raw):
    return list(struct.unpack("hhhh", bytearray(raw)))


def main():
    if len(sys.argv) < 2:
        print('please input convert file name')
        sys.exit(1)

    filename = sys.argv[1]
    out_name = time.strftime('%Y%m%d%H%M%S', time.localtime()) + '.txt'

    head_msg = ''
    msg = ''
    head_index = 0x50
    head_flag = 0
    key = 0
    raw = bytearray(11)

    fd = open(out_name, 'w+')

    def convert_byte(val):
        nonlocal head_flag, head_index, msg, head_msg, key, raw
        raw[key] = val
        key += 1
        if raw[0] != 0x55:
            key = 0
            return
        if key < 11:
            return

        if check_sum(raw[0:10], raw[10]):
            if head_index == raw[1]:
                if head_flag == 1:
                    fd.write(head_msg + '\n')
                    head_flag = 2
                fd.write(msg + '\n')
                msg = ''
            if head_flag == 0:
                head_flag = 1
                head_index = raw[1]

            v = hex_to_short(raw[2:10])
            if raw[1] == 0x50:
                if head_flag == 1:
                    head_msg += 'Chip-Time\t'
                ms = raw[9] * 256 + raw[8]
                msg += '20{:0>2d}-{:0>2d}-{:0>2d} {:0>2d}:{:0>2d}:{:0>2d}.{:0>3d}\t'.format(
                    raw[2], raw[3], raw[4], raw[5], raw[6], raw[7], ms)
            elif raw[1] == 0x51:
                if head_flag == 1:
                    head_msg += 'ax(g)\tay(g)\taz(g)\t'
                msg += '{:.3f}\t{:.3f}\t{:.3f}\t'.format(v[0]/2048.0, v[1]/2048.0, v[2]/2048.0)
            elif raw[1] == 0x52:
                if head_flag == 1:
                    head_msg += 'wx(deg/s)\twy(deg/s)\twz(deg/s)\t'
                msg += '{:.3f}\t{:.3f}\t{:.3f}\t'.format(
                    v[0]/32768.0*2000.0, v[1]/32768.0*2000.0, v[2]/32768.0*2000.0)
            elif raw[1] == 0x53:
                if head_flag == 1:
                    head_msg += 'AngleX(deg)\tAngleY(deg)\tAngleZ(deg)\t'
                msg += '{:.3f}\t{:.3f}\t{:.3f}\t'.format(
                    v[0]/32768.0*180.0, v[1]/32768.0*180.0, v[2]/32768.0*180.0)
            elif raw[1] == 0x54:
                if head_flag == 1:
                    head_msg += ' hx\thy\thz\t'
                msg += '{:.0f}\t{:.0f}\t{:.0f}\t'.format(v[0], v[1], v[2])
        key = 0

    with open(filename, 'rb') as f:
        try:
            while True:
                data = f.read(1024)
                if not data:
                    break
                for b in data:
                    convert_byte(b)
        except Exception as e:
            print(e)

    fd.close()
    print('convert {} -> {} done'.format(filename, out_name))


if __name__ == '__main__':
    main()
