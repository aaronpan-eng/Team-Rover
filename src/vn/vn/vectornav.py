from dataclasses import dataclass, asdict
from datetime import datetime
from math import sqrt, cos, sin
from typing import Union, TextIO

import numpy as np
import serial


def convert_to_quaternion(euler: tuple) -> np.ndarray:
    """Convert euler angle orientation to ndarray quaternion

    Inputs
    * euler: 3-tuple of euler angle floats in YPR order

    Returns
    * 4x1 quaternion with scalar last
    """
    yaw = euler[0]
    pitch = euler[1]
    roll = euler[2]
    dcm = np.array([
        [1, 0, 0],
        [0, cos(roll), sin(roll)],
        [0, -sin(roll), cos(roll)],
    ]) @ np.array([
        [cos(pitch), 0, -sin(pitch)],
        [0, 1, 0],
        [sin(pitch), 0, cos(pitch)],
    ]) @ np.array([
        [cos(yaw), sin(yaw), 0],
        [-sin(yaw), cos(yaw), 0],
        [0, 0, 1],
    ])
    q_sq = (np.array([
        [1, 1, -1, -1],
        [1, -1, 1, -1],
        [1, -1, -1, 1],
        [1, 1, 1, 1],
    ]) @ np.array([
        [1],
        [dcm[0, 0]],
        [dcm[1, 1]],
        [dcm[2, 2]],
    ]) / 4).reshape((4,))

    idx = np.argmax(q_sq)
    if idx == 0:
        quat = np.array([
            4 * q_sq[0],
            dcm[0, 1] + dcm[1, 0],
            dcm[2, 0] + dcm[0, 2],
            dcm[1, 2] - dcm[2, 1],
        ]) / (4 * sqrt(q_sq[0]))
    elif idx == 1:
        quat = np.array([
            dcm[0, 1] + dcm[1, 0],
            4 * q_sq[1],
            dcm[1, 2] + dcm[2, 1],
            dcm[2, 0] - dcm[0, 2],
        ]) / (4 * sqrt(q_sq[1]))
    elif idx == 2:
        quat = np.array([
            dcm[2, 0] + dcm[0, 2],
            dcm[1, 2] + dcm[2, 1],
            4 * q_sq[2],
            dcm[0, 1] - dcm[1, 0],
        ]) / (4 * sqrt(q_sq[2]))
    else:
        quat = np.array([
            dcm[1, 2] - dcm[2, 1],
            dcm[2, 0] - dcm[0, 2],
            dcm[0, 1] - dcm[1, 0],
            4 * q_sq[3],
        ]) / (4 * sqrt(q_sq[3]))

    return quat


def checksum(line: str) -> int:
    chk = 0
    if line[0] != '$':
        raise ValueError(f'checksum line should start with $ but starts with {line[0]}')
    if '*' in line:
        raise ValueError("checksum line should have the * and tail removed but didn't")
    for c in line[1:].encode('ascii'):
        chk ^= c
    return chk


@dataclass
class NavMsg:
    raw: str
    timestamp: datetime
    accel: (float, float, float)
    gyro: (float, float, float)
    orientation: (float, float, float)
    mag: (float, float, float)
    header: str = '$VNYMR'

    @classmethod
    def from_str(cls, raw: str) -> "NavMsg":
        (msg, chksm) = raw.strip().split('*')
        (header,
         ori_y,
         ori_p,
         ori_r,
         gyro_x,
         gyro_y,
         gyro_z,
         accel_x,
         accel_y,
         accel_z,
         mag_x,
         mag_y,
         mag_z) = msg.split(',')

        if header != cls.header:
            raise ValueError(f'not a {cls.header} message, header was {header}')

        timestamp = datetime.now()

        try:
            accel = (
                float(accel_x),
                float(accel_y),
                float(accel_z),
            )
        except ValueError:
            raise ValueError(f'invalid accel {accel_x},{accel_y},{accel_z}')

        try:
            gyro = (
                float(gyro_x),
                float(gyro_y),
                float(gyro_z),
            )
        except ValueError:
            raise ValueError(f'invalid gyro {gyro_x},{gyro_y},{gyro_z}')

        try:
            ori = (
                float(ori_r),
                float(ori_p),
                float(ori_y),
            )
        except ValueError:
            raise ValueError(f'invalid orientation {ori_r},{ori_p},{ori_y}')

        try:
            mag = (
                float(mag_x),
                float(mag_y),
                float(mag_z),
            )
        except ValueError:
            raise ValueError(f'invalid mag {mag_x},{mag_y},{mag_z}')

        try:
            chksm = int(chksm, base=16)
        except ValueError:
            raise ValueError(f'invalid checksum {chksm}')
        c = checksum(msg)
        if chksm != c:
            raise ValueError(f'invalid checksum {chksm}, expected {c}')

        return cls(
            raw=raw,
            timestamp=timestamp,
            accel=accel,
            gyro=gyro,
            orientation=ori,
            mag=mag,
        )

    to_dict = asdict


class NavDriver:
    stream: Union[serial.SerialBase, TextIO]

    def __init__(self, stream: Union[serial.SerialBase, TextIO]):
        self.stream = stream

    def setup(self):
        # Write to reg
        reg = 75
        # async mode 2 -> send to port 2 at fixed rate
        async_mode = 2
        # rate divisor 20 -> 800/20 = 40Hz
        rate_divisor = 20
        # group 1 only
        groups = 0b0000_0001
        # group 1 field
        # b3 YawPitchRoll
        # b5 AngularRate
        # b8 Accel
        # b10 MagPres
        group_1_field = 0b0000_0101_0010_1000
        self.write(f'$VNWRG,{reg:02},{async_mode:01},{rate_divisor:02},{groups:02x},{group_1_field:04x}')

    def write(self, line: str):
        chksm = checksum(line)
        line_with_chksm = line + f'*{chksm:02x}\r\n'
        self.stream.write(line_with_chksm.encode('ascii'))

    def __next__(self) -> NavMsg:
        while b := self.stream.readline():
            if isinstance(b, bytes):
                header_idx = b.rfind(b'$')
                if header_idx == -1:
                    continue
                try:
                    line = b[header_idx:].decode('ascii')
                except UnicodeDecodeError:
                    print('Failed to decode:', repr(b))
                    continue
            else:
                header_idx = b.rfind('$')
                if header_idx == -1:
                    continue
                line = b[header_idx:]

            try:
                return NavMsg.from_str(line)
            except ValueError as e:
                print(f'Error {e}, skipping "{line.strip()}"')
        raise StopIteration

    def __bool__(self):
        return self.stream.readable() and self.stream.writable()

    def __iter__(self):
        return self

    def close(self):
        self.stream.close()


# Just a quick example/test
if __name__ == '__main__':
    import sys

    port = sys.argv[1]
    print(f'Setting up VectorNav on port {port}')

    # stream = serial.Serial(port, timeout=1)
    # driver = NavDriver(stream)
    # driver.setup()

    file = sys.argv[1]
    stream = open(file, 'r')
    driver = NavDriver(stream)
    for msg in driver:
        print(msg)
        print(convert_to_quaternion(msg.orientation))
