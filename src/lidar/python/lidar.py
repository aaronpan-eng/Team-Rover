from dataclasses import dataclass, asdict
from typing import Union, TextIO

import serial


@dataclass
class LidarMsg:
    raw: str
    timestamp: None  # TODO decide data type for timestamp

    # TODO add other parsed data and types to this

    @classmethod
    def from_str(cls, raw: str) -> "LidarMsg":
        (header,
         data_1,  # TODO are we parsing the serial line manually or using another package?
         data_2
         ) = raw.strip().split(',')

        raise NotImplementedError('lidar message parse')

        return cls(
            raw=raw,
            timestamp=None,  # TODO fix timestamp
            # TODO add other parsed data
        )

    to_dict = asdict


class Lidar:
    stream: Union[serial.SerialBase, TextIO]

    def __init__(self, stream: Union[serial.SerialBase, TextIO]):
        self.stream = stream

    def setup(self):
        # TODO do we need to setup the lidar device before using?
        raise NotImplementedError('lidar setup')

    def write(self, line: str):
        # TODO do we ever need to write to the lidar device?
        raise NotImplementedError('lidar write')

    def __next__(self) -> LidarMsg:
        while b := self.stream.readline():
            if isinstance(b, bytes):
                line = b.decode('ascii')
            else:
                line = b

            try:
                return LidarMsg.from_str(line)
            except ValueError as e:
                print(f'Error {e}, skipping "{line.strip()}"')
        raise StopIteration

    def __bool__(self):
        return self.stream.readable()
        # TODO change to this if we need to write to lidar
        # return self.stream.readable() and self.stream.writable()

    def __iter__(self):
        return self

    def close(self):
        self.stream.close()


# This runs only when you execute this file directly on the commandline like `$ python3 lidar.py`
if __name__ == '__main__':
    import os
    from argparse import ArgumentParser

    parser = ArgumentParser(description='Driver for a serial lidar device or recorded file of messages')
    parser.add_argument('port_or_file', type=str, help='serial port of the device '
                                                       'or file of recorded messages')
    args = parser.parse_args()

    if os.path.isfile(args.port_or_file):
        # read from log file
        file = args.port_or_file
        stream = open(file, 'r')
        driver = Lidar(stream)
        for msg in driver:
            print(msg)

    else:
        # read from device
        port = args.port_or_file
        print(f'Setting up lidar on port {port}')
        stream = serial.Serial(port, timeout=1)
        driver = Lidar(stream)
        driver.setup()
        for msg in driver:
            print(msg)
