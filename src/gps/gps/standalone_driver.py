#!/usr/bin/env python3
from argparse import ArgumentParser
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rover_msgs.msg import Customgps
from gps.gps_reader import GpsReader

class GPSPublisher(Node):

    def __init__(self, name, port):
        super().__init__(name)
        self.publisher_ = self.create_publisher(
            Customgps, 
            'gps', 
            10)
        stream = serial.Serial(port, timeout=1)
        self.gps = GpsReader(stream)
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        data = next(self.gps)

        ts = data.timestamp
        sec = ts[0]
        nsec = ts[1]

        utm = data.to_utm()

        header = Header(
            frame_id='GPS1_Frame',
            stamp=rclpy.Time(sec, nsec)
        )

        msg = Customgps(
            header=header,
            latitude=data.latitude,
            longitude=data.longitude,
            altitude=data.altitude,
            utm_easting=utm.easting,
            utm_northing=utm.northing,
            zone=utm.zone,
            letter=utm.letter,
            hdop=data.hdop,
            gpgga_read=data.raw,
        )

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1

def main(name = 'gps_node', port = '/dev/ttyUSB0'):
    rclpy.init()

    node = GPSPublisher(name, port)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    parser = ArgumentParser(description='Parse Gps puck output and publish ROS messages')
    parser.add_argument('port', type=str, help='serial port of the Gps device')
    parser.add_argument('name', type=str)
    parser.add_argument('log', type=str)
    args = parser.parse_args()

    try:
        main(name=args.name)
    except rclpy.ROSInterruptException:
        pass
