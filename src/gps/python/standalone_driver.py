#!/usr/bin/env python3
from argparse import ArgumentParser
import serial

import rospy
from std_msgs.msg import Header
from gps_driver.msg import Customgps
from gps_reader import GpsReader


def publish_from_device(port: str):
    pub = rospy.Publisher('gps', Customgps, queue_size=10)
    rospy.init_node('gps_driver', anonymous=True)
    rate = rospy.Rate(10)

    stream = serial.Serial(port, timeout=1)
    rdr = GpsReader(stream)
    seq = 0

    while not rospy.is_shutdown():
        data = next(rdr)

        ts = data.timestamp
        sec = ts[0]
        nsec = ts[1]

        utm = data.to_utm()

        header = Header(
            frame_id='GPS1_Frame',
            stamp=rospy.Time(sec, nsec),
            seq=seq,
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

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

        seq += 1

    rdr.close()


if __name__ == '__main__':
    parser = ArgumentParser(description='Parse Gps puck output and publish ROS messages')
    parser.add_argument('port', type=str, help='serial port of the Gps device')
    parser.add_argument('name', type=str)
    parser.add_argument('log', type=str)
    args = parser.parse_args()

    try:
        publish_from_device(args.port)
    except rospy.ROSInterruptException:
        pass
