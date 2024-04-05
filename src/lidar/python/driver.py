#!/usr/bin/env python3
from argparse import ArgumentParser
import serial

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from lidar import Lidar


def publish_from_device(port: str):
    pub = rospy.Publisher('lidar', LaserScan, queue_size=10)
    rospy.init_node('lidar_driver', anonymous=True)
    rate = rospy.Rate(40)  # TODO set this to the rate from the lidar unit

    stream = serial.Serial(port, timeout=1)
    seq = 0

    device = Lidar(stream)

    # TODO is it necessary to configure the lidar unit before using?
    rospy.loginfo('Configuring device')
    device.setup()
    rospy.loginfo('Done configuring')

    while not rospy.is_shutdown():
        data = next(device)

        # TODO where do lidar msg timestamps come from?
        raise NotImplementedError('where do lidar msg timestamps come from')
        sec = None
        nsec = None
        timestamp = rospy.Time(sec, nsec)

        header = Header(
            frame_id='lidar1_frame',
            stamp=timestamp,
            seq=seq,
        )

        msg = LaserScan(
            header=header,
            # TODO convert `data` to LaserScan msg
        )

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

        seq += 1

    device.close()


if __name__ == '__main__':
    parser = ArgumentParser(description='Parse Lidar output and publish ROS messages')
    parser.add_argument('port', type=str, help='serial port of the Lidar device')
    parser.add_argument('name', type=str)
    parser.add_argument('log', type=str)
    args = parser.parse_args()

    try:
        publish_from_device(args.port)
    except rospy.ROSInterruptException:
        pass
