#!/usr/bin/env python

import sys
import rospy
import math
import serial
import numpy as np

from vn.msg import Vectornav
from gps.msg import Customgps
from rover.msg import Localization

if __name__ == '__main__':
    # Initialize node, message, and publisher for localization
    rospy.init_node('localization')
    localization = Localization()
    localization_pub = rospy.Publisher('/localization', Localization, queue_size = 5)

    # Initialize subscribers from imu and gps
    imu = rospy.Subscriber('/imu', Vectornav, queue_size=5)
    gps = rospy.Subscriber('/gps', Customgps, queue_size=5)

    while not rospy.is_shutdown():
        # Extracting messages
        northing = gps.utm_northing
        easting = gps.utm_easting

        yaw = imu.imu.orientation.z
        mag_x = imu.mag_field.magnetic_field.x
        mag_y = imu.mag_field.magnetic_field.y

        time_gps = gps.header.stamp
        time_gps = time_imu.to_sec()

        time_imu = imu.stamp
        time_imu = time_imu.to_sec()

        # TODO can add calibration or other calculations here

        # Setting messages to publish
        localization.time_imu = time_imu
        localization.time_gps = time_gps
        localization.northing = northing
        localization.easting = easting
        localization.yaw = yaw
        localization.mag_x = mag_x
        localization.mag_y = mag_y

        # Publishing
        localization_pub.publish(localization)