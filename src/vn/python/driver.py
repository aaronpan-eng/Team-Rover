#!/usr/bin/env python3
from argparse import ArgumentParser
import serial

import rospy
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from vn_driver.msg import Vectornav
from vectornav import NavDriver
import vectornav


def convert_to_quaternion(euler: tuple) -> Quaternion:
    """Convert euler angle orientation to quaternion

    Inputs
    * euler: 3-tuple of euler angle floats in YPR order

    Returns
    * Quaternion
    """
    quat = nav.convert_to_quaternion(euler)

    return Quaternion(
        x=quat[0],
        y=quat[1],
        z=quat[2],
        w=quat[3],
    )


def publish_from_device(port: str):
    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    rospy.init_node('vn_driver', anonymous=True)
    rate = rospy.Rate(40)

    stream = serial.Serial(port, timeout=1)
    seq = 0

    nav = NavDriver(stream)
    # this configures the device by writing to registers
    rospy.loginfo('Configuring device')
    nav.setup()
    rospy.loginfo('Done configuring')

    while not rospy.is_shutdown():
        data = next(nav)

        # TODO use updated lab5 timestamp
        raise NotImplementedError('need to fix vectornav time conversion')
        sec = None
        nsec = None
        timestamp = rospy.Time(sec, nsec)

        header = Header(
            frame_id='imu1_frame',
            stamp=timestamp,
            seq=seq,
        )

        imu = Imu(
            header=header,
            # VN gives in YPR
            orientation=convert_to_quaternion(data.orientation),
            orientation_covariance=[0] * 9,
            # VN gives in rad/s
            # ROS wants rad/s
            angular_velocity=Vector3(
                x=data.gyro[0],
                y=data.gyro[1],
                z=data.gyro[2],
            ),
            angular_velocity_covariance=[0] * 9,
            # VN gives in m/s^2
            # ROS wants m/s^2
            linear_acceleration=Vector3(
                x=data.accel[0],
                y=data.accel[1],
                z=data.accel[2],
            ),
            linear_acceleration_covariance=[0] * 9,
        )

        mag = MagneticField(
            header=header,
            # VN gives in Gauss
            # ROS wants Tesla
            magnetic_field=Vector3(
                x=data.mag[0] / 10000,
                y=data.mag[1] / 10000,
                z=data.mag[2] / 10000,
            ),
            magnetic_field_covariance=[0] * 9,
        )

        msg = Vectornav(
            header=header,
            imu=imu,
            mag_field=mag,
            raw=data.raw,
        )

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

        seq += 1

    nav.close()


if __name__ == '__main__':
    parser = ArgumentParser(description='Parse VectorNav output and publish ROS messages')
    parser.add_argument('port', type=str, help='serial port of the VectorNav device')
    parser.add_argument('name', type=str)
    parser.add_argument('log', type=str)
    args = parser.parse_args()

    try:
        publish_from_device(args.port)
    except rospy.ROSInterruptException:
        pass
