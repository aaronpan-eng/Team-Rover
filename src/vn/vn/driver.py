#!/usr/bin/env python3
from argparse import ArgumentParser
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from rover_msgs.msg import Vectornav
from vn.vectornav import NavDriver
import vn.vectornav as vn

class ImuPublisher(Node):

    def __init__(self, name, port):
        super().__init__(name)
        self.publisher_ = self.create_publisher(
            Vectornav, 
            'imu', 
            10)
        timer_period = 1/40  # seconds
        stream = serial.Serial(port, timeout=1)
        self.nav = NavDriver(stream)
        # this configures the device by writing to registers
        self.get_logger().info('Configuring device')
        self.nav.setup()
        self.get_logger().info('Done configuring')
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i

        data = next(self.nav)

        # raise NotImplementedError('need to fix vectornav time conversion')
        timestamp = self.get_clock().now().to_msg()

        header = Header(
            frame_id='imu1_frame',
            stamp=timestamp,
            # seq=seq,
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
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def convert_to_quaternion(euler: tuple) -> Quaternion:
    """Convert euler angle orientation to quaternion

    Inputs
    * euler: 3-tuple of euler angle floats in YPR order

    Returns
    * Quaternion
    """
    quat = vn.convert_to_quaternion(euler)

    return Quaternion(
        x=quat[0],
        y=quat[1],
        z=quat[2],
        w=quat[3],
    )


def main(name = 'imu_node', port = '/dev/vectornav'):
    rclpy.init()

    if port is None:
        raise ValueError('port is not set.')
    
    imu_publisher = ImuPublisher(name, port)
    
    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = ArgumentParser(description='Parse VectorNav output and publish ROS messages')
    parser.add_argument('name', type=str)
    parser.add_argument('port', type=str, help='serial port of the VectorNav device')
    args = parser.parse_args()

    try:
        main(name=args.name, port=args.port)
    except rclpy.ROSInterruptException:
        pass
