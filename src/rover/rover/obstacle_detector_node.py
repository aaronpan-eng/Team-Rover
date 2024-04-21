#!/usr/bin/env python3
from argparse import ArgumentParser

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from rover_msgs.msg import CircleObstacle, Obstacles

from rover.obstacle_detector import Scan


class ObstacleNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            self.qos_profile
        )
        self.publisher = self.create_publisher(
            Obstacles,
            'obstacles',
            self.qos_profile
        )
        self.get_logger().info('Listening for lidar scans!')

    def listener_callback(self, scan_msg):
        scan = Scan.from_ranges(
            angle_min=scan_msg.angle_min,
            angle_max=scan_msg.angle_max,
            angle_increment=scan_msg.angle_increment,
            time_increment=scan_msg.time_increment,
            scan_time=scan_msg.scan_time,
            range_min=scan_msg.range_min,
            range_max=scan_msg.range_max,
            ranges=scan_msg.ranges,
            intensities=scan_msg.intensities,
        )

        header = Header(
            frame_id='obstacles1_frame',
            stamp=self.get_clock().now().to_msg(),
        )

        out_msg = Obstacles(
            header=header,
            obstacles=[
                CircleObstacle(
                    center=Point(
                        x=o.forward,
                        y=-1.0 * o.left,
                        z=0.0,
                    ),
                    radius=o.radius,
                ) for o in scan.cluster()
            ]
        )
        self.get_logger().info(f'Found {len(out_msg.obstacles)} obstacles')
        self.publisher.publish(out_msg)


def main(name='obstacle_detector'):
    rclpy.init()

    node = ObstacleNode(name)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = ArgumentParser(description='Convert laser scans into lists of detected obstacles')
    parser.add_argument('name', type=str)
    args = parser.parse_args()

    try:
        main(name=args.name)
    except rclpy.ROSInterruptException:
        pass
