#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# Custom messages import
from rover_msgs.msg import Localization, Vectornav, Customgps

# Localization node that compiles gps and imu data into a single message
class LocalizationNode(Node):
    def __init__(self):
        super()._init_('localization_node')

        # Imu subscription and variables in node
        self.imu_message_count = 0
        self.yaw = 0.0
        self.vn_subscription = self.create_subscription(
            Vectornav,
            'imu',
            self.vn_callback,
            10)
        
        # Gps subscription and variables in node
        self.northing = None
        self.easting = None
        self.time_gps = None
        self.gps_subscription = self.create_subscription(
            Customgps,
            'gps',
            self.gps_callback,
            10)
        
        # Localization publish
        self.loc_publisher = self.create_publisher(
            Localization,
            'localization',
            10
        )

    # Imu (vectornav) callback to recieve messages and average running total in 1 second
    def vn_callback(self, msg):
        self.yaw += msg.imu.orientation.z
        self.imu_message_count += 1

        if self.imu_message_count == 40:
            self.yaw /= self.imu_message_count
            self.publish_localization_message()
            
    # Gps callback to recieve a gps message every second
    def gps_callback(self, msg):
        self.northing = msg.utm_northing
        self.easting = msg.utm_easting
        self.time_gps = msg.header.stamp
        self.publish_localization_message()

    # Compile all the messages from gps and imu average into a single message to publish
    def publish_localization_message(self):
        if self.northing is not None and self.easting is not None and self.time_gps is not None and self.imu_message_count == 40:
            # Collect messages from gps and imu callback to compile here
            localization_msg = Localization()
            localization_msg.time_gps = self.time_gps
            localization_msg.yaw = self.yaw
            localization_msg.northing = self.northing
            localization_msg.eating = self.easting
            self.loc_publisher(localization_msg)

            # Reset yaw running total and total imu message count to 0
            self.imu_message_count = 0
            self.yaw = 0.0

# Main loop
def main():
    # Initialize and run node
    rclpy.init()
    node = LocalizationNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()