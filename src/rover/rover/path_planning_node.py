import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rover_msgs.msg import Localization, Obstacles, LoRa, PathPlanning
from math import atan2, degrees, sqrt, pi
from functools import partial


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # Initialize variables for averaging GPS and IMU data
        self.northing = 0.0
        self.easting = 0.0
        self.yaw = 0.0
        # self.count = 0

        # Flag for setting default value or lora value of gps coordinates
        self.gps_flag = 0

        # Subscribe to localization, obstacles, and LoRa topics
        self.localization_sub = self.create_subscription(
            Localization,
            'localization',
            self.localization_callback,
            10
        )
        self.obstacles_sub = self.create_subscription(
            Obstacles,
            'obstacles',
            self.obstacles_callback,
            10
        )
        self.lora_sub = self.create_subscription(
            LoRa,
            'lora',
            self.lora_callback,
            10
        )

        # Publish path planning data
        self.path_planning_pub = self.create_publisher(
            PathPlanning, 'path_planning', 10)

    def localization_callback(self, msg):
        
        # Setting to default value of northing and easting if not set by lora
        if self.gps_flag == 0:
            self.goal_northing = msg.northing - 138
            self.goal_easting = msg.easting - 87
            self.gps_flag = 1

        # GPS northing, easting, and IMU yaw
        self.northing = msg.northing
        self.easting = msg.easting
        self.yaw = msg.yaw
        
        # calculate angle to rotate
        phi = atan2(self.goal_easting - self.easting, self.goal_northing - self.northing)
        phi = degrees(phi)
        print('yaw: ', self.yaw)
        print('phi: ', phi)
        
        # goal angle is angle that the rover should turn + is right, - is left
        goal_angle = phi - self.yaw

        if abs(goal_angle) > 180:
            if phi < 0:
                goal_angle = -(abs(phi)+abs(self.yaw)) + 360
            if phi > 0:
                goal_angle = (abs(phi)+abs(self.yaw)) - 360
        
        # Calculate distance to travel
        distance_to_travel = sqrt(
                (self.goal_northing - self.northing) ** 2 + (self.goal_easting - self.easting) ** 2)

        # Publish message
        
        path_planning_msg = PathPlanning()
        path_planning_msg.header.frame_id = 'path_planning1_frame'
        path_planning_msg.header.stamp = msg.header.stamp
        path_planning_msg.distance_to_travel = distance_to_travel
        path_planning_msg.deg_to_rot = goal_angle # [-180,180], + being turn right, - being turn left
        self.path_planning_pub.publish(path_planning_msg)
        self.get_logger().info('Publishing: "%s"' % path_planning_msg)

    def obstacles_callback(self, msg):
        # Process obstacles data if needed
        pass

    def lora_callback(self, msg):
        # Extract goal northing and easting from LoRa message
        self.goal_northing = msg.northing
        self.goal_easting = msg.easting
        self.gps_flag = 1


def main():
    rclpy.init()
    path_planning_node = PathPlanningNode()
    rclpy.spin(path_planning_node)
    path_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
