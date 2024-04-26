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
        self.avg_northing = 0.0
        self.avg_easting = 0.0
        self.avg_yaw = 0.0
        self.count = 0
        # TODO: set flag for gps coordinates to default to something later on

        # Subscribe to localization, obstacles, and LoRa topics
        self.localization_sub = self.create_subscription(
            Localization,
            'localization_topic',
            self.localization_callback,
            10
        )
        self.obstacles_sub = self.create_subscription(
            Obstacles,
            'obstacles_topic',
            self.obstacles_callback,
            10
        )
        self.lora_sub = self.create_subscription(
            LoRa,
            'lora_topic',
            self.lora_callback,
            10
        )

        # Publish path planning data
        self.path_planning_pub = self.create_publisher(
            PathPlanning, 'path_planning_topic', 10)

    def localization_callback(self, msg):
        # Average GPS northing, easting, and IMU yaw over 5 seconds
        self.avg_northing += msg.northing
        self.avg_easting += msg.easting
        self.avg_yaw += msg.yaw
        self.count += 1

        if self.count == 5:  # 5 seconds have passed
            self.avg_northing /= 5.0
            self.avg_easting /= 5.0
            self.avg_yaw /= 5.0

            # TODO: if self.goal flag is off, then set the GPS to a offset from current locatoin

            #calculate distance
            # TODO: substitute this angle calculation in
            phi = atan2(self.goal_easting - self.avg_easting, self.goal_northing - self.avg_northing)*(180/pi)
            if abs(self.avg_yaw) > (180-abs(phi)):
                if phi > 0:
                    goal_angle = (phi - self.avg_yaw) - 360
                elif phi < 0:
                    goal_angle = (phi - self.avg_yaw) + 360
            else:
                goal_angle = phi - self.avg_yaw


            # # Calculate distance to travel and angle to rotate
            # distance_to_travel = sqrt(
            #     (self.goal_northing - self.avg_northing)**2 + (self.goal_easting - self.avg_easting)**2)
            # deg_clockwise_to_rot = degrees(atan2(
            #     self.goal_northing - self.avg_northing, self.goal_easting - self.avg_easting))

            # Calculate distance to travel and angle to rotate
            distance_to_travel = sqrt(
                (self.goal_northing - self.avg_northing) ** 2 + (self.goal_easting - self.avg_easting) ** 2)
            
            # COMMENTED OUT #####
            # goal_angle = atan2(
            #     self.goal_northing - self.avg_northing, self.goal_easting - self.avg_easting)
            # # Ensure the angle is between 0 and 360 degrees
            # goal_angle = (goal_angle + 2 * pi) % (2 * pi)
            # # Calculate angle to rotate relative to current yaw angle from IMU
            # deg_clockwise_to_rot = degrees(goal_angle - self.avg_yaw)
            # # # Ensure the angle is between -180 and 180 degrees
            # # deg_clockwise_to_rot = (deg_clockwise_to_rot + 180) % 360 - 180
            # COMMENTED OUT #####


            # Publish path planning data
            path_planning_msg = PathPlanning()
            path_planning_msg.distance_to_travel = distance_to_travel
            path_planning_msg.deg_clockwise_to_rot = goal_angle # TODO: change message to now describe it as [-180,180], + being turn right, - being turn left
            self.path_planning_pub.publish(path_planning_msg)

            # Reset variables for next averaging period
            self.avg_northing = 0.0
            self.avg_easting = 0.0
            self.avg_yaw = 0.0
            self.count = 0

    def obstacles_callback(self, msg):
        # Process obstacles data if needed
        pass

    def lora_callback(self, msg):
        # Extract goal northing and easting from LoRa message
        self.goal_northing = msg.northing
        self.goal_easting = msg.easting


def main(args=None):
    rclpy.init(args=args)
    path_planning_node = PathPlanningNode()
    rclpy.spin(path_planning_node)
    path_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
