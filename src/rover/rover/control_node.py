#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# Custom messages import
from rover_msgs.msg import PathPlanning

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.create_subscription(
            PathPlanning,
            'path_planning_topic',
            self.path_planning_callback,
            10
        )
        # Steering thresholds
        self.steer_max = 45

        # PID tuning parameters
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 2.0

        # Pid parameters
        self.setpoint = 0
        self.error = 0
        self.integral_err = 0
        self.prev_error = 0
        self.derivative_err = 0
        self.output = 0

        self.create_publisher(
            MotorOutput,
            'motor',
            10
        )

    def path_planning_callback(self, msg):
        self.error = msg.deg_clockwise_to_rotate




