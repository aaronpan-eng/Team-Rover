#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# For motor control
from gpiozero import AngularServo
from time import sleep
import RPi.GPIO as gpio
import time

# Custom messages import
from rover_msgs.msg import PathPlanning

# Global variables
MAX_STEERING = 50
KP = 1
KI = 0.001
KD = 0.001
STOP_RADIUS = 75 # meters
# Start to finish is about 165 m, so we are starting big at about 75 m stop radius 
# and narrowing it down

TIME_STEP = 1 # set to constant for testing, should update based on gps or rclpy timestamp

# RPi GPIO setup
gpio.setmode(gpio.BCM)  # Use Broadcom (BCM), not BOARD numbering which is the small numbers in pin-out
gpio.setup(2, gpio.OUT)
gpio.setup(3, gpio.OUT)
gpio.setup(4, gpio.OUT)
gpio.setup(17, gpio.OUT)
servo = AngularServo(17, min_pulse_width=0.0006, max_pulse_width=0.0023) #these are standard values, I don't think it matters all that much

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Subscriber stuff
        self.distance_to_goal = 0
        self.motor_on = 0

        self.create_subscription(
            PathPlanning,
            'path_planning_topic',
            self.path_planning_callback,
            10
        )

        # PID tuning parameters
        self.kp = KP
        self.ki = KI
        self.kd = KD

        # Pid parameters
        self.setpoint = 0
        self.error = 0
        self.integral_err = 0
        self.prev_error = 0
        self.derivative_err = 0
        self.output = 0
        self.prev_time = None

    def path_planning_callback(self, msg):
        # Initialize previous time from gps and then start control after
        if self.prev_time == None:
            sec = msg.header.stamp.sec
            nsec = msg.header.stamp.nanosec
            current_time = sec + (nsec/1e9)
            self.prev_time = current_time
        # Start control once the previous time from the gps is initializes
        else:
            sec = msg.header.stamp.sec
            nsec = msg.header.stamp.nanosec
            current_time = sec + (nsec/1e9)
            time_step = current_time - self.prev_time
            self.prev_time = current_time

            # PID Control
            self.error = msg.deg_to_rotate
            self.integral_err = self.error + time_step
            self.derivative_err = (self.error - self.prev_error) / time_step
            self.prev_error = self.error
            self.output = self.kp*self.error

            # Setting max steering angle
            if self.output >= MAX_STEERING:
                self.output = MAX_STEERING
            elif self.output <= -MAX_STEERING:
                self.output = -MAX_STEERING
            
            # Distance to goal
            self.distance_to_goal = msg.distance_to_travel
            
            # If distance to goal is not within stop radius, keep going
            # If inside radius, stop
            if self.distance_to_goal <= STOP_RADIUS:
                # Servo angle
                servo.angle = 0

                # Turn motor on
                self.motor_on = 0 
                self.set_motor()
            else:
                # Servo angle
                servo.angle = self.output

                # Turn motor on
                # self.motor_on = 1 # COMMENT OUT FOR TESTING
                self.set_motor()
            

    def set_motor(self):
        if self.motor_on == 1:
            # motor to fwd low (val_10=5)
            gpio.output(2, gpio.HIGH) # "two to the second"
            gpio.output(3, gpio.LOW) # "two to the first"
            gpio.output(4, gpio.HIGH) # "two to the zero"
            print("pin2(2^2)=1, pin3(2^1)=0, pin4(2^0)=1") # TODO: maybe need to add sleep for 1 second
        else:
            # motor to neutral (val_10=4)
            gpio.output(2, gpio.HIGH) # "two to the second"
            gpio.output(3, gpio.LOW) # "two to the first"
            gpio.output(4, gpio.LOW) # "two to the zero"
            print("pin2(2^2)=1, pin3(2^1)=0, pin4(2^0)=0")


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # After shutdown set pins to high to stop rover just in case
    gpio.output(2, gpio.HIGH) # "two to the second"
    gpio.output(3, gpio.HIGH) # "two to the first"
    gpio.output(4, gpio.HIGH) # "two to the zero"

if __name__ == '__main__':
    main()

