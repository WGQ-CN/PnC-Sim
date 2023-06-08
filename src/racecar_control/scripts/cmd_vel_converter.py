#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray,Twist
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import tf


class cmd_vel_converter:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.callback_cmd_vel, queue_size=1)
        self.navigation_input = rospy.Publisher('/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=1)
        self.Low_Speed_Mode = False
        self.ackermann_control = AckermannDriveStamped()
        self.ackermann_control.drive.speed = 0.0
        self.ackermann_control.drive.steering_angle = 0.0
        self.ackermann_control.drive.steering_angle_velocity = 0.0

    def callback_cmd_vel(self, data):
        self.ackermann_control.drive.speed = data.linear.x
        # self.ackermann_control.drive.steering_angle = data.angular.z
        self.ackermann_control.drive.steering_angle = math.atan2(data.angular.z*0.335,data.linear.x)
        self.navigation_input.publish(self.ackermann_control)

if __name__ == "__main__":
    rospy.init_node("cmd_vel_converter")
    cmd_vel_converter()
    rospy.spin()

