#!/usr/bin/env python3
import math
import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from sklearn.cluster import DBSCAN
import numpy as np

turtlebot3_model = rospy.get_param("model", "burger")


class Teleop_Server:
    def __init__(self):
        self.teleop_twist = None
        self.laser_data = None
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0
        self.Turning = False
        self.pubR = rospy.Publisher('/red1/teleop_vel', Twist, queue_size=1)
        self.pubG = rospy.Publisher('/green1/teleop_vel', Twist, queue_size=1)
        self.pubB = rospy.Publisher('/blue1/teleop_vel', Twist, queue_size=1)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input
        return output

    def callbackTeleopReceivedR(self, msg):
        print(msg)
        if self.teleop_twist == None or (self.teleop_twist.angular.z == 0 and self.teleop_twist.linear.x == 0 and self.teleop_twist.linear.y == 0):
            if msg.angular.z == 0 and msg.linear.x == 0 and msg.linear.y == 0:
                return
        self.teleop_twist = msg
        self.pubR.publish(msg)
        rospy.loginfo(msg)

    def callbackTeleopReceivedG(self, msg):
        print(msg)
        if self.teleop_twist == None or (self.teleop_twist.angular.z == 0 and self.teleop_twist.linear.x == 0 and self.teleop_twist.linear.y == 0):
            if msg.angular.z == 0 and msg.linear.x == 0 and msg.linear.y == 0:
                return
        self.teleop_twist = msg
        self.pubG.publish(msg)
        rospy.loginfo(msg)

    def callbackTeleopReceivedB(self, msg):
        print(msg)
        if self.teleop_twist == None or (self.teleop_twist.angular.z == 0 and self.teleop_twist.linear.x == 0 and self.teleop_twist.linear.y == 0):
            if msg.angular.z == 0 and msg.linear.x == 0 and msg.linear.y == 0:
                return
        self.teleop_twist = msg
        self.pubB.publish(msg)
        rospy.loginfo(msg)


def main():
    teleop_server = Teleop_Server()
    rospy.init_node('teleop_processing_node', anonymous=False)

    rospy.Subscriber('/red1/remote_teleop', Twist, callback=teleop_server.callbackTeleopReceivedR)
    rospy.Subscriber('/green1/remote_teleop', Twist, callback=teleop_server.callbackTeleopReceivedG)
    rospy.Subscriber('/blue1/remote_teleop', Twist, callback=teleop_server.callbackTeleopReceivedB)
    rospy.spin()


if __name__ == '__main__':
    main()