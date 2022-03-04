#!/usr/bin/env python3
import math
import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from sklearn.cluster import DBSCAN
import numpy as np

turtlebot3_model = rospy.get_param("model", "burger")




BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.2

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
        self.pub = rospy.Publisher('/p_mcarvalho/cmd_vel', Twist, queue_size=1)


    def callbackTeleopReceived(self, msg):
        print(msg)
        if self.teleop_twist == None or (self.teleop_twist.angular.z == 0 and self.teleop_twist.linear.x == 0 and self.teleop_twist.linear.y == 0):
            if msg.angular.z == 0 and msg.linear.x == 0 and msg.linear.y == 0:
                return
        self.teleop_twist = msg
        self.pub.publish(msg)
        rospy.loginfo(msg)



def main():
    teleop_server = Teleop_Server()
    rospy.init_node('teleop_processing_node', anonymous=False)

    rospy.Subscriber('/remote_teleop', Twist, callback=teleop_server.callbackTeleopReceived)

    rospy.spin()


if __name__ == '__main__':
    main()