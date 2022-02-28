#!/usr/bin/env python3
import math
import random

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from sklearn.cluster import DBSCAN
import numpy as np

turtlebot3_model = rospy.get_param("model", "burger")



pub = rospy.Publisher('/p_mcarvalho/cmd_vel', Twist, queue_size=1)

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

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output


    def callbackTeleopReceived(self, msg):
        self.teleop_twist = msg
        pub.publish(msg)
        rospy.loginfo(msg)


    def callbackMessageReceived(self, msg):
        #rospy.loginfo('Received laser scan message')
        rospy.loginfo(msg)

        # convert from polar coordinates to cartesian and fill the point cloud
        points = []
        z = 0
        for idx, r in enumerate(msg.ranges):
            if r >= msg.range_min and r <= msg.range_max:
                theta = msg.angle_min + msg.angle_increment * idx
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                points.append([x, y, z])
        first_p = points[0]
        
        same = False

        if first_p=="inf":
            for p in range(1,len(points)):
                if points[p] != first_p:
                    same = True
                    break
            if same:
                pub.publish(self.teleop_twist)
                return


        points = np.array(points)
        
        clustering = DBSCAN(eps=0.5, min_samples=1).fit(points)
        
        #print(clustering.labels_)
        
        robot_labels = []
        for label in set(clustering.labels_):
            if 2<= len([l for l in clustering.labels_ if l==label]) <=20:
                robot_labels.append(label)

        center_dist=msg.range_max
        for r in msg.ranges[0:int(math.pi/24/msg.angle_increment)]+msg.ranges[int(47*math.pi/24/msg.angle_increment):]:
            if r >= msg.range_min and r <= msg.range_max and r < center_dist:
                center_dist = r
        
        right_dist=msg.range_max
        for r in msg.ranges[int(1.5*math.pi/msg.angle_increment):int(11*math.pi/6/msg.angle_increment)]:
            if r >= msg.range_min and r <= msg.range_max and r < right_dist:
                right_dist = r
        
        left_dist=msg.range_max
        for r in msg.ranges[int(math.pi/6/msg.angle_increment):int(0.5*math.pi/msg.angle_increment)]:
            if r >= msg.range_min and r <= msg.range_max and r < left_dist:
                left_dist = r
        
        if center_dist>2.5 and left_dist>0.3 and right_dist>0.3 and self.teleop_twist and -0.3 < self.teleop_twist.angular.z < 0.3:
            self.Turning = False
            pub.publish(self.teleop_twist)
            # self.control_linear_vel = makeSimpleProfile(control_linear_vel, 0.7, (LIN_VEL_STEP_SIZE/2.0))
            # self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, 0, (ANG_VEL_STEP_SIZE/2.0))
            return
        
        self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, 0.2, (LIN_VEL_STEP_SIZE/2.0))
        
        if left_dist<2 or right_dist<2:
            if left_dist>right_dist:
                self.Turning=0.5
            else:
                self.Turning=-0.5
        
        elif not self.Turning:
            self.Turning = 0.5 if random.random()>0.5 else -0.5
            
        self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.Turning, (ANG_VEL_STEP_SIZE/2.0))

        twist = Twist()
        twist.linear.x = self.control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.control_angular_vel
        pub.publish(twist)


def main():
    teleop_server = Teleop_Server()
    rospy.init_node('teleop_processing_node', anonymous=False)

    rospy.Subscriber('/teleop_vel', Twist, callback=teleop_server.callbackTeleopReceived)
    #rospy.Subscriber('/scan', LaserScan, callback=teleop_server.callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()