#!/usr/bin/env python3
import math
import random

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.2

turtlebot3_model = rospy.get_param("model", "burger")

target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0

Turning = False

pub = rospy.Publisher('/blue1/cmd_vel', Twist, queue_size=1)

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

def callbackMessageReceived(msg):
    global target_angular_vel, target_linear_vel, control_angular_vel, control_linear_vel, Turning
    #rospy.loginfo('Received laser scan message')
    #rospy.loginfo(msg)

    # convert from polar coordinates to cartesian and fill the point cloud
    points = []
    z = 0
    for idx, range in enumerate(msg.ranges):
        if range >= msg.range_min and range <= msg.range_max:
            theta = msg.angle_min + msg.angle_increment * idx
            x = range * math.cos(theta)
            y = range * math.sin(theta)
            points.append([x, y, z])
    
    center_dist=msg.range_max
    for range in msg.ranges[0:int(math.pi/24/msg.angle_increment)]+msg.ranges[int(47*math.pi/24/msg.angle_increment):]:
        if range >= msg.range_min and range <= msg.range_max and range < center_dist:
            center_dist = range
    
    right_dist=msg.range_max
    for range in msg.ranges[int(1.5*math.pi/msg.angle_increment):int(11*math.pi/6/msg.angle_increment)]:
        if range >= msg.range_min and range <= msg.range_max and range < right_dist:
            right_dist = range
    
    left_dist=msg.range_max
    for range in msg.ranges[int(math.pi/6/msg.angle_increment):int(0.5*math.pi/msg.angle_increment)]:
        if range >= msg.range_min and range <= msg.range_max and range < left_dist:
            left_dist = range
    
    if center_dist>2.5 and left_dist>0.3 and right_dist>0.3:
        Turning = False
        twist = Twist()

        control_linear_vel = makeSimpleProfile(control_linear_vel, 0.7, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, 0, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)

    else:
        
        twist = Twist()

        control_linear_vel = makeSimpleProfile(control_linear_vel, 0.2, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        
        if left_dist<2 or right_dist<2:
            if left_dist>right_dist:
                Turning=0.5
            else:
                Turning=-0.5
        
        elif not Turning:
            Turning = 0.5 if random.random()>0.5 else -0.5
            
        control_angular_vel = makeSimpleProfile(control_angular_vel, Turning, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)


def main():

    rospy.init_node('lidar_subscriber_blue', anonymous=False)

    rospy.Subscriber('/blue1/scan', LaserScan, callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()