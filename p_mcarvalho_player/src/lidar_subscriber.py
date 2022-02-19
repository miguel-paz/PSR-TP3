#!/usr/bin/env python3
import math

import rospy
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2
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

#publisher = rospy.Publisher('/left_laser/point_cloud', PointCloud2)
pub = rospy.Publisher('/green1/green1/cmd_vel', Twist, queue_size=1)

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
    global target_angular_vel, target_linear_vel, control_angular_vel, control_linear_vel
    #rospy.loginfo('Received laser scan message')
    #rospy.loginfo(msg)

    #header = std_msgs.msg.Header(seq=msg.header.seq, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
    #fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #          PointField('y', 4, PointField.FLOAT32, 1),
    #          PointField('z', 8, PointField.FLOAT32, 1)]

    # convert from polar coordinates to cartesian and fill the point cloud
    points = []
    z = 0
    for idx, range in enumerate(msg.ranges):
        if range >= msg.range_min and range <= msg.range_max:
            theta = msg.angle_min + msg.angle_increment * idx
            x = range * math.cos(theta)
            y = range * math.sin(theta)
            points.append([x, y, z])
    
    center_dist=4
    if msg.ranges[0] < center_dist:
        center_dist = msg.ranges[0]
    
    right_dist=4
    if msg.ranges[int(1.5*math.pi/msg.angle_increment)] < right_dist:
        right_dist = msg.ranges[int(1.5*math.pi/msg.angle_increment)]
    
    left_dist=4
    if msg.ranges[int(1.5*math.pi/msg.angle_increment)] < left_dist:
        left_dist = msg.ranges[int(0.5*math.pi/msg.angle_increment)]
    
    if center_dist>3.5 and left_dist>1.5 and right_dist>1.5:
        twist = Twist()

        control_linear_vel = makeSimpleProfile(control_linear_vel, center_dist/5, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, 0, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)

    else:
        twist = Twist()

        control_linear_vel = makeSimpleProfile(control_linear_vel, 0, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, 1.5 if left_dist>right_dist else -1.5, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)
        

    #pc2 = point_cloud2.create_cloud(header, fields, points)  # create point_cloud2 data structure
    #publisher.publish(pc2)  # publish (will automatically convert from point_cloud2 to Pointcloud2 message)
    #rospy.loginfo('Published PointCloud2 msg')


def main():

    rospy.init_node('lidar_subscriber_green', anonymous=False)

    rospy.Subscriber('/green1/green1/scan', LaserScan, callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    
    main()