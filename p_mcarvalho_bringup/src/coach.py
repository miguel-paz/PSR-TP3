#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class Coach:

    def __init__(self):
        rospy.Subscriber("coach_comm", String, self.callback)

    def callback(self, data):
        print(data)
        

def main():
    # ---------------------------------------------------
    # INITIALIZATION
    # ---------------------------------------------------
    default_node_name = 'coach'
    rospy.init_node(default_node_name, anonymous=False)

    print('Initializing Coach node with name: ' + default_node_name)
    coach = Coach()
    rospy.spin()
    

if __name__ == '__main__':
    main()