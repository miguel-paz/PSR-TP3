#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from p_mcarvalho_bringup.msg import Coach_msg
import cv2
from cv_bridge import CvBridge
import sqlite3

class Coach:

    def __init__(self):
        self.name = rospy.get_name()
        self.name = self.name.strip('/') # remove initial /

        print('Initializing Coach node with name: ' + self.name)

        self.team_color = self.name.split('_', 1)[0]

        self.team = {}

        self.bridge = CvBridge()

        try:
            sqliteConnection = sqlite3.connect('TP3_Django/psr_tp3_test/db.sqlite3')
            cursor = sqliteConnection.cursor()
            print("Successfully Connected to Django DB")

            cursor.execute("INSERT INTO TeamHunt_coach(name) VALUES (?)", (self.name,))
            
            sqliteConnection.commit()
            cursor.close()

        except sqlite3.Error as error:
            print("Error while connecting to Django DB", error)
        finally:
            if sqliteConnection:
                sqliteConnection.close()
                print("The Django DB connection is closed")

        topic = self.name + "_comm"
        print('Subscribing to topic ' + topic)
        rospy.Subscriber(topic, Coach_msg, self.callbackmsg)

    def callbackmsg(self, data):
        content = data.content.split('/')
        data_previous_state = content[0]
        data_current_state = content[1]
        data_reason = content[2]
        new_rob = False
        valid_data = False

        
        # if data_type=='Init':
        #     if self.team_color in data_sender:
        #         self.team[data_sender] = ''
        #         print('Robot ' + data_sender + ' added to team of coach ' + self.name)
        #     else:
        #         print('Robot ' + data_sender + ' cannot be added to team of coach ' + self.name)
        if data.type=='State_Update':
            if self.team_color not in data.sender:
                print('Robot ' + data.sender + ' cannot be added to team of coach ' + self.name)
                return
            if data.sender not in self.team:
                print('Robot ' + data.sender + ' added to team of coach ' + self.name)
                new_rob = True
            self.team[data.sender] = {'previous_state': data_previous_state, 'current_state': data_current_state, 'reason': data_reason} #, 'front_camera': data.front_camera 'back_camera': data.back_camera}
            print('Robot ' + data.sender + ' is changing from state ' + data_previous_state + ' to state ' + data_current_state + ' because it ' + data_reason)
            valid_data = True
        else:
            print('Unrecognized message type by robot ' + data.sender)
            return

        print(self.team)

        if valid_data:
            try:
                sqliteConnection = sqlite3.connect('TP3_Django/psr_tp3_test/db.sqlite3')
                cursor = sqliteConnection.cursor()
                print("Successfully Connected to Django DB")

                if new_rob:
                    cursor.execute('''INSERT INTO TeamHunt_robot(name,state,coach_id) VALUES (?,?,?)''', (data.sender,data_current_state,self.name))
                
                cursor.execute('''INSERT INTO TeamHunt_message(sender_id,previous_state,current_state,reason,receiver_id) VALUES (?,?,?,?,?)''', (data.sender, data_previous_state, data_current_state, data_reason, self.name))
                sqliteConnection.commit()
                cursor.close()

            except sqlite3.Error as error:
                print("Error while connecting to Django DB", error)
            finally:
                if sqliteConnection:
                    sqliteConnection.close()
                    print("The Django DB connection is closed")


        try:
            cv_image_front = self.bridge.imgmsg_to_cv2(data.front_camera, "bgr8")
            cv_image_back = self.bridge.imgmsg_to_cv2(data.back_camera, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imshow(data.sender + ' Front Camera', cv_image_front)
        cv2.imshow(data.sender + ' Back Camera', cv_image_back)
        #cv2.imwrite('testcoachcam.jpg',cv_image_front)
        cv2.waitKey(3)
        

def main():
    # ---------------------------------------------------
    # INITIALIZATION
    # ---------------------------------------------------
    default_node_name = 'coach'
    rospy.init_node(default_node_name, anonymous=False)

    coach = Coach()
    rospy.spin()
    

if __name__ == '__main__':
    main()