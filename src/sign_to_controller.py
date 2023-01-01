#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2 as cv

class GestureController:

    def __init__(self):
        rospy.init_node('sign_to_controller', anonymous=True)

        self.gesture_subcriber = rospy.Subscriber(rospy.get_param("hand_sign_recognition/publish_gesture_topic"), 
                                                String, self.callback)
        # Publisher which will publish to the topic 
        self.vel_publisher = rospy.Publisher("/mobile_robot/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()

    def callback(self, gesture):
        print(gesture.data)
        if gesture.data ==  "Forward":
            self.vel_msg.linear.x += 0.01
        elif gesture.data ==  "Backward":
            self.vel_msg.linear.x -= 0.01
        elif gesture.data == "Turn Right":
            self.vel_msg.angular.z += 0.1
        elif gesture.data == "Turn Left":
            self.vel_msg.angular.z -= 0.1
        elif gesture.data == "Stop":
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0

        if self.vel_msg.linear.x >= 3. :
            self.vel_msg.linear.x = 3.
        if self.vel_msg.angular.z >= 3. :
            self.vel_msg.angular.z = 3

        # self.vel_msg.linear.y = 0
        # self.vel_msg.linear.z = 0
        # self.vel_msg.angular.x = 0
        # self.vel_msg.angular.y = 0
        
        self.vel_publisher.publish(self.vel_msg)
        

if __name__=="__main__":
    try:
        ges2control = GestureController()
        rospy.spin()
    # If we press control + C, the node will stop.
    except rospy.ROSInternalException:
        cv.destroyAllWindows()
        pass
