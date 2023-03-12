#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2 as cv

class GestureController:

    def __init__(self):
        rospy.init_node('hand_sign_control', anonymous=True)
        # Subscriber for subscribing the hand signs
        self.gesture_subcriber = rospy.Subscriber(rospy.get_param("hand_sign_recognition/publish_gesture_topic"), 
                                                String, self.callback)
        # Publisher for publishing velocities 
        self.vel_publisher = rospy.Publisher("/robot_diff_drive_controller/cmd_vel", Twist, queue_size=10)
        # Velocity message
        self.vel_msg = Twist()
        # Velocity increments
        self.linear_vel = 0.01 #[m/s]
        self.angular_vel = 0.1 #[rad/s]

    def callback(self, gesture):
        """Convert the current hand sign to a velocity command.
        The command is put to self.vel_msg which is published to topic "/robot_diff_drive_controller/cmd_vel".

        Args:
            gesture (string): Forward, Backward, Turn Right, Turn Left, Stop, NONE
            Linear velocity is m/s. Angular velocity is rad/s.
        """
        if gesture.data ==  "Forward":
            self.vel_msg.linear.x += self.linear_vel
            self.vel_msg.angular.z = 0.
        elif gesture.data ==  "Backward":
            self.vel_msg.linear.x -= self.linear_vel
            self.vel_msg.angular.z = 0.
        elif gesture.data == "Turn Right":
            self.vel_msg.angular.z -= self.angular_vel
        elif gesture.data == "Turn Left":
            self.vel_msg.angular.z += self.angular_vel
        elif gesture.data == "Stop" or gesture.data == "NONE" :
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
        
        self.vel_publisher.publish(self.vel_msg)
        print(gesture.data, " Linear: {0} m/s, Angular: {1} rad/s".format(round(self.vel_msg.linear.x,6), round(self.vel_msg.angular.z,6)))
        

if __name__=="__main__":
    try:
        ges2control = GestureController()
        rospy.spin()
    # If we press control + C, the node will stop.
    except rospy.ROSInternalException:
        cv.destroyAllWindows()
        pass
