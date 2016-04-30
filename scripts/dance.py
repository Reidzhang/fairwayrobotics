#!/usr/bin/env python
# Roomba script by Fairway robotic
import rospy

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from random import randint
import time

import sys, select, termios, tty

msg = """
Watch the turtlebot dance!
"""

class Dance:

    def __init__(self):
        self.twist = Twist()
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
        self.spinAround()
        self.forwardAndBackward()
        self.spinAround()

    def spinAround(self):
        for k in range(0,50):
            self.twist.linear.x = 0
            self.twist.angular.z = 1.25
            self.pub.publish(self.twist)
            time.sleep(0.1)

    def forwardAndBackward(self):
        for i in range(0,2):
            for k in range(0,15):
                self.twist.angular.z = 0
                self.twist.linear.x = 0.2
                self.pub.publish(self.twist)
                time.sleep(0.1)

            for k in range(0, 15):
                self.twist.angular.z = 0
                self.twist.linear.x = -0.2
                self.pub.publish(self.twist)
                time.sleep(0.1)

if __name__=="__main__":
    rospy.init_node('turtlebot_dance')
    obj = Dance()
    rospy.spin()