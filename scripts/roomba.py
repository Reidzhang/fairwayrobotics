#!/usr/bin/env python
# Roomba script by Fairway robotic
import rospy

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from random import randint
import time

import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

class Roomba:

    def __init__(self):
	self.twist = Twist()
        self.bump_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.processBump)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
	self.lock = False
        self.moveForward()

    def processBump(self, msg):
        if msg.state == BumperEvent.PRESSED:
	    self.lock = True
	    for k in range(0, 10):
	        self.twist.angular.z = 0
                self.twist.linear.x = -0.1
                self.pub.publish(self.twist)
                time.sleep(0.1)

	    sign = randint(-1,1)
	    
            for k in range(0, 25):
		self.twist.angular.z = sign * 0.75
                self.twist.linear.x = 0
                self.pub.publish(self.twist)
		time.sleep(0.1)
	    
	    self.lock = False
            

    def moveForward(self):
        while(1):
            if (not self.lock):
	    	self.twist.angular.z = 0
            	self.twist.linear.x = 0.1
            	self.pub.publish(self.twist)

if __name__=="__main__": 
    rospy.init_node('turtlebot_roomba')
    obj = Roomba()
    rospy.spin()
    


