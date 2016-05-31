#!/usr/bin/env python
'''
    Fariway robotics
    Dispencer node
'''

import json
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
import time
import tf

class Dispense:
    def __init__(self):
        # callback process sensor data
        self.bump_sub = rospy.Subscriber('/urg_scan', LaserScan, self.processSensor, queue_size=1)
        # connection to laser sensor
        self.gum_pub = rospy.Publisher('/dispense_ball', Empty, queue_size=1)


    def processSensor(self, msg):
        # send a command to arduino
        # The bumper is pressed
        # check location
        for i in range(0, 100):
            if msg.ranges[490 + i] < 0.3 and msg.ranges[490 + i] > msg.range_min:
                # print "Release a ball"
                self.gum_pub.publish(Empty())
                rospy.sleep(2.0)
                break

if __name__ == '__main__':
    rospy.init_node('ball_dispence', anonymous=True)
    disp = Dispense()
    # keep robot from shutdown
    rospy.spin()
