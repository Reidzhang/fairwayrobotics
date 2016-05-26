#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import Float32

if __name__ == '__main__':
    pub = rospy.Publisher('weight', Float32, queue_size=10)
    rospy.init_node('weight_publisher', anonymous=True)
    rospy.loginfo("Initalize weight node")
    rate = rospy.Rate(10)
    with open('weight.txt', 'r', os.O_NONBLOCK) as f_d:
        while(1):
            content = f_d.readline()
            if len(content) > 0:
                weight = float(content)
                if weight < 6000:
                    print weight
                    pub.publish(weight)

   # keep robot from shutdown