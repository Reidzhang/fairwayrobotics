#!/usr/bin/env python
'''
    Fariway robotics
    Dispencer node
'''

import json
import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped, Point, Vector3, PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from math import sqrt
import time
import tf

class Dispense:
    def __init__(self, goal):
        self.station = goal[0]
        # callback process sensor data
        self.bump_sub = rospy.Subscriber('/urg_scan', LaserScan, self.processSensor, queue_size=1)


    def processSensor(self, msg):
        # send a command to arduino
        # The bumper is pressed
        # check location
        for i in range(0, 100):
            if msg.ranges[490 + i] < 0.3 and msg.ranges[490 + i] > msg.range_min:
                print "Release a ball"
                rospy.sleep(2.0)
                break

    def getCoords(self):
        mapPoint = None
        mapRot = None
        listener = tf.TransformListener()
        while True:
            try:
                # look for the location 3 secondes ago
                now = rospy.Time.now() - rospy.Duration(2.0)
                listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
                (mapPoint, mapRot) = listener.lookupTransform("/map", "/base_link", now)
                break
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                continue
        return (mapPoint, mapRot)

    def check_pose(self, point, rot):
        goal_pos = self.station.position
        goal_rot = self.station.orientation
        dis = self.cal_dist(goal_pos, point)
        if dis < 0.1:
            return True

        return False

    def cal_dist(self, goal, point):
        a = (goal.x, goal.y, goal.z)
        b = (point[0], point[1], point[2])
        return sqrt(sum( (a - b) ** 2 for a, b in zip(a, b)))

if __name__ == '__main__':
    rospy.init_node('ball_dispence', anonymous=True)

    rospy.loginfo("ball_dispence node initiated")

    with open('pose-map.json', 'r') as dFile:
        data = json.loads(dFile.read())
    # now we have the data
    # we can print out the name
    rospy.loginfo("You have the following known locations")

    locations = dict()
    i = 0
    for key, val in data.items():
        # populate the location Dict
        if 'r' in val.keys():
            locations[i] = [Pose(Point(val['x'], val['y'], val['z']),
                                 Quaternion(val['quatX'], val['quatY'], val['quatZ'], val['quatW'])), val['r']]
        else:
            locations[i] = [Pose(Point(val['x'], val['y'], val['z']),
                                 Quaternion(val['quatX'], val['quatY'], val['quatZ'], val['quatW'])), None]
        i += 1
    goal = None
    index = raw_input("Enter an index for station lication: ")
    goal = locations[int(index)]
    rospy.loginfo("got the station location")
    disp = Dispense(goal)
    # keep robot from shutdown
    rospy.spin()