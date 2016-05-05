#!/usr/bin/env python
'''
    Fariway robotics
    Dispencer node
'''

import json
import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped, Point, Vector3, PoseStamped
from std_msgs.msg import Header
# bumper messege
from kobuki_msgs.msg import BumperEvent
import tf

class Dispense:
    def __init__(self, goal):
        self.station = goal[0]
        # callback process sensor data
        self.bump_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.processBump)

    def processBump(self, msg):
        # send a command to arduino
        if msg.state == BumperEvent.PRESSED:
            # The bumper is pressed
            # check location
            mapPoint, mapRot = self.getCoords()
            if self.check_pose(mapPoint, mapRot):
                rospy.loginfo("Release a ball")

    def getCoords(self):
        mapPoint = None
        mapRot = None
        listener = tf.TransformListener()
        while True:
            try:
                listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(5.0))
                (mapPoint, mapRot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                break
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                continue
        return (mapPoint, mapRot)

    def check_pose(self, point, rot):
        goal_pos = self.goal.position
        goal_rot = self.goal.orientation
        if goal_pos.x == point.x and goal_pos.y == point.y and goal_pos.z == point.z \
                and goal_rot.x == rot.x and goal_rot.y == rot.y and goal_rot.z == rot.z and goal_rot.w == rot.w:
            return True
        return False

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