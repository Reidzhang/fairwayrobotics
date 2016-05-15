#!/usr/bin/env python
'''
    Fariway robotics
    Dispencer node
'''

import json
import rospy
from std_msgs.msg import Int8, Empty, String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from sendRobot import NavTest
from dispense import Dispense
import time
import tf

currentState = 'Dispenser'

def main():
    rospy.init_node('rangeBot', anonymous=True)
    global currentState
    with open('pose-map.json', 'r') as dFile:
        data = json.loads(dFile.read())
    # now we have the data
    # we can print out the name
    print "You have the following known locations"
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
    print 'waiting for mesg'
    msg = rospy.wait_for_message('/choose_station', Int8)

    print 'station num'
    print msg

    station_pos = locations[msg.data]
    dispenser_pos = locations[0]

    # TO DO:
    # 1. Send the robot to the station
    # 2. Need to integrate gum ball machine
    # 3. Integrate the scale
    # Another two types of messages: '/finish', '/request_balls'
    # states = ['Dispenser', 'Station', 'MovingToStation', 'MovingToDispenser', 'Empty', 'Filled', 'Finish']
    weight = 'Filled'

    nav = NavTest()

    rospy.Subscriber('/finish', Empty, finish, queue_size=1)

    currentState = 'Station'

    while(1):
        if currentState == 'Dispenser':
            while weight != 'Filled':
                # TO-DO: check the scale
                pass
            nav.send_to_goal(station_pos)
            currentState = 'MovingToStation'

        elif currentState == 'Station':
            disp = Dispense()
            tmp_msg = rospy.wait_for_message('/request_balls', Empty)
            print 'tmp_msg'
            print tmp_msg
            # request more balls
        elif currentState == 'MovingToStation':
            # check our status
            pass
        elif currentState == 'MovingToDispenser':
            # check our status
            pass
        else:
            # finish
            break

def finish(msg):
    global currentState
    currentState = 'Finish'


if __name__ == '__main__':
    main()