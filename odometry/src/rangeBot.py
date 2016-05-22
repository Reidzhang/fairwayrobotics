#!/usr/bin/env python
'''
    Fariway robotics
    Dispencer node
'''

import json
import rospy
import threading
from std_msgs.msg import Int8, Empty, Float32
from geometry_msgs.msg import Pose, Point, Quaternion
from sendRobot import NavTest
from dispense import Dispense
from actionlib import GoalStatus
import time


currentState = 'Dispenser'
weight = 'Empty'
lock = threading.Lock()

def main():
    rospy.init_node('rangeBot', anonymous=True)
    global currentState
    global weight
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

    nav = NavTest()

    rospy.Subscriber('/finished', Empty, finish, queue_size=1)
    # currentState = 'Station'
    while(1):
        if currentState == 'Dispenser':
            weightSubscriber = rospy.Subscriber('weight', Float32, callback=processWeight)
            while lock.acquire() and weight != 'Filled':
                lock.release()

            rospy.sleep(1)
            # nav.send_to_goal(station_pos)
            currentState = 'MovingToStation'
            weightSubscriber.unregister()
            print currentState

        elif currentState == 'Station':
            disp = Dispense()
            tmp_msg = rospy.wait_for_message('/request_balls', Empty)
            print 'tmp_msg'
            print tmp_msg
            # request more balls
        elif currentState == 'MovingToStation':
            # check our status
            if nav.move_base.get_state() == GoalStatus.SUCCEEDED:
                currentState = 'Station'
                pub = rospy.Publisher('at_home', Empty, queue_size=1)
                rate = rospy.Rate(10)
                time.sleep(5)
                pub.publish(Empty())
            elif nav.move_base.get_state() == GoalStatus.ABORTED:
                # resent goal if abort
                nav.move_base.cancel_goal()
                nav.send_to_goal(station_pos)
            else:
                continue
        elif currentState == 'MovingToDispenser':
            pass
            # check our status
            if nav.move_base.get_state() == GoalStatus.SUCCEEDED:
                currentState = 'Dispenser'
                weight = 'Empty'
            elif nav.move_base.get_state() == GoalStatus.ABORTED:
                nav.move_base.cancel_goal()
                nav.send_to_goal(dispenser_pos)
            else:
                continue
        else:
            # finish
            break

def finish(msg):
    global currentState
    currentState = 'Finish'

def processWeight(msg):
    global weight
    if msg.data >= 16.2:
        print 'Basket filled'
        lock.acquire()
        weight = 'Filled'
        lock.release()


if __name__ == '__main__':
    main()