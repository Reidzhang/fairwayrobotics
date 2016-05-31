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
wLock = threading.Lock()


def main():
    rospy.init_node('rangeBot', anonymous=True)
    global currentState
    global weight

    print 'waiting for mesg'
    msg = rospy.wait_for_message('/choose_station', Int8)

    print 'station num'
    print msg

    # process the location
    locations = processLocations()

    station_pos = locations[msg.data + 1]
    home = locations[0]
    dispenser_pos = locations[1]

    # TO DO:
    # 1. Send the robot to the station
    # 2. Need to integrate gum ball machine
    # 3. Integrate the scale
    # Another two types of messages: '/finish', '/request_balls'
    # states = ['Dispenser', 'Station', 'MovingToStation', 'MovingToDispenser', 'Empty', 'Filled', 'Finish']

    nav = NavTest()

    currentState = 'MovingToStation'
    nav.send_to_goal(station_pos)

    while (1):
        print 'Current state = ' + currentState
        if currentState == 'Dispenser':
            weightSubscriber = rospy.Subscriber('weight', Float32, callback=processWeight)
            while wLock.acquire() and weight != 'Filled':
                wLock.release()

            rospy.sleep(1)
            # nav.send_to_goal(station_pos)
            currentState = 'MovingToStation'
            weightSubscriber.unregister()
            print currentState

        elif currentState == 'Station':
            # request more balls
            disp = Dispense()
            req_sub = rospy.Subscriber('/request_balls', Empty, callback=refill_balls)
            finish_sub = rospy.Subscriber('/finished', Empty, finish, queue_size=1)
            # tmp_msg = rospy.wait_for_message('/request_balls', Empty)

            while 1:
                if currentState != 'Station':
                    break
            disp.bump_sub.unregister()
            req_sub.unregister()
            finish_sub.unregister()
        elif currentState == 'MovingToStation':
            # check our status
            if nav.move_base.get_state() == GoalStatus.SUCCEEDED:
                currentState = 'Station'
                pub = rospy.Publisher('at_station', Empty, queue_size=1)
                rospy.Rate(5).sleep()
                # time.sleep(5)
                pub.publish(Empty())
            elif nav.move_base.get_state() == GoalStatus.ABORTED:
                res = recoveryPlan(nav, station_pos, home)
                if res:
                    # recovery success
                    currentState = 'Station'
                else:
                    currentState = 'Finish'
            else:
                continue
        elif currentState == 'MovingToDispenser':
            # check our status
            if nav.move_base.get_state() == GoalStatus.SUCCEEDED:
                currentState = 'Dispenser'
                weight = 'Empty'
            elif nav.move_base.get_state() == GoalStatus.ABORTED:
                res = recoveryPlan(nav, dispenser_pos, home)
                if res:
                    # recovery success
                    currentState = 'Dispenser'
                else:
                    currentState = 'Finish'
            else:
                continue
        else:
            # finish
            break


'''Recovery behavior'''
def recoveryPlan(navg, dist, home):
    # =================== First phase ==========================

    print 'First Phase'
    if check_recovery(navg, dist):
        print 'First Phase success ! '
        return True

    # ===================== Second phase ========================
    alert_pub = rospy.Publisher('please_move', Empty, queue_size=1)
    rospy.Rate(5).sleep()
    alert_pub.publish(Empty())
    print 'Second Phase'
    rospy.sleep(5)
    pleasemovesuccess_pub = rospy.Publisher('please_move_success', Empty, queue_size=1)
    rospy.Rate(5).sleep()
    pleasemovesuccess_pub.publish(Empty())
    if check_recovery(navg, dist):
        print 'Second Phase success !'
        return True

    # ===================== Third phase =========================

    print 'Third Phase'
    navg.move_base.cancel_all_goals()
    alert_pub = rospy.Publisher('mission_abort', Empty, queue_size=1)
    rospy.Rate(5).sleep()
    alert_pub.publish(Empty())
    # send it home
    navg.send_to_goal(home)
    return False

'''Helper function: check the recovery progress'''
def check_recovery(navg, dist):
    # clear the map and re-send to dist
    navg.move_base.cancel_all_goals()
    navg.clear_costmaps()
    # Wait 1 second
    rospy.sleep(1.0)
    navg.send_to_goal(dist)
    while navg.move_base.get_state() != GoalStatus.ABORTED:
        if navg.move_base.get_state() == GoalStatus.SUCCEEDED:
            # achieve the goal
            return True
        elif navg.move_base.get_state() == GoalStatus.LOST:
            navg.move_base.cancel_goal()
            navg.send_to_goal(dist)
    return False

'''Helper function: process the location from json file'''
def processLocations():
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

    return locations

def refill_balls(msg):
    global currentState
    currentState = 'MovingToDispenser'


def finish(msg):
    global currentState
    currentState = 'Finish'


def processWeight(msg):
    global weight
    if msg.data >= 16.2:
        print 'Basket filled'
        wLock.acquire()
        weight = 'Filled'
        wLock.release()


if __name__ == '__main__':
    main()
