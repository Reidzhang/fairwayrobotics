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

currentState = 'Home'
# weight = 'Empty'
# wLock = threading.Lock()


def main():
    rospy.init_node('rangeBot', anonymous=True)
    global currentState

    print 'waiting for mesg'
    msg = rospy.wait_for_message('/choose_station', Int8)

    # process the location
    locations = processLocations()

    station_pos = locations['station' + str(msg.data)]
    home = locations['home']
    dispenser_pos = locations['dispenser']

    # TO DO:
    # 1. Send the robot to the station
    # 2. Need to integrate gum ball machine
    # 3. Integrate the scale
    # Another two types of messages: '/finish', '/request_balls'
    # states = ['Dispenser', 'Station', 'MovingToStation', 'MovingToDispenser', 'Finish']

    nav = NavTest()

    currentState = 'MovingToDispenser'
    nav.send_to_goal(dispenser_pos)

    while (1):
        print 'Current state = ' + currentState
        if currentState == 'Dispenser':
            print 'At: Dispenser'
            # weightSubscriber = rospy.Subscriber('weight', Float32, callback=processWeight, queue_size=2)
            # while wLock.acquire() and weight != 'Filled':
            #     wLock.release()
            # while weight != 'Filled':
            #     pass

            # weightSubscriber.unregister()

            rospy.sleep(2.0)
            currentState = 'MovingToStation'
            print 'From: Dispenser -- To: MovingToStation'
            nav.send_to_goal(station_pos)
        elif currentState == 'Station':
            print 'At: Station'
            # request more balls
            disp = Dispense()
            req_sub = rospy.Subscriber('/request_balls', Empty, callback=refill_balls, queue_size=1)
            finish_sub = rospy.Subscriber('/done', Empty, finish, queue_size=1)
            # tmp_msg = rospy.wait_for_message('/request_balls', Empty)

            while 1:
                if currentState != 'Station':
                    break
            if currentState == 'Finish':
                nav.send_to_goal(home)
            else:
                # wLock.acquire()
                # weight = 'Empty'
                # wLock.release()
                nav.send_to_goal(dispenser_pos)

            req_sub.unregister()
            finish_sub.unregister()
            disp.shutDown()
        elif currentState == 'MovingToStation':
            # check our status
            if nav.move_base.get_state() == GoalStatus.SUCCEEDED:
                currentState = 'Station'
                pub = rospy.Publisher('at_station', Empty, queue_size=1)
                rospy.sleep(2)
                # time.sleep(5)
                pub.publish(Empty())
                pub.unregister()
            elif nav.move_base.get_state() == GoalStatus.ABORTED:
                res = recoveryPlan(nav, station_pos, home)
                if res:
                    # recovery success
                    currentState = 'Station'
                else:
                    currentState = 'Finish'
            elif nav.move_base.get_state() == GoalStatus.LOST:
                print "message lost"
                nav.move_base.cancel_goal()
                nav.move_base.send_goal(station_pos)
            else:
                continue
        elif currentState == 'MovingToDispenser':
            # check our status
            if nav.move_base.get_state() == GoalStatus.SUCCEEDED:
                currentState = 'Dispenser'
                # weight = 'Empty'
            elif nav.move_base.get_state() == GoalStatus.ABORTED:
                print '-- MovingToDispenser Aborted --'
                res = recoveryPlan(nav, dispenser_pos, home)
                if res:
                    # recovery success
                    currentState = 'Dispenser'
                    print 'From: MovingToDispenser -- To: Dispenser'
                else:
                    currentState = 'Finish'
                    print 'From: MovingToDispenser -- To: Finish'
            elif nav.move_base.get_state() == GoalStatus.LOST:
                print "message lost"
                nav.move_base.cancel_goal()
                nav.move_base.send_goal(dispenser_pos)
            else:
                continue
        else:
            if nav.move_base.get_state() == GoalStatus.ABORTED:
                print "Moving to Home"
                res = recoveryPlan(nav, dispenser_pos, home)
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
    locations = {}
    for key, val in data.items():
        # populate the location Dict
        if 'r' in val.keys():
            locations[key] = [Pose(Point(val['x'], val['y'], val['z']),
                                 Quaternion(val['quatX'], val['quatY'], val['quatZ'], val['quatW'])), val['r']]
        else:
            locations[key] = [Pose(Point(val['x'], val['y'], val['z']),
                                 Quaternion(val['quatX'], val['quatY'], val['quatZ'], val['quatW'])), None]

    return locations

def refill_balls(msg):
    global currentState
    currentState = 'MovingToDispenser'


def finish(msg):
    global currentState
    currentState = 'Finish'


# def processWeight(msg):
#     global weight
#     if msg.data >= 3.0:
#         print 'Basket filled'
#         wLock.acquire()
#         weight = 'Filled'
#         wLock.release()


if __name__ == '__main__':
    main()
