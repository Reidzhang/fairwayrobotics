#!/usr/bin/env python

"""
    Fairway robotics
"""
import json
import roslib; roslib.load_manifest('odometry')
import rospy
import actionlib
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pow, sqrt

class NavTest():
    def __init__(self):
        rospy.init_node('sendRobot', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.type = 'P'
        self.location = []
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        # initial_pose = PoseWithCovarianceStamped()

        # Get the initial pose from the user
        rospy.loginfo("Starting navigation test")

    def send_to_goal(self, location):
        if location[1] is not None:
            self.type = 'r'
        else:
            self.type = 'p'

        self.location = location

        goal = MoveBaseGoal()
        # set goal
        goal.target_pose.pose = location[0]
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(goal, feedback_cb=self.check_progress)

        self.move_base.wait_for_result()

        # print result
        print self.move_base.get_result()

    def check_progress(self, f_back):
        # check the prgross
        cur_pos = f_back.base_position.pose.position
        goal_pos = self.location[0]
        if self.type == 'r':
            # going for a region
            x_diff = cur_pos.x - goal_pos.position.x
            y_diff = cur_pos.y - goal_pos.position.y
            z_diff = cur_pos.z - goal_pos.position.y
            dist = sqrt(x_diff ** 2 + y_diff ** 2 + z_diff ** 2)
            if dist < int(self.location[1]):
                self.move_base.cancel_goal()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    # rospy.init_node('sendRobot', anonymous=True)
    with open('pose-map.json', 'r') as dFile:
        data = json.loads(dFile.read())
    # now we have the data
    # we can print out the name
    print "You have the following known locations"
    nav = NavTest()
    locations = dict()
    i = 0
    for key, val in data.items():
        # populate the location Dict
        if 'r' in val.keys():
            locations[i] = [Pose(Point(val['x'], val['y'], val['z']), Quaternion(val['quatX'], val['quatY'], val['quatZ'], val['quatW'])), val['r']]
        else:
            locations[i] = [Pose(Point(val['x'], val['y'], val['z']), Quaternion(val['quatX'], val['quatY'], val['quatZ'], val['quatW'])), None]
        print str(i) + " " + key
        i += 1
    goal = None
    while (1):
        index = raw_input("Enter an Index for location: ")
        if index == 'exit':
            break
        goal = locations[int(index)]
        nav.send_to_goal(locations[int(index)])

    nav.shutdown()
