#!/usr/bin/env python
# from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped, Point, Vector3
# from std_msgs.msg import Header, ColorRGBA
import rospy
import time
from geometry_msgs.msg import Twist
import tf
import math
from tf.transformations import euler_from_quaternion

go_home_var = False
already_going = False


def go_home(msg):
    global already_going
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    twist = Twist()
    if go_home_var and not already_going:
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z)
        cur_angle =  euler[2] # yaw

        # right angle to go back origin
        # angle = math.atan(position[1]/position[0])
        angle = math.atan2(-position[0], -position[1])
        print "angle = " + str(angle)
        print "cur_angle = " + str(cur_angle)
        dist = math.sqrt(position[0] ** 2 + position[1] ** 2)
        if dist < 0.1:
            # manhantan distance
            twist.linear.x = 0
            twist.angular.z = 0
        else:
            variable = angle + cur_angle
            if angle < 0 and cur_angle < 0:
                if variable > -4.9 and variable < -3.9:
                    twist.linear.x = 0.25
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0.25
            else:
                if variable > 1.3 and variable < 1.7:
                    twist.linear.x = 0.25
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0.25
        pub.publish(twist)




if __name__ == '__main__':
    rospy.init_node('homing_bot', anonymous=True)
    rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, go_home)
    cmd = raw_input("hit 'enter' to go home !")
    go_home_var = True
    rospy.spin()