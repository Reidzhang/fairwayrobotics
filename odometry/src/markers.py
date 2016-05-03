#!/usr/bin/env python
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import rospy
import tf
import math


def show_text_in_rviz(marker_publisher, points):
    marker = Marker(type=Marker.LINE_STRIP, id=0,
                lifetime=rospy.Duration(10),
                pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.3, 0.2, 0.2),
                points=points,
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 1))
    marker_publisher.publish(marker)

if __name__ == '__main__':
    rospy.init_node('marker_dropper')
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    listener = tf.TransformListener()

    rate = rospy.Rate(0.5)
    oldTrans = None
    points = []
    point_idx = 0
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform("/base_link", "/odom", rospy.Time(0), rospy.Duration(1.0))
            (transNow, rotNow) = listener.lookupTransform("/base_link", "/odom", rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            continue

        if oldTrans is not None:
            distance = math.sqrt(abs(transNow[0] - oldTrans[0]) ** 2 + abs(transNow[1] - oldTrans[1]) ** 2)
            print distance
            if distance > 0.5:
                show_text_in_rviz(marker_publisher, points)
                oldPoint = points[point_idx]
                point_idx = point_idx + 1
                points.append(Point(oldPoint.y + (transNow[1] - oldTrans[1]), oldPoint.x - (transNow[0] - oldTrans[0]), 0))
                print points
                oldTrans = transNow
        else:
            oldTrans = transNow
            points.append(Point(0, 0, 0))

        rate.sleep()
