#!/usr/bin/env python
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped, Point, Vector3, PoseStamped
from std_msgs.msg import Header, ColorRGBA
import rospy
import time
from geometry_msgs.msg import Twist
import tf
import math

lock = False
point = None
curr_quaternion = None
firstFileItem = True
id_counter = 0

def writeToFile(fName, cmd, name, point, rot, radius=0):
    global firstFileItem
    print point
    if not firstFileItem:
        fName.write(",\n")
    else:
        firstFileItem = False
    # write data structure into location data file
    fName.write("\t\"" + name + "\": {\n")
    fName.write("\t\t\"x\": " + str(point[0]) + ",\n")
    fName.write("\t\t\"y\": " + str(point[1]) + ",\n")
    fName.write("\t\t\"z\": " + str(point[2]) + ",\n")

    fName.write("\t\t\"quatX\": " + str(rot[0]) + ",\n")
    fName.write("\t\t\"quatY\": " + str(rot[1]) + ",\n")
    fName.write("\t\t\"quatZ\": " + str(rot[2]) + ",\n")
    fName.write("\t\t\"quatW\": " + str(rot[3]))

    print cmd
    if (cmd == "r"):
        fName.write(",\n")
        fName.write("\t\t\"r\": " + str(radius) + "\n")
    else:
        fName.write("\n")
    fName.write("\t}")

def show_text_in_rviz(marker_publisher, curr_point, rot, radius = 0):
    global id_counter
    if radius > 0:
        scaleVector = Vector3(radius, radius, 0.1)
        cylinderColor = ColorRGBA(0.0, 0.0, 1.0, 1)
    else:
        scaleVector = Vector3(0.1, 0.1, 0.1)
        cylinderColor = ColorRGBA(1.0, 0.0, 0.0, 1)

    marker = Marker(type=Marker.CYLINDER, id=id_counter,
                lifetime=rospy.Duration(),
                pose=Pose(Point(curr_point[0], curr_point[1], curr_point[2]), Quaternion(0,0,0,1)),
                scale=scaleVector,
                header=Header(frame_id='map'),
                color=cylinderColor, frame_locked=True)
    marker_publisher.publish(marker)
    id_counter+=1

def getCoords():
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


if __name__ == '__main__':
    rospy.init_node('annotateMap', anonymous=True)
    # rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, processPose)
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    with open('pose-map.json', 'w+') as dFile:
        dFile.write("{\n")
        while (1):

            cmd = raw_input("Enter p or r: ")
            if cmd == 'exit':
                break
            lock = True
            name = raw_input("Enter a name: ")
            point, rot = getCoords()
            radius = 0
            if cmd is "r":
                radius = 1
            writeToFile(dFile, cmd, name, point, rot, radius)
            show_text_in_rviz(marker_publisher, point, rot, radius)
            lock = False
        dFile.write("\n}")
