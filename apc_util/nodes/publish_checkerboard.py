#!/usr/bin/python

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

if __name__ == "__main__":
    rospy.init_node("publish_checkerboard")
    markerPub = rospy.Publisher('checkerboard_marker', Marker)

    startX = 0.2760
    startY = 0.4665
    startZ = 0.002
    boxWidth = 0.0245

    checkerboard_marker = Marker()
    checkerboard_marker.type = Marker.CUBE_LIST
    checkerboard_marker.header.frame_id = "/base_link"
    checkerboard_marker.action = Marker.ADD
    checkerboard_marker.frame_locked = True
    checkerboard_marker.ns = "checkerboard_marker"
    checkerboard_marker.id = 1
    checkerboard_marker.scale.x = 0.0245
    checkerboard_marker.scale.y = 0.0245
    checkerboard_marker.scale.z = 0.004
    checkerboard_marker.pose.position.x = startX - (0.5 * boxWidth)
    checkerboard_marker.pose.position.y = startY - (0.5 * boxWidth)
    checkerboard_marker.pose.position.z = startZ
    checkerboard_marker.pose.orientation.x = 0
    checkerboard_marker.pose.orientation.y = 0
    checkerboard_marker.pose.orientation.z = 0
    checkerboard_marker.pose.orientation.w = 1

    colorBlack = False
    for x in range(6):
        for y in range(5):
            point = Point(x * boxWidth, y * boxWidth, 0)
            checkerboard_marker.points.append(point)

            colorBlack = not colorBlack
            if colorBlack:
                color = ColorRGBA(0.0, 0.0, 0.0, 0.6)
            else:
                color = ColorRGBA(1.0, 1.0, 1.0, 0.6)
            checkerboard_marker.colors.append(color)

    while not rospy.is_shutdown():
        markerPub.publish(checkerboard_marker)
        rospy.sleep(1.0)
