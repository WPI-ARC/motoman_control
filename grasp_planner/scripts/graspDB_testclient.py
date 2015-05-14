#!/usr/bin/env python

import tf
import tf2_ros
import math
import rospy
import sys
import traceback
import time
import openravepy
import geometry_msgs.msg
import sensor_msgs.msg
import moveit_commander
from numpy import *
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray
from sensor_msgs.msg import PointCloud2


if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def main():
    try:
        client = rospy.ServiceProxy('getGrasps', apcGraspDB)
        item = 'expo_dry_erase_board_eraser' # Set response item
        # item = 'cheezit_big_original'
        tfs = []
        pts = []

        msg = geometry_msgs.msg.Pose()
        msg.position.x = 0.885315
        msg.position.y = 0.413907
        msg.position.z = 0.787417
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 1



        points = sensor_msgs.msg.PointCloud2()
        points.data = pts
        binnum = "B"

        response = client(item, binnum, msg, points)
        print "returned pose"
        print response.status
        print response.grasps


    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
                
if __name__ == '__main__':    
    main()
