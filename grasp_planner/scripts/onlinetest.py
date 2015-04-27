#!/usr/bin/env python

import math
import rospy
import sys
import traceback
import time
import openravepy
import geometry_msgs.msg
from numpy import *
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray

def main():
    try:
        client = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
        item = 'expo_dry_erase_board_eraser' # Set response item
        #item = 'cheezit_big_original'

        msg = geometry_msgs.msg.Pose()
        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = 0
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 1

        pose = client(item, msg)
        print "returned pose"
        print pose.status
        print pose.apcGraspArray

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print traceback.format_exc()
                
if __name__ == '__main__':    
    main()
