#!/usr/bin/env python

import math
import rospy
import sys
import traceback
import time
import openravepy

from numpy import *
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
#from itertools import izip
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def main():
    try:
        client = rospy.ServiceProxy('getGrasps', apcGraspDB)
        item = 'cheezit_big_original' # Set response item
        """
        Trob_obj = PoseStamped(
            header='base',
            pose=Pose(
                position=Point(
                    x=1,
                    y=1,
                    z=1,
                ),
                orientation=Quaternion(
                    x=1,
                    y=1,
                    z=1,
                    w=1,
                ),
            ),
        )
        """
        Trobobj = pose=Pose(
                position=Point(
                    x=1,
                    y=1,
                    z=1,
                ),
                orientation=Quaternion(
                    x=0.5,
                    y=0.5,
                    z=0.5,
                    w=0.5,
                )
            )

        pose = client(item, Trobobj)
        print "returned pose"
        print pose.status
        print pose.apcGraspArray

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
                
if __name__ == '__main__':    
    main()
