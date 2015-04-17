#!/usr/bin/env python

import math
import rospy
import numpy
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
import sys
import traceback
import time
import openravepy
from itertools import izip
from std_msgs.msg import Header
from geometry_msgs.msg import(
    PoseArray,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Vector3)
from grasp_planner.msg import(
    apcGraspPose,
    apcGraspArray)

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

"""@package docstring
This service takes as input the item. It then searches the database for
validgrasps and returns approach direction and pose for the gripper.
Approach vector is an array: [x y z]
Pose is a split into position and quaternion
Position is an array: [ ]
Quaternion is an array: [ ]
"""

"""
camObjectPose = PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=rightInputX,
                y=rightInputY,
                z=rightInputZ,
            ),
            orientation=Quaternion(
                x=rightInputXR,
                y=rightInputYR,
                z=rightInputZR,
                w=rightInputWR,
            ),
        ),
    )

camObjectPose = Pose(
        position=Point(
            x=rightInputX,
            y=rightInputY,
            z=rightInputZ,
        ),
        orientation=Quaternion(
            x=rightInputXR,
            y=rightInputYR,
            z=rightInputZR,
            w=rightInputWR,
        ),
"""

def matrixToQuat(matrix):
    qw = numpy.sqrt(1 + matrix.item(00) + matrix.item(11) + matrix.item(22))/2
    qx = (matrix.item(2,1) - matrix.item(1,2)) / (4*qw)
    qy = (matrix.item(0,2) - matrix.item(2,0)) / (4*qw)
    qz = (matrix.item(1,0) - matrix.item(0,1)) / (4*qw)
    quat = [qw, qx, qy, qz]
    return quat

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
