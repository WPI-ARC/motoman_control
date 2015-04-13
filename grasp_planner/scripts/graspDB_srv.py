#!/usr/bin/env python

import rospy
import numpy
from graspDB.srv import *
import sys
import traceback
import time
import openravepy
from itertools import izip

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

def matrixToQuat(matrix):
    qw = numpy.sqrt(1 + matrix.item(00) + matrix.item(11) + matrix.item(22))/2
    qx = (matrix.item(2,1) - matrix.item(1,2)) / (4*qw)
    qy = (matrix.item(0,2) - matrix.item(2,0)) / (4*qw)
    qz = (matrix.item(1,0) - matrix.item(0,1)) / (4*qw)
    quat = [qw qx qy qz]
    return quat

def CB_getGrasp(req):
    print "Returning valid grasps for %s"%req.item
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    #RaveSetDebugLevel(DebugLevel.Verbose)

    # Load Scene
    if req.item == 'cheezit':
        item = '../env/cheezit.env.xml'
    elif req.item == 'colorballs':
        item = '../env/colorballs.env.xml'
    elif req.item == 'crayon':
        item = '../env/crayon.env.xml'
    elif req.item == 'dentaltreat':
        item = '../env/dentaltreat.env.xml'
    elif req.item == 'eraser':
        item = '../env/eraser.env.xml'
    elif req.item == 'glue':
        item = '../env/glue.env.xml'
    elif req.item == 'highlighters':
        item = '../env/highlighters.env.xml'
    elif req.item == 'huckfinn':
        item = '../env/huckfinn.env.xml'
    elif req.item == 'indexcards':
        item = '../env/indexcards.env.xml'
    elif req.item == 'oreo':
        item = '../env/oreo.env.xml'
    elif req.item == 'outletplugs':
        item = '../env/outletplugs.env.xml'
    elif req.item == 'pencil':
        item = '../env/pencil.env.xml'
    elif req.item == 'pencilcup':
        item = '../env/pencilcup.env.xml'
    elif req.item == 'plushduck':
        item = '../env/plushduck.env.xml'
    elif req.item == 'plushfrog':
        item = '../env/plushfrog.env.xml'
    elif req.item == 'rubberduck':
        item = '../env/rubberduck.env.xml'
    elif req.item == 'safetyglasses':
        item = '../env/safetyglasses.env.xml'
    elif req.item == 'screwdrivers':
        item = '../env/screwdrivers.env.xml'
    elif req.item == 'sparkplug':
        item = '../env/sparkplug.env.xml'
    elif req.item == 'stickynotes':
        item = '../env/stickynotes.env.xml'
    elif req.item == 'stirsticks':
        item = '../env/stirsticks.env.xml'
    elif req.item == 'stawcups':
        item = '../env/strawcups.env.xml'
    elif req.item == 'tennisball':
        item = '../env/tennisball.env.xml'
    else
        print "could not find scene xml for object: %s"%req.item

    try:
        object = env.Load(item) # Set request item
        print "loading object XML: "+ item

        time.sleep(0.1)
        robot = env.GetRobots()[0]

        # Find bodies
        bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
        target = bodies[random.randint(len(bodies))]
        print 'choosing target %s'%target

        # Set manipulator
        robot.SetActiveManipulator('hand')

        # Load database
        gmodel = openravepy.databases.grasping.GraspingModel(robot,target)
        gmodel.load()
        validgrasps,validindices = gmodel.computeValidGrasps(returnnum=inf, checkik=False)
        print "Number of good grasps found: " + str(len(validgrasps))

        raw_input("Press enter to show grasp")

        # Show grasps
        for index in range(0, len(validgrasps)):
            print "Showing grasp #: " + str(index)
            validgrasp=validgrasps[index] # choose grasp based index number
            gmodel.showgrasp(validgrasp) # show the grasp
            # It says global but the object is at (0,0,0) so this is actually object frame.
            approach = gmodel.getGlobalApproachDir(validgrasp)
            print "Global approach vector: "+str(approach)
            print "Global grasp transform: "
            transform = gmodel.getGlobalGraspTransform(validgrasp)
            print transform
            # append approach and pose to list
            approahList[index] = approach
            quatList[index] = matrixToQuat(transform)

    except:
        print "Failed to load grasp from database for object: " + item
        print "Traceback of error:"
        print traceback.format_exc()

    return itemResponse(True)

def publisher():
    # Main loop which requests validgrasps from database and returns it
    # Valid grasps should be in object frame I think

    rospy.init_node('graspDatabase_service')
    s = rospy.Service('getGrasp', item, CB_getGrasp)
    print "Ready to retrieve grasps from database"
    rospy.spin()

if __name__ == '__main__':
    publisher()
