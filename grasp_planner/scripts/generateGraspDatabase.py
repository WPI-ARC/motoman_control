#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import traceback
import time
import openravepy
import os
from itertools import izip

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    RaveSetDebugLevel(DebugLevel.Verbose)

    # Set program options
    showgoodgrasp = False # Use to show or not show good grasp. False will skip showing good grasps

    ''' current objects that have strange no point error.
    ['env/cheezit.env.xml', 'env/dentaltreat.env.xml', 'env/outletplugs.env.xml', 'env/pencilcup.env.xml', 'env/sparkplug.env.xml', 'env/stickynotes.env.xml', 'env/strawcups.env.xml']
    '''

    # Load Scene
    objectfailed = []
    objectlist = ['../env/cheezit.env.xml',
                  '../env/colorballs.env.xml',
                  '../env/crayon.env.xml',
                  '../env/dentaltreat.env.xml',
                  '../env/eraser.env.xml',
                  '../env/glue.env.xml',
                  '../env/highlighters.env.xml',
                  '../env/huckfinn.env.xml',
                  '../env/indexcards.env.xml',
                  '../env/oreo.env.xml',
                  '../env/outletplugs.env.xml',
                  '../env/pencil.env.xml',
                  '../env/pencilcup.env.xml',
                  '../env/plushduck.env.xml',
                  '../env/plushfrog.env.xml',
                  '../env/rubberduck.env.xml',
                  '../env/safetyglasses.env.xml',
                  '../env/screwdrivers.env.xml',
                  '../env/sparkplug.env.xml',
                  '../env/stickynotes.env.xml',
                  '../env/stirsticks.env.xml',
                  '../env/strawcups.env.xml',
                  '../env/tennisball.env.xml']

    print objectlist

    # Generate grasp database for objects in list
    for item in objectlist:
        try:
            item = os.path.join(os.path.dirname(__file__), item)
            print "Loading object XML: "+ item
            object = env.Load(item)

            time.sleep(0.1)
            robot = env.GetRobots()[0]

            # Find bodies
            bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
            target = bodies[random.randint(len(bodies))]
            print 'choosing target %s'%target

            # Set manipulator
            robot.SetActiveManipulator('hand')

            # Generate grasp
            # It seems to load database I just pass in robot and target. So the robotiq hand and object I want. Will need
            # to probably make it so database only saves pose and not joint values or anything. Check the parameters i can set for
            # computevalidgrasps(). Then need to populate a list then reorder the quaternion to match format of it in moveit. Then
            # need to do some transrom between object frame and robot camera frame?? Maybe one of the parameters to set saves the object
            # frame transform??
            gmodel = openravepy.databases.grasping.GraspingModel(robot,target)
            if not gmodel.load():
                print 'generating grasp database'
                #gmodel.autogenerate()
                gmodel.init(friction=0.4,avoidlinks=[])
                gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.04,normalanglerange=0))
                gmodel.save()

            if showgoodgrasp:
                ''' Parameters for computevalidgrasps()
                :param returnnum: If set, will also return once that many number of grasps are found.
                :param backupdist: If > 0, then will move the hand along negative approach direction and check for validity.
                :param checkgrasper: If True, will execute the grasp and check if gripper only contacts target.
                :param startindex: The index to start searching for grasps
                :param checkik: If True will check that the grasp is reachable by the arm.
                :param checkcollision: If true will return only collision-free grasps. If checkik is also True, will return grasps that have collision-free arm solutions.
                '''
                validgrasps,validindices = gmodel.computeValidGrasps(returnnum=inf, checkik=False)
                print "Number of good grasps found: " + str(len(validgrasps))
                raw_input("Press enter to show grasp")
                for index in range(0, len(validgrasps)):
                    validgrasp=validgrasps[index] # choose grasp based index number
                    gmodel.showgrasp(validgrasp) # show the grasp
            env.Reset()
        except:
            print "Failed to generate grasp database for object: " + item
            print "Skipping object"
            objectfailed.append(item) # Appended failed object to list
            print "Traceback of error:"
            print traceback.format_exc()
            env.Reset() # Reset environment to load next object

    print "Finished generating grasp database"
    print "Failed to generate grasp database for object: "
    print objectfailed

    raw_input("Press enter to exit...")


