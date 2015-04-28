#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import traceback
import time
import openravepy
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
    #RaveSetDebugLevel(DebugLevel.Verbose)

    ''' current objects that have strange no point error.
    ['env/cheezit.env.xml', 'env/dentaltreat.env.xml', 'env/outletplugs.env.xml', 'env/pencilcup.env.xml', 'env/sparkplug.env.xml', 'env/stickynotes.env.xml', 'env/strawcups.env.xml']
    '''

    # Load Scene

    cheezit = '../env/cheezit.env.xml'
    colorballs = '../env/colorballs.env.xml'
    crayon = '../env/crayon.env.xml'
    dentaltreat = '../env/dentaltreat.env.xml'
    eraser = '../env/eraser.env.xml'
    glue = '../env/glue.env.xml'
    highlighters = '../env/highlighters.env.xml'
    huckfinn = '../env/huckfinn.env.xml'
    indexcards = '../env/indexcards.env.xml'
    oreo = '../env/oreo.env.xml'
    outletplugs = '../env/outletplugs.env.xml'
    pencil = '../env/pencil.env.xml'
    pencilcup = '../env/pencilcup.env.xml'
    plushduck = '../env/plushduck.env.xml'
    plushfrog = '../env/plushfrog.env.xml'
    rubberduck = '../env/rubberduck.env.xml'
    safetyglasses = '../env/safetyglasses.env.xml'
    screwdrivers = '../env/screwdrivers.env.xml'
    sparkplug = '../env/sparkplug.env.xml'
    stickynotes = '../env/stickynotes.env.xml'
    stirsticks = '../env/stirsticks.env.xml'
    strawcups = '../env/strawcups.env.xml'
    tennisball = '../env/tennisball.env.xml'

    try:
        item = dentaltreat #Set item to load grasps for
        object = env.Load(item)
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
        filename = gmodel.getfilename() # Get filename + location
        print "filename: "
        print filename
        raw_input("Press enter to show grasp")

        # Show grasps
        for index in range(0, len(validgrasps)):
            print "Showing grasp #: " + str(index)
            validgrasp=validgrasps[index] # choose grasp based index number
            gmodel.showgrasp(validgrasp) # show the grasp
            # It says global but the object is at (0,0,0) so this is actually object frame.
            print "Global approach vector: "+str(gmodel.getGlobalApproachDir(validgrasp))
            print "Global grasp transform: "
            transform = gmodel.getGlobalGraspTransform(validgrasp)
            print transform
            print transform.item(0,0)
            print transform.item(0,1)

    except:
        print "Failed to load grasp from database for object: " + item
        print "Traceback of error:"
        print traceback.format_exc()

    raw_input("Press enter to exit...")


