#!/usr/bin/env python

import math
import rospy
import numpy
import roslib
import sys
import traceback
import time
import os
import csv
import cPickle as pickle
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

if __name__ == "__main__":
    # Set program options
    showOutput = False # If set to true, will show all print statments
    #Debug level, one of (fatal,error,warn,info,debug,verbose,verifyplans)
    #RaveSetDebugLevel(DebugLevel.Verbose)

    # Objects to generate grasps for
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

    objectlist = ['../env/cheezit.env.xml']

    # Generate grasp database for objects in list
    graspDict = {}
    for item in objectlist:
        try:
            grasplist = []

            # Initialize OpenRave
            # RaveSetDebugLevel(DebugLevel.Verbose)
            env = Environment()
            env.SetViewer('qtcoin')
            env.Reset()

            # Load item
            item = os.path.join(os.path.dirname(__file__), item)
            env.Load(item)
            print "loading object XML: "+ item

            # Find bodies
            bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
            target = bodies[random.randint(len(bodies))]
            if showOutput:
                print 'choosing target %s'%target

            # Get robot from environment
            time.sleep(0.1)
            robot = env.GetRobots()[0]

            # Set manipulator
            robot.SetActiveManipulator('hand')

            # Generate grasps
            gmodel = openravepy.databases.grasping.GraspingModel(robot,target)
            if True or not gmodel.load():
                print 'Generating grasp database'
                gmodel.init(friction=0.4,avoidlinks=[])
                gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.2,normalanglerange=0.17), graspingnoise = 0.0)
                gmodel.save()

            # Load grasps
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

            # Transform each grasp from database
            for index in range(0, len(validgrasps)):
                validgrasp=validgrasps[index]                     
                approachVector = gmodel.getGlobalApproachDir(validgrasp) # It says global but the object is at (0,0,0) so this is actually object frame.
                Tobjgrasp = gmodel.getGlobalGraspTransform(validgrasp)
                grasplist.append(Tobjgrasp)
                if showOutput:
                    print "Global approach vector: " + str(approachVector)
                    print "Global grasp transform: " + str(Tobjgrasp)

            # Dictionary of grasps for each object
            item = os.path.basename(item)
            graspDict[item] = grasplist
            env.Reset()
            if showOutput:
                print "Adding grasps for " + item + " to dictionary"
        except:
            print "Failed to generate grasp database for object: " + item +" Skipping object"
            objectfailed.append(item) # Appended failed object to list
            print "Traceback of error:"
            print traceback.format_exc()
            env.Reset() # Reset environment to load next object

    # Create dictionary from grasp library
    print "Finish dictionary generation"
    print "Number of items in dictionary: " + str(len(graspDict))
    RaveDestroy() # destroys all environments and loaded plugins

    # Write graspDict to a csv file using pickle
    with open(os.path.join(os.path.dirname(__file__), "graspDict_new.csv"), "w") as file:
        pickle.dump(graspDict, file)

    print "Finished generating grasp database"
    print "Failed to generate grasp database for object: " + str(objectfailed)
    raw_input("Press enter to exit...")