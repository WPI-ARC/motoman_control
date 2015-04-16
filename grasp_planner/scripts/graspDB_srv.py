#!/usr/bin/env python

import rospy
import numpy
from grasp_planner.srv import *
import sys
import traceback
import time
import openravepy
from itertools import izip
from geometry_msgs.msg import(
    PoseArray,
    PoseStamped,
    Pose,
    Point,
    Quaternion)

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

def matrixToPose(matrix):    
    qw = numpy.sqrt(1 + matrix.item(0,0) + matrix.item(1,1) + matrix.item(2,2))/2
    qx = (matrix.item(2,1) - matrix.item(1,2)) / (4*qw)
    qy = (matrix.item(0,2) - matrix.item(2,0)) / (4*qw)
    qz = (matrix.item(1,0) - matrix.item(0,1)) / (4*qw)
    """
    quat.position.x = matrix.item(0,3)
    quat.position.y = matrix.item(1,3)
    quat.position.z = matrix.item(2,3)
    quat.orientation.w = qw
    quat.orientation.x = qx
    quat.orientation.y = qy
    quat.orientation.z = qz
    """
    pose = Pose(
        position=Point(
            x=matrix.item(0,3),
            y=matrix.item(1,3),
            z=matrix.item(2,3),
        ),
        orientation=Quaternion(
            x=qx,
            y=qy,
            z=qz,
            w=qw,
        )
    )
    print "convert matrix to pose msg"           
    return pose

def quatToMatrix(quat):
    x = quat.position.x
    y = quat.position.y
    z = quat.position.z
    qw = quat.orientation.w
    qx = quat.orientation.x
    qy = quat.orientation.y
    qz = quat.orientation.z

    Trobobj = numpy.matrix([[1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz + 2*qy*qw, x],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, y],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, z],
        [0, 0, 0, 1]])             
    print "Inside quatToMatrix(). Conversion of req.Trob_obj to numpy matrix"
    print Trobobj
    return Trobobj
    
#def genTargetPose(graspPoseMsg, graspPoseReq):
    # take pose of multiply together to get trasnfrom of robot frame to grasping frame.
    # Outputs transform from grasping frame wrt to base frame to use for arm code.    

def CB_getGrasp(req):
    print "Returning valid grasps for %s"%req.item
    print "Input Transform pose:"
    Trob_obj = req.Trob_obj
    print Trob_obj
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
    else:
        print "could not find scene xml for object: %s"%req.item

    try:
        approachList=[]
        poseList=[]
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
            print "Transfrom from library: %i"%index
            print transform
            Tobjgrasp = numpy.matrix([[transform.item(0,0), transform.item(0,1), transform.item(0,2), transform.item(0,3)],
                                    [transform.item(1,0), transform.item(1,1), transform.item(1,2), transform.item(1,3)],
                                    [transform.item(2,0), transform.item(2,1), transform.item(2,2), transform.item(2,3)],
                                    [transform.item(3,0), transform.item(3,1), transform.item(3,2), transform.item(3,3)]])             
            print "Transform from library after conversion to numpy matrix. should be same as above"
            print Tobjgrasp
            Trobobj = quatToMatrix(Trob_obj)
            Trobgrasp = Trobobj * Tobjgrasp

            # Reponse msg
            """
            Trobgrasp = PoseArray(
                header = "base_link",
                poses= Pose(
                        position=Point(
                            x=transform.item(0,1),
                            y=transform.item(0,2),
                            z=transform.item(0,3),
                        ),
                        orientation=Quaternion(
                            x=quat.qx,
                            y=quat.qy,
                            z=quat.qz,
                            w=quat.qw+1,
                        )
                    )
                )
            """
            print "Concatenated Transform Trobgrasp:"
            print Trobgrasp
            print "Converted Trobgrasp into Pose msg"
            Trobgrasp_msg = matrixToPose(Trobgrasp)
            print Trobgrasp_msg

            poseList.append(Trobgrasp_msg)
            #print poseList                
            # append approach and pose to list  
            #approachList.append = approach
            poseList.append = Trobgrasp_msg
    except:
        print "Failed to load grasp from database for object: " + item
        print "Traceback of error:"
        print traceback.format_exc()


    temp = []
    
    print "PoseList:"
    poseArrayList = PoseArray(
            #header = Header(stamp=rospy.time.now(), frame_id='base_link'),
            header = "base_link",
            poses = temp
        )
    print poseArrayList

    return graspDBResponse(status=True,Trobgrasp=poseArrayList)

def publisher():
    # Main loop which requests validgrasps from database and returns it
    # Valid grasps should be in object frame I think

    rospy.init_node('graspDatabase_service')
    s = rospy.Service('getGrasps', graspDB, CB_getGrasp)
    print "Ready to retrieve grasps from database"
    rospy.spin()

if __name__ == '__main__':
    publisher()
