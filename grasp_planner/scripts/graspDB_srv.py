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

def genAPCGraspPose(matrix,vector):    
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
    grasp = apcGraspPose(
        posegrasp = Pose(
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
        ),    
        poseapproach = Vector3(
            x=vector.item(0),
            y=vector.item(1),
            z=vector.item(2)
            )
        )

    print "convert matrix to pose msg"           
    return grasp

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
    if req.item == 'cheezit_big_original':
        item = '../env/cheezit.env.xml'
    elif req.item == 'kyjen_squeakin_eggs_plush_puppies':
        item = '../env/colorballs.env.xml'
    elif req.item == 'crayola_64_ct':
        item = '../env/crayon.env.xml'
    elif req.item == 'feline_greenies_dental_treats':
        item = '../env/dentaltreat.env.xml'
    elif req.item == 'expo_dry_erase_board_eraser':
        item = '../env/eraser.env.xml'
    elif req.item == 'elmers_washable_no_run_school_glue':
        item = '../env/glue.env.xml'
    elif req.item == 'sharpie_accent_tank_style_highlighters':
        item = '../env/highlighters.env.xml'
    elif req.item == 'mark_twain_huckleberry_finn':
        item = '../env/huckfinn.env.xml'
    elif req.item == 'mead_index_cards':
        item = '../env/indexcards.env.xml'
    elif req.item == 'oreo_mega_stuf':
        item = '../env/oreo.env.xml'
    elif req.item == 'mommys_helper_outlet_plugs':
        item = '../env/outletplugs.env.xml'
    elif req.item == 'paper_mate_12_count_mirado_black_warrior':
        item = '../env/pencil.env.xml'
    elif req.item == 'rolodex_jumbo_pencil_cup':
        item = '../env/pencilcup.env.xml'
    elif req.item == 'kong_duck_dog_toy':
        item = '../env/plushduck.env.xml'
    elif req.item == 'kong_sitting_frog_dog_toy':
        item = '../env/plushfrog.env.xml'
    elif req.item == 'munchkin_white_hot_duck_bath_toy':
        item = '../env/rubberduck.env.xml'
    elif req.item == 'safety_works_safety_glasses':
        item = '../env/safetyglasses.env.xml'
    elif req.item == 'stanley_66_052':
        item = '../env/screwdrivers.env.xml'
    elif req.item == 'champion_copper_plus_spark_plug':
        item = '../env/sparkplug.env.xml'
    elif req.item == 'highland_6539_self_stick_notes':
        item = '../env/stickynotes.env.xml'
    elif req.item == 'genuine_joe_plastic_stir_sticks':
        item = '../env/stirsticks.env.xml'
    elif req.item == 'first_years_take_and_toss_straw_cup':
        item = '../env/strawcups.env.xml'
    elif req.item == 'kong_air_dog_squeakair_tennis_ball':
        item = '../env/tennisball.env.xml'
    else:
        print "could not find scene xml for object: %s"%req.item

    try:
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
            approachVector = numpy.array([approach.item(0), approach.item(1), approach.item(2)])
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
            """
            print "Concatenated Transform Trobgrasp:"
            print Trobgrasp
            print "Converted Trobgrasp into APCGraspPose msg"
            Trobgrasp_msg = genAPCGraspPose(Trobgrasp,approachVector) # This is a pose msg
            print Trobgrasp_msg
            qw = Trobgrasp_msg.posegrasp.orientation.w
            if math.isnan(float(qw)) == False:
                poseList.append(Trobgrasp_msg) # append pose to poselist                
            elif math.isnan(float(qw)) == True:
                print "Nan value detected. Not appending pose to pose list"

            #print poseList                
            # append approach and pose to list  
            #approachList.append = approach
            #poseList.append = Trobgrasp_msg
    except:
        print "Failed to load grasp from database for object: " + item
        print "Traceback of error:"
        print traceback.format_exc()


    temp = []
    
    print "PoseList:"
    grasps = apcGraspArray(
            grasps = poseList
        )
    print grasps
    #res.status=True
    #res.Trobgrasp=poseArrayList

    #return graspDBResponse(res)
    return apcGraspDBResponse(status=True,apcGraspArray=grasps)
    #return graspDBResponse(status=True)
def publisher():
    # Main loop which requests validgrasps from database and returns it
    # Valid grasps should be in object frame I think

    rospy.init_node('graspDatabase_service')
    s = rospy.Service('getGrasps', apcGraspDB, CB_getGrasp)
    print "Ready to retrieve grasps from database"
    rospy.spin()

if __name__ == '__main__':
    publisher()
