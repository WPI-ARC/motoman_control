#!/usr/bin/env python


import tf
import math
import rospy
import numpy
import roslib
import sys
import traceback
import time
#from itertools import izip
"""OpenRave dependencies"""
import openravepy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

"""
br = tf2_ros.TransformBroadcaster()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "grasp"
t.transform.translation = grasp.position
t.transform.rotation = grasp.orientation
br.sendTransform(t)

"""

"""
def graspOffset(apcPose):
    offset = 0.1 #offset by 10 cm
    x = apcPose.posegrasp.position.x
    y = apcPose.posegrasp.position.y
    z = apcPose.posegrasp.position.z
    vecx = apcPose.poseapproach.x
    vecy = apcPose.poseapproach.y
    vecz = apcPose.poseapproach.z
    vector = numpy.vector([vecx, vecy, vecz]) #approach vector
    vectorMagnitude = numpy.sqrt(vecx,vecy,vecz) #magnitude of approach vector
    unitVector = vector/vectorMagnitude
    offsetVector = -unitVector*offset #new grasp point

    print newPoint
"""
def genAPCGraspPose(grasp, object):    
    showOutput = False # If set to true, will show all print statments
    qw = numpy.sqrt(1 + grasp.item(0,0) + grasp.item(1,1) + grasp.item(2,2))/2
    qx = (grasp.item(2,1) - grasp.item(1,2)) / (4*qw)
    qy = (grasp.item(0,2) - grasp.item(2,0)) / (4*qw)
    qz = (grasp.item(1,0) - grasp.item(0,1)) / (4*qw)

    offset = 0.1 #offset by 10 cm
    vecx = grasp.item(0,3) - object.item(0,3)
    vecy = grasp.item(1,3) - object.item(1,3)
    vecz = grasp.item(2,3) - object.item(2,3)
    vector = numpy.array([grasp.item(0,3), grasp.item(1,3), grasp.item(2,3)]) #approach vector
    vectorMagnitude = numpy.sqrt(vecx*vecx+vecy*vecy+vecz*vecz) #magnitude of approach vector
    unitVector = numpy.array([vecx, vecy, vecz])/vectorMagnitude
    offsetVector = unitVector*offset #new grasp point
    newApproachVector = vector + offsetVector

    grasp = apcGraspPose(
        posegrasp = Pose(
            position=Point(
                x=grasp.item(0,3),
                y=grasp.item(1,3),
                z=grasp.item(2,3),
            ),
            orientation=Quaternion(
                x=qx,
                y=qy,
                z=qz,
                w=qw,
            )
        ),    
        poseapproach = Pose(
            position=Point(
                x=newApproachVector.item(0),
                y=newApproachVector.item(1),
                z=newApproachVector.item(2),
            ),
            orientation=Quaternion(
                x=qx,
                y=qy,
                z=qz,
                w=qw,
            )
        )
    )
      
    return grasp

def quatToMatrix(quat):
    showOutput = False # If set to true, will show all print statments
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
    if showOutput:
        print "Inside quatToMatrix(). Conversion of req.Trob_obj to numpy matrix"
        print Trobobj
    return Trobobj

        if True: # If true is here just so i can hide the list # if true is only used to hide the long list of if statements
def requestOBBPoints(size):
    # do a service call to get min max of object. Or maybe a subscribe to topic is fine too since all services start when
    # the robot launches. Alex code is requesting sample and process service so maybe when those are call just publish the min max values to
    # a topic. May be the easiest way
    x = size.item(0)
    y= size.item(1)
    z = size.item(2)

    point1 = (x/2, y/2, z/2)
    point2 = (x/2, -y/2, z/2)
    point3 = (-x/2, y/2, z/2)
    point4 = (-x/2, -y/2, z/2)
    point5 = (x/2, y/2, -z/2)
    point6 = (x/2, -y/2, -z/2)
    point7 = (-x/2, y/2, -z/2)
    point8 = (-x/2, -y/2, -z/2)
    points = numpy.vector([point1, point2, point3, point4, point5, point6, point7, point8])
    """

    delta = numpy.array([maxx-minx maxy-miny maxz-minz]) #return array vector that contains the delta for x, y, and z axis of bounding box.
    """
    return points

def computeWidth(min_y, max_y):
    width = abs(abs(max_y)-abs(min_y))
    return width

def checkWidth(width):    
    offset = 0.02 # safety buffer between object and gripper is 1cm on each side. Total of 2cm
    gripperwidth = 0.155 - offset
    if width <= gripperwidth: # 0.155 is the gripper width in meters. 15.5cm
        isSmaller = True
    else:
        isSmaller = False
    return isSmaller

def computeCost(width):
    offset = 0.02 # safety buffer between object and gripper is 1cm on each side. Total of 2cm
    gripperwidth = 0.155 - offset
    cost = width/gripperwidth
    return cost

def findMinMax(pointList):
    #Loop through pointList to find the min & max for the x, y, and z
    # min_x = 0;
    # min_y = 0;
    # min_z = 0;
    # max_x = 0;
    # max_y = 0;
    # max_z = 0;
    for point in pointList:
        if point.item(0) > max_x: #if x point is larger than current max x then replace with new one
            max_x = point.item(0)
        if point.item(0) < min_x: #if x point is smaller than current min x then replace with new one
            min_x = point.item(0)
        if point.item(1) > max_y: #if x point is larger than current max x then replace with new one
            max_y = point.item(1)
        if point.item(1) < min_y: #if x point is smaller than current min x then replace with new one
            min_y = point.item(1)
        if point.item(2) > max_z: #if x point is larger than current max x then replace with new one
            max_z = point.item(2)
        if point.item(2) < min_z: #if x point is smaller than current min x then replace with new one
            min_z = point.item(2)
    min_max = numpy.vector([min_x, max_x, min_y, max_y, min_z, max_z])
    return min_max

def genRotMatrix(theta): 
    # Generate matrix to rotation about the z-axis of shelf frame to get bunch of new transforms to use for projection
    transform = numpy.matrix([[numpy.cos(theta), -numpy.sin(theta), 0, 0],
                            [numpy.sin(theta), numpy.cos(theta), 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    return transform                        

def CB_getGrasp(req):
    showOutput = False # Enable to show print statements
    pointsList = []

    print "Requesting valid grasps for %s"%req.item

    """
    print "Asking TF for Transform from robot base to object:"                
    try:
        (Trobobj_trans,Trobobj_rot) = listener.lookupTransform('/base_link', '/target_object', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Failed to lookup TF transform"        
    print Trobobj_trans
    print Trobobj_rot
    """

    Trob_obj = req.Trob_obj

    # Select object
    if True: # If true is here just so i can hide the list # if true is only used to hide the long list of if statements
        if req.item == 'cheezit_big_original':
            item = '../env/cheezit.env.xml'
            size = [0.062, 0.16, 0.226]
        elif req.item == 'kyjen_squeakin_eggs_plush_puppies':
            item = '../env/colorballs.env.xml'
            size = [0.06, 0.121, 0.18]
        elif req.item == 'crayola_64_ct':
            item = '../env/crayon.env.xml'
            size = [0.037, 0.125, 0.145]
        elif req.item == 'feline_greenies_dental_treats':
            item = '../env/dentaltreat.env.xml'
            size = [0.04, 0.175, 0.21]
        elif req.item == 'expo_dry_erase_board_eraser':
            item = '../env/eraser.env.xml'
            size = [0.035, 0.055, 0.13]
        elif req.item == 'elmers_washable_no_run_school_glue':
            item = '../env/glue.env.xml'
            size = [0.025, 0.06, 0.145]
        elif req.item == 'sharpie_accent_tank_style_highlighters':
            item = '../env/highlighters.env.xml'
            size = [0.023, 0.116, 0.125]
        elif req.item == 'mark_twain_huckleberry_finn':
            item = '../env/huckfinn.env.xml'
            size = [0.012, 0.132, 0.21]
        elif req.item == 'mead_index_cards':
            item = '../env/indexcards.env.xml'
            size = [0.02, 0.076, 0.127]
        elif req.item == 'oreo_mega_stuf':
            item = '../env/oreo.env.xml'
            size = [0.06, 0.155, 0.2]
        elif req.item == 'mommys_helper_outlet_plugs':
            item = '../env/outletplugs.env.xml'
            size = [0.055, 0.09, 0.19]
        elif req.item == 'paper_mate_12_count_mirado_black_warrior':
            item = '../env/pencil.env.xml'
            size = [0.016, 0.05, 0.195]
        elif req.item == 'rolodex_jumbo_pencil_cup':
            item = '../env/pencilcup.env.xml'
            size = [0.107, 0.14, 0.107]
        elif req.item == 'kong_duck_dog_toy':
            item = '../env/plushduck.env.xml'
            size = [0.05, 0.09, 0.18]
        elif req.item == 'kong_sitting_frog_dog_toy':
            item = '../env/plushfrog.env.xml'
            size = [0.045, 0.09, 0.21]
        elif req.item == 'munchkin_white_hot_duck_bath_toy':
            item = '../env/rubberduck.env.xml'
            size = [0.065, 0.095, 0.13]
        elif req.item == 'safety_works_safety_glasses':
            item = '../env/safetyglasses.env.xml'
            size = [0.05, 0.07, 0.2]
        elif req.item == 'stanley_66_052':
            item = '../env/screwdrivers.env.xml'
            size = [0.02, 0.1, 0.195]
        elif req.item == 'champion_copper_plus_spark_plug':
            item = '../env/sparkplug.env.xml'
            size = [0.02, 0.025, 0.095]
        elif req.item == 'highland_6539_self_stick_notes':
            item = '../env/stickynotes.env.xml'
            size = [0.05, 0.04, 0.115]
        elif req.item == 'genuine_joe_plastic_stir_sticks':
            item = '../env/stirsticks.env.xml'
            size = [0.095, 0.107, 0.145]
        elif req.item == 'first_years_take_and_toss_straw_cup':
            item = '../env/strawcups.env.xml'
            size = [0.08, 0.09, 0.17]
        elif req.item == 'kong_air_dog_squeakair_tennis_ball':
            item = '../env/tennisball.env.xml'
            size = [0.07, 0.108, 0.19]
        else:
            print "could not find scene xml for object: %s"%req.item 

    """
    Right now just computing one approach vector which is straight into bin. First make sure this works. After it does the way I
    have it set up I should be able to just simply loop this same structure. I can have a phi angle which is +-phi. Phi is combine with theta to 
    used to get the transformation matrix. This is used with maybe the arc of a circle with center at center of object return from vision code. maybe
    phi is simply the angles that give lines tangent to the circle. This is probably more consistent method to get more theta values. But anyway. I want 
    to transform the transform matrix I have for object to all of the new ones which are rotated by some angle wrt to the object frame. This way I can
    compute the different widths and if valid will compute the appraoch vectors. Then the height offset etc. 
    """
    #make an array with theta values that I get by discretizing a range from minus to plus like +-10 degrees. Then create list of of the theta values then use it to generate the extra frames for projection to get more approach vectors. Anothe way is to use a circle then a small segment of it Then find lines that are tangent to it. use the lines rotation, then use the points that reside on that line as the xyz transform for the 4x4 matrix. Now I project onto all of those points. Use TF to show all of the frames as part of presentation
    
    # Request for min max values for the axes from bounding box
    # Request 8 bounding box points
    OBBPoints = requestOBBPoints(size)

    # Request transform from shelf to object from TF
    if showOutput:
        print "Looking up TF transform from shelf to object"
    try:
        (Tshelfobj_trans,Tshelfobj_rot) = listener.lookupTransform('/shelf', '/target_object', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "Failed to lookup TF transform shelf to target_object"

    # Construct the 4x4 Transformation Matrix
    Tshelfobj = numpy.matrix([[Tshelfobj_rot.item(0,0), Tshelfobj_rot.item(0,1), Tshelfobj_rot.item(0,2), Tshelfobj_trans.item(0,3)],
                            [Tshelfobj_rot.item(1,0), Tshelfobj_rot.item(1,1), Tshelfobj_rot.item(1,2), Tshelfobj_trans.item(1,3)],
                            [Tshelfobj_rot.item(2,0), Tshelfobj_rot.item(2,1), Tshelfobj_rot.item(2,2), Tshelfobj_trans.item(2,3)],
                            [0, 0, 0, 1]])
    if showOutput:
        print Tshelfobj

    # Loop through OBB points and transform them to be wrt to the target frame
    for OBBPoint in OBBPoints:
        pointsList.append(Tshelfobj*OBBPoint)

    # Compute width of projection shadow. Width is the y axis because shelf frame is set that way with y axis as width
    width = computeWidth(min_y, max_y)

    # Computel cost
    if isSmaller(width) == True:
        cost = computeCost(width)
        
        # should also compute the x,y,z or pose or approach vector? Then append to to some list
        # after implemnted loop of this for different frame projections, add else case so it prints somehting like failed or something for this frame










    # From center of palm to edge of it is 6cm then lip is about 2cm up so need to offset robot z-axis to move hand approach vector to be 8cm
    # up from bottom of object.        
    """
    # Function to compute the width of shadow. populate component 1 and 2 depending on which two vectors lenghts I care about
    #component1 = delta.item(#)
    #component2 = delta.item(#)
    #theta = computeTheta()
    width = computeWidth(component1, component2, theta)

    # Check if width of shadow projection can fit inside gripper width
    isSmaller = checkWidth(width)

    # If width is smaller than compute the approach vector to grasp center of projection
    if isSmaller == True:
        approach = computeApproach()
    else:
        fail = True
        print "Unable to grasp object"

    # Construct the poseApproach msg and return just like in offline planner
    """





    return apcGraspDBResponse(status=True,apcGraspArray=grasps)
    
def publisher():
    rospy.init_node('graspDatabaseOnline_service')
    rate = rospy.Rate(60.0)
    graspDict = initialize()
    #print graspDict
    while not rospy.is_shutdown():
        #t.header.stamp = rospy.Time.now()
        #br.sendTransform(t)
        #rospy.Subscriber("/object_segmentation/pose", PoseStamped, )
        s = rospy.Service('getGrasps_onlineplanner', apcGraspDB, CB_getGrasp)
        print "Online planner ready"
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    publisher()
