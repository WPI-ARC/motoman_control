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
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse

def genAPCGraspPose(grasp, object):    
    showOutput = False # If set to true, will show all print statments
    qw = numpy.sqrt(1 + grasp.item(0,0) + grasp.item(1,1) + grasp.item(2,2))/2
    qx = (grasp.item(2,1) - grasp.item(1,2)) / (4*qw)
    qy = (grasp.item(0,2) - grasp.item(2,0)) / (4*qw)
    qz = (grasp.item(1,0) - grasp.item(0,1)) / (4*qw)

    offset = 0.2 #offset by 20 cm
    vecx = grasp.item(0,3) - object.item(0,3)
    vecy = grasp.item(1,3) - object.item(1,3)
    vecz = grasp.item(2,3) - object.item(2,3)
    vector = numpy.array([grasp.item(0,3), grasp.item(1,3), grasp.item(2,3)]) #approach vector
    vectorMagnitude = numpy.sqrt(vecx*vecx+vecy*vecy+vecz*vecz) #magnitude of approach vector
    unitVector = numpy.array([vecx, vecy, vecz])/vectorMagnitude
    offsetVector = unitVector*offset #new grasp point
    newApproachVector = vector + offsetVector

    grasp = apcGraspPose(
        pregrasp = Pose(
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
        approach = Pose(
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


    #graspOffset(grasp)
    if showOutput:
        print "convert matrix to pose msg"           
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
    
def CB_getGrasp(req):
        global graspDict
        showOutput = False # If set to true, will show all print statments

        if showOutput:
            print graspDict

        graspList=[]
        poseList=[]
        Tobjgrasp=[]
        if showOutput:
            print "Requesting valid grasps for %s"%req.item
        if showOutput:
            print "Asking TF for Transform:"
        # try:
        #     (Trobobj_trans,Trobobj_rot) = listener.lookupTransform('/base_link', '/target_object', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print "Failed to lookup TF transform"
        Trob_obj = req.object_pose
        # print Trobobj_trans
        # print Trobobj_rot

        # Load Scene in OpenRave
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
        #env.Load(item) # Load requested item
        item =os.path.basename(item)
        if showOutput:
            print "loading object XML: "+ item

        try:
            graspList = graspDict[item]
            if showOutput:
                print "Grasp list from dictionary"
                print graspList
            for grasp in graspList:
                Tobjgrasp = grasp
                if showOutput:
                    print Tobjgrasp

                TgraspIK = numpy.array([[1, 0, 0, 0],
                                        [0, 0, 1, -0.17],
                                        [0, -1, 0, 0],
                                        [0, 0, 0, 1]])
                Trobobj = quatToMatrix(Trob_obj)
                Trobgrasp = Trobobj * Tobjgrasp
                TrobIK = Trobgrasp * TgraspIK 

                if showOutput:
                    print "Converted Trobgrasp into APCGraspPose msg"
                Trobgrasp_msg = genAPCGraspPose(TrobIK, Trobobj)
                if showOutput:
                    print Trobgrasp_msg
                grasp_qw = Trobgrasp_msg.pregrasp.orientation.w
                grasp_x = Trobgrasp_msg.pregrasp.position.x
                grasp_y = Trobgrasp_msg.pregrasp.position.y
                grasp_z = Trobgrasp_msg.pregrasp.position.z
                approach_qw = Trobgrasp_msg.approach.orientation.w
                approach_x = Trobgrasp_msg.approach.position.x
                approach_y = Trobgrasp_msg.approach.position.y
                approach_z = Trobgrasp_msg.approach.position.z
                
                if math.isnan(float(grasp_qw)) == False and math.isnan(float(approach_qw)) == False  and math.isnan(float(grasp_x)) == False and math.isnan(float(approach_x)) == False  and math.isnan(float(grasp_y)) == False and math.isnan(float(approach_y)) == False  and math.isnan(float(grasp_z)) == False and math.isnan(float(approach_z)) == False:
                    poseList.append(Trobgrasp_msg) # append pose to poselist                
                else:
                    print "Nan/inf value detected. Not appending pose to pose list"
        except:
            print "Failed : " + item
            print "Traceback of error:"
            print traceback.format_exc()
      
        if showOutput:
            print "PoseList:"
        grasps = apcGraspArray(
                grasps = poseList
            )
        if showOutput:
            print grasps
        
        return apcGraspDBResponse(status=True,grasps=grasps)
    
def publisher():
    global graspDict
    # global graspDict
    rospy.init_node('graspDatabase_service')
    rate = rospy.Rate(10.0)
    # graspDict = initialize()

    with open(os.path.join(os.path.dirname(__file__), "graspDict_new.csv")) as file:
        graspDict = pickle.load(file)
    while not rospy.is_shutdown():
        s = rospy.Service('getGrasps', apcGraspDB, CB_getGrasp)
        print "Ready to retrieve grasps from database"
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    publisher()
