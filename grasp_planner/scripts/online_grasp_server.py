#!/usr/bin/env python


import tf
import tf2_ros
import math
import rospy
import numpy
import roslib
import sys
import traceback
import time
import geometry_msgs.msg
from copy import deepcopy
# Transformation helper file. quater is (x,y,z,w) format
from transformation_helper import ExtractFromMatrix, BuildMatrix, PoseFromMatrix, PoseToMatrix, InvertMatrix
# Message and Service imports
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion, TransformStamped
from grasp_planner.msg import apcGraspPose, apcGraspArray
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse

class grasping:

    def __init__(self):
        self.description = "Online grasp planning code"
        self.offset = 0.02 # safety buffer between object and gripper is 1cm on each side. Total of 2cm
        self.gripperwidth = 0.155 - self.offset # 0.155 is the gripper width in meters. 15.5cm
        self.showOutput = False # Enable to show print statements
        self.tf = tf.TransformListener(True, rospy.Duration(10.0))
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(60.0)
        self.tfList = []        
        self.thetaList = numpy.linspace(-0.34906585, 0.34906585, num=41) # 0.174532925 = 10deg. SO range is from -10deg to 10deg using num=41 spacing        
        self.thetaList = numpy.linspace(-0.698131701, 0.698131701, num=21)
        #self.thetaList = numpy.linspace(0, 0.34906585, num=21) # 0.174532925 = 10deg. SO range is from -10deg to 10deg using num=41 spacing        
        self.thetaList = numpy.array([0])
        #self.thetaList = numpy.array([0.0174532925]) #1 deg
        #self.thetaList = numpy.array([1.57079633 ]) #90 deg
        #self.thetaList = numpy.array([0.785398163]) #45 deg
        if self.showOutput:
            print "theta range list"
            print self.thetaList

        rospy.sleep(rospy.Duration(1.0)) # Wait for network timing to load TFs

    def get_tf(self, parent, child): #target = parent
        if self.showOutput:
            print "Looking up TF transform from %s to %s" %(parent, child)
        try:
            (trans, quat) = self.tf.lookupTransform(parent, child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Failed to lookup TF transform from source:%s to target:%s" %(parent, child)
        # print trans
        # print type(trans)
        # print rot
        trans = numpy.asarray(trans)
        quat = numpy.asarray(quat)
        translation = [trans.item(0), trans.item(1), trans.item(2)]
        quaternion = [quat.item(0), quat.item(1), quat.item(2), quat.item(3)]
        # print type(trans)
        # print trans
        transform = BuildMatrix(translation, quaternion)
        return transform

    def get_tf_msg(self, parent, child):
        if self.showOutput:
            print "Looking up TF transform from %s to %s" %(parent, child)
        try:
            (trans, quat) = self.tf.lookupTransform(parent, child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Failed to lookup TF transform from source:%s to target:%s" %(parent, child)
        translation = numpy.asarray(trans)
        quaternion = numpy.asarray(quat)
        msg = geometry_msgs.msg.Pose()
        msg.position.x = translation[0]
        msg.position.y = translation[1]
        msg.position.z = translation[2]     
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        return msg

    def broadcast_tf(self, tfList):
        rate = rospy.Rate(1000)        
        for time in range(0,1000):        
            for t in tfList:
                t.header.stamp = rospy.Time.now()
                self.br.sendTransform(t)                
            rate.sleep()

    def broadcast_single_tf(self, t):
        rate = rospy.Rate(1000)        
        for time in range(0,50):
            t.header.stamp = rospy.Time.now()
            self.br.sendTransform(t)                
            rate.sleep()

    def generate_tf(self, parent, child, position, rotation):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation = position #needs to be postion tuple
        t.transform.rotation = rotation #needs to be quat tuple  
        self.tfList.append(t)
        return t

    #def construct_matrix(self, trans, rot):
        transform = numpy.matrix([[rot.item(0,0), rot.item(0,1), rot.item(0,2), trans.item(0,3)],
                            [rot.item(1,0), rot.item(1,1), rot.item(1,2), trans.item(1,3)],
                            [rot.item(2,0), rot.item(2,1), rot.item(2,2), trans.item(2,3)],
                            [0, 0, 0, 1]])
        return transform

    def generate_rotation_matrix(self, theta): 
        # Generate matrix to rotation about the z-axis of shelf frame to get bunch of new transforms to use for projection
        transform = numpy.array([[numpy.cos(theta), numpy.sin(theta), 0, 0],
                                [-numpy.sin(theta), numpy.cos(theta), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
        return transform 

    #def generate_translation_matrix(self, point): 
        # Generate matrix to rotation about the z-axis of shelf frame to get bunch of new transforms to use for projection
        transform = numpy.array([[1, 0, 0, point.item(0)],
                                [0, 1, 0, point.item(1)],
                                [0, 0, 1, point.item(2)],
                                [0, 0, 0, 1]])
        return transform

    def get_object_extents(self, req):
        # Select object
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
        #self.size = numpy.array(size)
        #self.item = req.item

        #return self.item, self.size
        return numpy.array(size)

    def generate_apc_grasp_poses(self, projectionList, approachList):  
        grasps = []
        for i in range(len(approachList)):
            projection = projectionList[i]
            approach = approachList[i]
            grasp = apcGraspPose()
            grasp.posegrasp.position = projection.position         
            grasp.posegrasp.orientation = projection.orientation
            grasp.poseapproach.position = approach.position         
            grasp.poseapproach.orientation = approach.orientation
            grasps.append(grasp)
          
        return grasps

    def get_quaternion(self, matrix):
        qw = numpy.sqrt(1 + matrix.item(0,0) + matrix.item(1,1) + matrix.item(2,2))/2
        qx = (matrix.item(2,1) - matrix.item(1,2)) / (4*qw)
        qy = (matrix.item(0,2) - matrix.item(2,0)) / (4*qw)
        qz = (matrix.item(1,0) - matrix.item(0,1)) / (4*qw)
        quatVector = numpy.array([qx, qy, qz, qw])
        return quatVector
        
    def get_obb_points(self, size):
        # do a service call to get min max of object. Or maybe a subscribe to topic is fine too since all services start when
        # the robot launches. Alex code is requesting sample and process service so maybe when those are call just publish the min max values to
        # a topic. May be the easiest way
        x = size.item(0)
        y = size.item(1)
        z = size.item(2)

        point1 = (x/2, y/2, z/2)
        point2 = (x/2, -y/2, z/2)
        point3 = (-x/2, y/2, z/2)
        point4 = (-x/2, -y/2, z/2)
        point5 = (x/2, y/2, -z/2)
        point6 = (x/2, -y/2, -z/2)
        point7 = (-x/2, y/2, -z/2)
        point8 = (-x/2, -y/2, -z/2)
        points = numpy.array([point1, point2, point3, point4, point5, point6, point7, point8])
        """

        delta = numpy.array([maxx-minx maxy-miny maxz-minz]) #return array vector that contains the delta for x, y, and z axis of bounding box.
        """
        return points

    def compute_width(self, min_y, max_y):
        width = abs(max_y-min_y)
        return width

    def compute_y_mid(self, miny, maxy, Tbaseproj):     
        y = (miny+maxy)/2
        ymid = numpy.matrix([[0],[y],[0],[1]])
        Tmidpt = Tbaseproj*ymid #y mid wrt to proj frame is now wrt to base
        mid_y = numpy.array([Tmidpt.item(0,0), Tmidpt.item(1,0), Tmidpt.item(2,0)])
        return mid_y

    def check_width(self, width):        
        if width <= self.gripperwidth: 
            isSmaller = True
        else:
            isSmaller = False
        return isSmaller

    def compute_score(self, width):
        score = width/self.gripperwidth
        return score

    def compute_minmax(self, pointList):
        #Loop through pointList to find the min & max for the x, y, and z
        min_x = 0
        min_y = 0
        min_z = 0
        max_x = 0
        max_y = 0
        max_z = 0
        for point in pointList:
            if point[0] > max_x: #if x point is larger than current max x then replace with new one
                max_x = point[0]
            if point[0] < min_x: #if x point is smaller than current min x then replace with new one
                min_x = point[0]
            if point[1] > max_y: #if y point is larger than current max y then replace with new one
                max_y = point[1]
            if point[1] < min_y: #if y point is smaller than current min y then replace with new one
                min_y = point[1]
            if point[2] > max_z: #if z point is larger than current max z then replace with new one
                max_z = point[2]
            if point[2] < min_z: #if z point is smaller than current min z then replace with new one
                min_z = point[2]
        min_max = numpy.array([min_x, max_x, min_y, max_y, min_z, max_z])
        return min_max                    

    #def compute_approach_tf(self, Tbaseobj):
        xDirVector = numpy.matrix([1],[0],[0],[1])
        approachx = numpy.matrix([[Tbaseproj.item(0,0)], [Tbaseproj.item(1,0)], [Tbaseproj.item(2,0)], [1] ])
        return approachx

    def get_grasp_cb(self, req):

        poseList = []
        approachList = []
        projectionList = []
        tfs = []
        countbad = 0
        print "Requesting valid grasps for %s" %req.item
    
        # Request the 8 Oriented bounding box points wrt to the object's frame.
        size = self.get_object_extents(req)
        OBBPoints = self.get_obb_points(size)

        # Get TFs
        Tbaseshelf = self.get_tf('/base_link', '/shelf')
        Tshelfobj = self.get_tf('/shelf', '/object')
        if self.showOutput:
            print OBBPoints
            print Tbaseshelf
            print Tshelfobj                 

        # Generate TFs to project onto
        for theta in self.thetaList:            
            Tshelfproj = self.generate_rotation_matrix(theta)
            Tbaseproj = Tbaseshelf*Tshelfproj
            if theta == 0:
                print "projecting to shelf frame"
             
            if self.showOutput:
                print "Transform from shelf to proj"
                print Tshelfproj, type(Tshelfproj)
                print "Transform from base to proj"
                print Tbaseproj, type(Tbaseproj)

            # Generate TF for shelf to projection
            proj = geometry_msgs.msg.Pose()
            [transobj, quatobj] = ExtractFromMatrix(Tshelfobj)
            [transproj, quatproj] = ExtractFromMatrix(Tshelfproj)
            proj.position.x = transobj[0]
            proj.position.y = transobj[1]
            proj.position.z = transobj[2]            
            proj.orientation.x = quatproj[0]
            proj.orientation.y = quatproj[1]
            proj.orientation.z = quatproj[2]
            proj.orientation.w = quatproj[3]
            poseList.append(proj)
            tf_shelfproj = self.generate_tf('/shelf', '/projection', proj.position, proj.orientation)
            self.broadcast_single_tf(tf_shelfproj)

            # Get TF for projection to object
            tf_projobj = self.get_tf('/projection', "/object")
            if self.showOutput:
                print "Transform from projection to object"
                print tf_projobj

            # Loop through OBB points and transform them to be wrt to the target projection frame
            for OBBPoint in OBBPoints:
                OBBPointsList = []
                OBBPoint = numpy.matrix([[OBBPoint[0]], [OBBPoint[1]], [OBBPoint[2]], [1]])
                projected_OBBPoint = tf_projobj*OBBPoint
                OBBPointsList.append(projected_OBBPoint) #Pass in Tbaseproj
            if self.showOutput:
                print "List of transformed OBB points after projection to target projection frame"
                print OBBPointsList

            # Get min max points. Pass in transformed OBBPoints list to get min max for target frame
            # Compute width of projection shadow. Width is the y axis because shelf frame is set that way with y axis as width
            # Check if width of shadow projection can fit inside gripper width
            min_max = self.compute_minmax(OBBPointsList)
            min_x, max_x, min_y, max_y, min_z, max_z = min_max         
            width = self.compute_width(min_y, max_y)
            if self.showOutput:
                print "Min-max values [minx,maxx,miny,maxy,minz,maxz]: ", min_x, max_x, min_y, max_y, min_z, max_z
                print "projection width: " + str(width)
                print "min_y: " + str(min_y)
                print "max_y: " + str(max_y)            
            isSmaller = self.check_width(width)

            # Compute cost
            if isSmaller == True:
                score = self.compute_score(width)
                
                # Compute mid-y point using miny maxy 
                mid_y = self.compute_y_mid(min_y, max_y, Tbaseproj)
                if self.showOutput:
                    print "midpt: "+str(mid_y), mid_y[1]

                # Update projection TF with new y value set to be middle of the projection wrt to the projection frame
                proj = deepcopy(proj)
                proj.position.y = mid_y[1]
                
                tf_shelfproj = self.generate_tf('/shelf', '/projection', proj.position, proj.orientation)
                self.broadcast_single_tf(tf_shelfproj)

                # Generate TF for projection to approach pose
                approach = geometry_msgs.msg.Pose()
                approach.position.x = -0.2 #20 cm
                approach.position.y = 0
                approach.position.z = 0            
                approach.orientation.x = 0
                approach.orientation.y = 0
                approach.orientation.z = 0
                approach.orientation.w = 1
                
                tf_projapproach = self.generate_tf('/projection', '/approach', approach.position, approach.orientation)
                self.broadcast_single_tf(tf_projapproach)

                # Get projection and approach TF wrt to the base frame and append to list
                # self.get_tf('/base_link',"/projection")
                # self.get_tf('/base_link',"/approach")
                proj_msg = self.get_tf_msg('/base_link', "/projection")
                approach_msg = self.get_tf_msg('/base_link', "/approach")
                projectionList.append(proj_msg)
                approachList.append(approach_msg)
                if self.showOutput:
                    print "score is %i. Good approach direction. Gripper is wide enough" %score
            else:
                poseList.pop() #Removed the pose that has bad cost. projection width doesn't fit in gripper
                cost = self.compute_cost(width)
                if self.showOutput:
                    print "score is %i. Bad approach direction. Gripper not wide enough" %score
                countbad += countbad
        if self.showOutput:
            print "number of bad approach directions: " + str(countbad)

        """
        # Generate TF for projection to approach pose
        for i in range(len(approachList)):
            position = approachList[i].position
            orientation = approachList[i].orientation
            t = self.generate_tf('/projection', 'approach', position, orientation)
            tfs.append(t)
        print tfs
        self.broadcast_tf(tfs)
        """

        # Temp grasp messageto return
        graspList = self.generate_apc_grasp_poses(projectionList, approachList)
        grasps = apcGraspArray(
                grasps = graspList
            )


        # From center of palm to edge of it is 6cm then lip is about 2cm up so need to offset robot z-axis to move hand approach vector to be 8cm
        # up from bottom of object.        
        print "Request complete-----------------------------------------------------------------------------------------------------"
        return apcGraspDBResponse(status=True,apcGraspArray=grasps)
    
def publisher():
    rospy.init_node('online_grasp_server')

    grasp = grasping()
    while not rospy.is_shutdown():
        s = rospy.Service('getGrasps_online_server', apcGraspDB, grasp.get_grasp_cb)
        print "Request start--------------------------------------------------------------------------------------------------------"
        print "Online grasp planner ready"
        rospy.spin()

if __name__ == '__main__':
    publisher()



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

"""


# If width is smaller than compute the approach vector to grasp center of projection
if isSmaller == True:
    approach = compute_approach_tf()
else:
    fail = True
    print "Unable to grasp object"

# Construct the poseApproach msg and return just like in offline planner
"""

"""

####################################################
# Start loop for iterating through everything 
####################################################
failedTFList = []
newprojList = []
approachList = []  
if self.showOutput:
    print "#######################################################"
    print "this is when the looping through projections section"
    print "#######################################################"
# Generate more transform frames to project to
for theta in self.thetaList:
    Tshelfproj = self.generate_rotation_matrix(theta)
    Tbaseproj = Tbaseshelf*Tshelfproj
    if theta == 0:
        Tbaseproj = Tbaseshelf
    newprojList.append(Tbaseproj)
     
    if self.showOutput:
        print "Transform from shelf to proj"
        print Tshelfproj
        print "Generate list of projection transforms"
        print newprojList

# Generate and publish TF for each of the projection planes
print "Generating TF for projection frames"
for i in range(len(newprojList)):
    Tproj = newprojList[i]
    pose = geometry_msgs.msg.Pose()
    posex = pose.position.x = Tproj.item(0,3)
    posey = pose.position.y = Tproj.item(1,3)
    posez = pose.position.z = Tproj.item(2,3)
    quat = self.get_quaternion(Tproj)
    quatx = pose.orientation.x = quat.item(0)
    quaty = pose.orientation.y = quat.item(1)
    quatz = pose.orientation.z = quat.item(2)
    quatw = pose.orientation.w = quat.item(3)
    print "angle: " + str(quatw)
    if math.isnan(float(quatw)) == False and math.isnan(float(quatx)) == False and math.isnan(float(quaty)) == False and math.isnan(float(quatz)) == False and math.isnan(float(posex)) == False and math.isnan(float(posey)) == False and math.isnan(float(posez)) == False:
            projectionList.append(Tproj) # List of new frames to probject OBB points to.
    else:
        print "Nan/inf value detected. Not appending projction TF to list"
        failedTFList.append(i)
    Tbaseproj = self.generate_tf('/base_link', "/projection"+str(i), pose.position, pose.orientation) # Transform from base to proj frame
self.broadcast_tf()

if self.showOutput:
    print "--------------------------------------------------failedTFList"
    print failedTFList

for i in range(0,len(projectionList)-len(failedTFList)): 
    pointsList = [] #Reset points list
    print len(failedTFList)
    
    # Get transform from 
    print '/projectionList'+str(i)
    # Construct the 4x4 Transformation Matrix
    if i not in failedTFList: #if i is not index ins failed list then make transforms, else skip it cause of nan
        # Tprojobj = self.get_tf('/projection'+str(i), '/object')
        Tprojobj = self.get_tf('/projection'+str(i), '/object')
        Tbaseproj = self.get_tf('/base_link', '/projection'+str(i))
        Tbaseobj = self.get_tf('/base_link', '/object')
        if self.showOutput:
            print Tprojobj        
    else:
        print "skipped /projectionList"+str(i)           

    # Loop through OBB points and transform them to be wrt to the target projection frame
    for OBBPoint in OBBPoints:
        if self.showOutput:
            print OBBPoint
        OBBPoint = numpy.array([[OBBPoint.item(0)], [OBBPoint.item(1)], [OBBPoint.item(2)], [1]])
        pointsList.append(Tprojobj*OBBPoint)
    if self.showOutput:
        print "pointList after transform to target frame: %s"%('/projection'+str(i))
        print pointsList

    # Get min max points. Pass in transformed OBBPoints list to get min max for target frame
    min_max = self.compute_minmax(pointsList)

    # Compute width of projection shadow. Width is the y axis because shelf frame is set that way with y axis as width
    min_y = min_max.item(2)
    max_y = min_max.item(3)
    width = self.compute_width(min_y, max_y)
    if self.showOutput:
        print "width: " + str(width)
        print "min_y: " + str(min_y)
        print "max_y: " + str(max_y)

    # Compute midpt using miny maxy points. Midpoint wrt to current projection frame
    minpt = numpy.array([min_max.item(0), min_max.item(2), min_max.item(4)])
    maxpt = numpy.array([min_max.item(1), min_max.item(3), min_max.item(5)]) 
    midpt = self.compute_y_mid(minpt, maxpt)

    # Transform midpt to be wrt to the object frame


    # Check if width of shadow projection can fit inside gripper width
    isSmaller = self.check_width(width)

    # Generate and publish TF for each of the projection planes            
    
    pose = geometry_msgs.msg.Pose()
    posex = pose.position.x = Tproj.item(0,3)+midpt.item(0)
    posey = pose.position.y = midpt.item(1)
    posez = pose.position.z = Tproj.item(2,3)+midpt.item(2)
    quat = self.get_quaternion(Tbaseproj)
    quatx = pose.orientation.x = quat.item(0)
    quaty = pose.orientation.y = quat.item(1)
    quatz = pose.orientation.z = quat.item(2)
    quatw = pose.orientation.w = quat.item(3)
    self.generate_tf('/base_link', "/projection"+str(i), pose.position, pose.orientation) # Transform from base to proj frame
    
    # Compute cost
    if isSmaller == True:
        cost = self.compute_cost(width)
        approach = self.compute_approach_tf(Tbaseobj*Tprojobj) # Pass in Transform_baseobj x-vector of it is the approach I want to use 
        approachList.append(approach)

        if self.showOutput:
            print "approach vector"
            print approach
            print "valid approach cost: " + str(cost)
    else:
        cost = self.compute_cost(width)
        print "Cost is %i. Bad approach direction. Gripper not wide enough" %cost
self.broadcast_tf()
"""

###################################################
# end of loop of iterating everythig
###################################################