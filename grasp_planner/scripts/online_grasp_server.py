#!/usr/bin/env python

import datetime
import os, random
import cPickle as pickle
import Queue as Q
import tf
import tf2_ros
import rospy
import numpy
import traceback
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from copy import deepcopy

from math import pi
from numpy.linalg import inv
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from grasp_planner.msg import apcGraspPose, apcGraspArray
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
from apc_util.transformation_helpers import ExtractFromMatrix, BuildMatrix, PoseFromMatrix, PoseToMatrix, InvertMatrix
# from transformation_helper import ExtractFromMatrix, BuildMatrix, PoseFromMatrix, PoseToMatrix, InvertMatrix


class Grasping:

    def __init__(self):
        self.description = "Online grasp planning code"
        self.status = True
        self.tf = tf.TransformListener(True, rospy.Duration(10.0))
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(60.0)
        self.tfList = []

        # Adjustable variables in planner
        # self.pitchList = numpy.linspace(0, pi/12, num=3)
        #self.pitchList = [pi/12]
        self.pitchList = [0]
        self.thetaList = numpy.linspace(-pi/6, pi/6, num=101) # Rotation of generated projection frames
        # self.thetaList = [0]
        self.padding = 0.015  # Extra padding between object and gripper is 1 cm.
        self.fingerlength = 0.115  # palm to finger tip offset is 11.5 cm
        self.gripperwidth = 0.155 - self.padding  # gripper width is 15.5 cm
        self.z_lowerboundoffset = 0.065 #- 0.02  # Palm center to bottom of hand is 6.5 cm
        self.approachpose_offset = 0.3  # Set aproach pose to be 30cm back from the front of the bin
        # palm -15 deg offset about z-axis
        self.hand_theta = 0.261799
        self.shelfpitch = pi/12
        # Transform rotation from tooltip to hand's palm. If no transforms then it grabs object at a slant
        self.Rtooltip_palm = numpy.array([[1, 0, 0, 0],
                                    [0, numpy.cos(self.hand_theta), -numpy.sin(self.hand_theta), 0],
                                    [0, numpy.sin(self.hand_theta), numpy.cos(self.hand_theta), 0],
                                    [0, 0, 0, 1]])
        # Transform to orient hand to use x as approach direction
        self.Ttooltip_grasp = numpy.array([[0, 0, -1, -0.17],
                                     [-1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 0, 1]])

        rospy.logdebug("theta range list")
        rospy.logdebug(str(self.thetaList))

        rospy.sleep(rospy.Duration(1.0))  # Wait for network timing to load TFs

    def get_tf(self, parent, child):
        rospy.logdebug("Looking up TF transform from %s to %s", parent, child)
        try:
            (trans, quat) = self.tf.lookupTransform(parent, child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to lookup TF transform from source:%s to target:%s", parent, child)
            self.status = False
        trans = numpy.asarray(trans)
        quat = numpy.asarray(quat)
        translation = [trans.item(0), trans.item(1), trans.item(2)]
        quaternion = [quat.item(0), quat.item(1), quat.item(2), quat.item(3)]
        transform = BuildMatrix(translation, quaternion)
        return transform

    def broadcast_single_tf(self, t):
        rate = rospy.Rate(1000)
        for time in range(0, 10):
            t.header.stamp = rospy.Time.now()
            self.br.sendTransform(t)
            rate.sleep()

    def generate_tf(self, parent, child, transform):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        [trans, quat] = ExtractFromMatrix(transform)
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.broadcast_single_tf(t)

    def construct_4Dmatrix(self, trans, rot):
        # print trans, rot
        transform = numpy.zeros([4, 4])
        transform[0:3, 0:3] = rot
        transform[0:3, 3] = trans
        transform[3, 3] = 1
        return transform

    def generate_rotation_matrix(self, theta, pitch):
        # Generate matrix to rotation about the z-axis of shelf frame to get bunch of new transforms to use for projection
        rotation = numpy.array([[numpy.cos(theta), numpy.sin(theta), 0],
                                [-numpy.sin(theta), numpy.cos(theta), 0],
                                [0, 0, 1]])
        pitch = numpy.array([[numpy.cos(pitch), 0, numpy.sin(pitch)],
                                        [0, 1, 0],
                                        [-numpy.sin(pitch), 0, numpy.cos(pitch)]])
        transform = numpy.dot(rotation, pitch)
        return transform

    def get_shelf_bounds(self, req):
        try:
            bin_bounds = rospy.get_param("/shelf/bins/"+req.bin)
        except:
            rospy.logerr("could not find rosparam for bin: %s", req.bin)
            print traceback.format_exc()
            rospy.logerr(str(traceback.format_exc()))
            self.status = False
        return bin_bounds

    def get_object_height_offset(self, req):
        if req.item == 'cheezit_big_original':
            item = '../env/cheezit.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'kyjen_squeakin_eggs_plush_puppies':
            item = '../env/colorballs.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'crayola_64_ct':
            item = '../env/crayon.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'feline_greenies_dental_treats':
            item = '../env/dentaltreat.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'expo_dry_erase_board_eraser':
            item = '../env/eraser.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'elmers_washable_no_run_school_glue':
            item = '../env/glue.env.xml'
            self.objectheightoffset = -0.05
        elif req.item == 'sharpie_accent_tank_style_highlighters':
            item = '../env/highlighters.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'mark_twain_huckleberry_finn':
            item = '../env/huckfinn.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'mead_index_cards':
            item = '../env/indexcards.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'oreo_mega_stuf':
            item = '../env/oreo.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'mommys_helper_outlet_plugs':
            item = '../env/outletplugs.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'paper_mate_12_count_mirado_black_warrior':
            item = '../env/pencil.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'rolodex_jumbo_pencil_cup':
            item = '../env/pencilcup.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'kong_duck_dog_toy':
            item = '../env/plushduck.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'kong_sitting_frog_dog_toy':
            item = '../env/plushfrog.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'munchkin_white_hot_duck_bath_toy':
            item = '../env/rubberduck.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'safety_works_safety_glasses':
            item = '../env/safetyglasses.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'stanley_66_052':
            item = '../env/screwdrivers.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'champion_copper_plus_spark_plug':
            item = '../env/sparkplug.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'highland_6539_self_stick_notes':
            item = '../env/stickynotes.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'genuine_joe_plastic_stir_sticks':
            item = '../env/stirsticks.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'first_years_take_and_toss_straw_cup':
            item = '../env/strawcups.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'kong_air_dog_squeakair_tennis_ball':
            item = '../env/tennisball.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'laugh_out_loud_joke_book':
            item = '../env/huckfinn.env.xml'
            self.objectheightoffset = 0.0
        elif req.item == 'dr_browns_bottle_brush':
            item = '../env/huckfinn.env.xml'
            self.objectheightoffset = 0.0
        else:
            rospy.logerr("could not find scene xml for object: %s", req.item)
            self.status = False

    def generate_apc_grasp_poses(self, projectionList, approachList):
        grasps = []
        for i in range(len(approachList)):
            projection = projectionList[i]
            approach = approachList[i]
            grasp = apcGraspPose()
            grasp.pregrasp.position = projection.position
            grasp.pregrasp.orientation = projection.orientation
            grasp.approach.position = approach.position
            grasp.approach.orientation = approach.orientation
            grasps.append(grasp)
        return grasps

    def compute_width(self, min_y, max_y):
        return abs(max_y-min_y)

    def compute_depth(self, min_x, max_x):
        obj_depth = abs(max_x-min_x)
        edge_offset = min_x
        extensions = 0.15 # 15cm finger extensions
        offset = numpy.true_divide(obj_depth, 2)
        if offset > self.fingerlength + extensions:
            offset = 0.10
        if offset < 0.10:
            offset = 0.10
        return edge_offset - self.fingerlength - extensions + offset # the palm is located at min_x so move out till lenght of finger to place lenght of finger at min_x. Then move in 1/4 of the total depth of object
        #return edge_offset - self.fingerlength - extensions

    def compute_height(self, bin_min_z):
        return bin_min_z + self.z_lowerboundoffset + self.objectheightoffset  # minus 5cm height as magic number adjustment. Should have to do this if binmin z is correct
        #return bin_min_z + self.z_lowerboundoffset

    def compute_approach_offset(self, projection_x, bin_min_x, theta):
        dist = projection_x - bin_min_x
        offset = (dist - self.approachpose_offset) * numpy.cos(abs(theta))
        # return dist - self.approachpose_offset
        return bin_min_x - 0.20

    def compute_score(self, width, pitch):
        fscore = numpy.true_divide(width, self.gripperwidth)
        gscore = abs(pitch)
        weight = 0.6
        score = (1-weight)*fscore - weight*gscore
        if score < 0:
            score = 0
            rospy.logerr("WARNING! Score is negative. Setting score to zero. Consider adjusting weight bias.")
        rospy.logdebug("%s = 0.5*%s + 0.5*%s", score, fscore, gscore)
        return score

    def compute_minmax(self, pointList):
        x = []
        y = []
        z = []
        for point in pointList:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        min_x = min(x)
        max_x = max(x)
        min_y = min(y)
        max_y = max(y)
        min_z = min(z)
        max_z = max(z)
        min_max = numpy.array([min_x, max_x, min_y, max_y, min_z, max_z])
        return min_max

    def compute_minmax_filtered(self, pointList):
        x = []
        y = []
        z = []
        for point in pointList:
            # if point[0] > 0:
            #     continue
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        min_x = min(x)
        max_x = max(x)
        min_y = min(y)
        max_y = max(y)
        min_z = min(z)
        max_z = max(z)
        min_max = numpy.array([min_x, max_x, min_y, max_y, min_z, max_z])
        return min_max

    def checkquaternion(self, transform, name):
        approach = geometry_msgs.msg.Pose()
        [trans, quat] = ExtractFromMatrix(transform)
        approach.position.x = trans[0]
        approach.position.y = trans[1]
        approach.position.z = trans[2]
        approach.orientation.x = quat[0]
        approach.orientation.y = quat[1]
        approach.orientation.z = quat[2]
        approach.orientation.w = quat[3]
        unit_quat = str((approach.orientation.x)**2 + (approach.orientation.y)**2 + (approach.orientation.z)**2 + (approach.orientation.w)**2)
        if unit_quat != 1:
            rospy.logerr(name + " is unit quaternion: " + str(unit_quat))
            self.status = False

    def compute_jaw_separation(self, width):
        padding = 0.02 # 2cm padding. 1cm on each side of jaw
        gpos = (-6.245 * width + 109.06) + padding
        return gpos

    def save_request(self, req):
        rospy.loginfo("Saving request msg to file")
        now = datetime.datetime.now()
        timenow = now.strftime("%Y-%m-%d_%H:%M")
        folderpath = os.path.join(os.path.dirname(__file__), "savedmsg")
        filename = "bin"+req.bin+"_"+req.item+"_"+str(timenow)
        with open(folderpath + "/" + filename, "w") as file:
            pickle.dump(req, file)

    def load_request(self):
        rospy.loginfo("Loading request msg from file")
        folderpath = os.path.join(os.path.dirname(__file__), "savedmsg")
        filename = random.choice(os.listdir(folderpath))  # Load random file from directory
        with open(folderpath + "/" + filename, 'rb') as file:
            request = pickle.load(file)
        return request

    def get_grasp_cb(self, req):
        # Dump request to file
        # self.save_request(req)
                
        q_proj_msg = Q.PriorityQueue()
        q_approach_msg = Q.PriorityQueue()
        approachList = []
        projectionList = []
        countbad = 0

        rospy.logdebug( "************************************************** Request start **************************************************")
        rospy.logdebug("Requesting valid grasps for %s", req.item)
        rospy.logdebug("Requesting object in bin %s", req.bin)

        self.get_object_height_offset(req)
        bin_bounds = self.get_shelf_bounds(req)
        bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z = bin_bounds

        rospy.logdebug( "bin min z: " + str(bin_min_z))

        # use point cloud
        Tbaseobj = PoseToMatrix(req.object_pose)  # same Trob_obj request from offline planner
        Tbaseshelf = PoseToMatrix(req.shelf_pose) # Tbaseshelf transform passed in from state matchine
        Tshelfobj = numpy.dot(inv(Tbaseshelf), Tbaseobj)
        pointcloud = list(pc2.read_points(req.object_points, skip_nans=True,
                          field_names=("x", "y", "z")))

        rospy.logdebug("OBBPoints: " + str(pointcloud))
        rospy.logdebug("Transform from base to shelf: " + str(Tbaseshelf))
        rospy.logdebug("Transform from shelf to obj: " + str(Tshelfobj))

        rospy.logdebug("********************************************** Start loop ********************************************")

        # Generate TFs to project onto
        for pitch in self.pitchList:
            for theta in self.thetaList:

                # Construction projection frames
                Rot_shelfproj = self.generate_rotation_matrix(theta, pitch)
                # Rot_shelfproj_new = numpy.dot(Rot_shelfproj, self.Tshelfpitch)
                Trans_shelfobj = numpy.array([Tshelfobj[0, 3], Tshelfobj[1, 3], bin_min_z])
                # Trans_shelfobj = Tshelfobj[0:3,3]
                Tshelfproj = self.construct_4Dmatrix(Trans_shelfobj, Rot_shelfproj)
                # Tshelfproj = self.construct_4Dmatrix(Trans_shelfobj, Rot_shelfproj_new)

                rospy.logdebug(str(Tshelfproj))

                # Transform from projection to object
                Tprojshelf = inv(Tshelfproj)

                # Loop through object points in shelf frame and transform them to be wrt to the target projection frame 
                points = []
                for point in pointcloud:
                    oldpt = numpy.array([[point[0]], [point[1]], [point[2]], [1]])
                    projected_point = numpy.dot(Tprojshelf, oldpt)
                    points.append(projected_point)
                if not points:
                    rospy.logerr("No points in poincloud!!!")
                    return apcGraspDBResponse(status=False)


                rospy.logdebug("List of transformed object points after projection to target projection frame")
                rospy.logdebug(str(len(points)))

                # Get min max points. Pass in transformed points list to get min max for target frame. Compute width of projection shadow. Width is the y axis because shelf frame is set that way with y axis as width. Check if width of shadow projection can fit inside gripper width
                min_max = self.compute_minmax(points)
                min_x, max_x, min_y, max_y, min_z, max_z = min_max
              

                # filtered_min_max = self.compute_minmax_filtered(points)
                # f_min_x, f_max_x, f_min_y, f_max_y, f_min_z, f_max_z = filtered_min_max
                # width = self.compute_width(f_min_y, f_max_y)
                width = self.compute_width(min_y, max_y)

                jaw_separation = self.compute_jaw_separation(width)

                rospy.logdebug("Min-max values [minx,maxx,miny,maxy,minz,maxz]: " + str(min_max))
                rospy.logdebug("projection width: " + str(width))
                rospy.logdebug("min_y: " + str(min_y))
                rospy.logdebug("max_y: " + str(max_y))

                # Get hand pose
                if width <= self.gripperwidth:
                    score = self.compute_score(width, pitch)

                    grasp_depth = self.compute_depth(min_x, max_x)  # set how far hand should go past front edge of object
                    height = self.compute_height(bin_min_z)  # select the height so bottom of object and also hand won't collide wit shelf lip. may need to take into acount the max_z and objects height to see if object will hit top of shelf.
                    approach_offset = self.compute_approach_offset(Trans_shelfobj[0], bin_min_x, theta)

                    rospy.logdebug("score: "+str(score))
                    rospy.logdebug("x grasp depth value "+str(grasp_depth))
                    rospy.logdebug("x height value "+str(height))

                    # Transform from projection to pregrasp pose
                    Trans_projpregrasp = numpy.array([grasp_depth, 0, 0])

                    Rot_projpregrasp = numpy.eye(3, 3)
                    Tprojpregrasp = self.construct_4Dmatrix(Trans_projpregrasp, Rot_projpregrasp)
                    Tshelfpregrasp = numpy.dot(Tshelfproj, Tprojpregrasp)

                    # Transform from pregrasp to approach pose
                    # Trans_pregraspapproach = numpy.array([-approach_offset, 0, 0])
                    Trans_pregraspapproach = numpy.array([-self.approachpose_offset, 0, 0])
                    Rot_pregraspapproach = numpy.eye(3, 3)
                    Tpregraspapproach = self.construct_4Dmatrix(Trans_pregraspapproach, Rot_pregraspapproach)

                    # Generate and display TF in Rviz
                    self.generate_tf('/shelf', '/pregrasp', Tshelfpregrasp)
                    self.generate_tf('/pregrasp', '/approach', Tpregraspapproach)

                    # Transform of grasp wrt to camera frame. From toollink which has approach backwards and using Z-axis to using x for the approach.
                    TgraspIK = numpy.dot(self.Rtooltip_palm, self.Ttooltip_grasp)

                    # Transform from arm solved from IK to handpose for pregrasp
                    Tbasepregrasp = numpy.dot(Tbaseshelf, Tshelfpregrasp)
                    TbaseIK_pregrasp = numpy.dot(Tbasepregrasp, TgraspIK)

                    # TbaseIK_pregrasp = numpy.dot(numpy.dot(Tbasepregrasp, TgraspIK), Rotz)


                    # Transform from arm solved from IK to handpose for approach
                    Tshelfapproach = numpy.dot(Tshelfpregrasp, Tpregraspapproach)
                    Tbaseapproach = numpy.dot(Tbaseshelf, Tshelfapproach)
                    TbaseIK_approach = numpy.dot(Tbaseapproach, TgraspIK)

                    # TbaseIK_approach = numpy.dot(numpy.dot(Tbaseapproach, TgraspIK), Rotz)

                    # Construct msg. Then appened to queue with score as the priority in queue. This will put lowest score msg first in list.
                    proj_msg = PoseFromMatrix(TbaseIK_pregrasp)
                    pregrasp_height_extra = abs(grasp_depth) * numpy.sin(pitch)
                    proj_msg.position.z = height + pregrasp_height_extra
                    approach_msg = PoseFromMatrix(TbaseIK_approach)
                    approach_height_extra = abs(self.approachpose_offset) * numpy.sin(pitch)
                    approach_msg.position.z = height + pregrasp_height_extra + approach_height_extra
                    q_proj_msg.put((score, proj_msg))
                    q_approach_msg.put((score, approach_msg))

                    rospy.logdebug("score is %f. Good approach direction. Gripper is wide enough", score)
                else:
                    score = self.compute_score(width)
                    rospy.logdebug("score is %f. Bad approach direction. Gripper not wide enough", score)
                    countbad += 1

        rospy.logdebug("number of bad approach directions: " + str(countbad))

        while not q_proj_msg.empty():
            projectionList.append(q_proj_msg.get()[1])
        while not q_approach_msg.empty():
            approachList.append(q_approach_msg.get()[1])

        # Construct APC grasp message to return
        graspList = self.generate_apc_grasp_poses(projectionList, approachList)
        grasps = apcGraspArray(
            grasps=graspList
        )
        rospy.logdebug("************************************************** end loop **************************************************")
        rospy.logdebug("************************************************** Request end ***********************************************")
        return apcGraspDBResponse(status=self.status, grasps=grasps, preshape = jaw_separation)


def publisher():
    # rospy.init_node('online_grasp_server', log_level=rospy.DEBUG)
    rospy.init_node('online_grasp_server')

    grasp = Grasping()
    while not rospy.is_shutdown():
        try:
            rospy.Service('getGrasps_online_server', apcGraspDB, grasp.get_grasp_cb)
            print "Online grasp planner ready"
            rospy.spin()
        except:
            print traceback.format_exc()
            rospy.logerr(str(traceback.format_exc()))

if __name__ == '__main__':
    publisher()
