#!/usr/bin/env python

import roslib
import Queue as Q
import tf
import tf2_ros
import rospy
import numpy
from numpy.linalg import inv
import traceback
import geometry_msgs.msg
from math import pi

# Transformation helper file. quater is (x,y,z,w) format
from transformation_helper import ExtractFromMatrix, BuildMatrix, PoseFromMatrix, PoseToMatrix, InvertMatrix

# Message and Service imports
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from grasp_planner.msg import apcGraspPose, apcGraspArray
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse


class grasping:

    def __init__(self):
        self.description = "Online grasp planning code"
        self.offset = 0.01  # safety buffer between object and gripper is 1cm on each side. Total of 2cm
        self.fingerlength = 0.11  # palm to finger tip offset is 11cm
        self.z_lowerboundoffset = 0.08  # move gripper up 1.5 cm to avoid lip when going straigh in
        self.z_upperboundoffset = 0.18
        self.gripperwidth = 0.155 - self.offset  # 0.155 is the gripper width in meters. 15.5cm        
        self.x_upperboundoffset = 0.04  # fingertip is allowed move at most 4cm beyond the lower bound value
        self.x_lowerboundoffset = 0.1  # fingertip goes past nearest point cloud's x value by at least 4 cm 
        self.showOutput = False  # Enable to show print statements
        self.tf = tf.TransformListener(True, rospy.Duration(10.0))
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(60.0)
        self.tfList = []
        # self.thetaList = numpy.linspace(-0.698131701, 0.698131701, num=41)
        self.thetaList = numpy.linspace(-pi/2, pi/2, num=51)
        # self.thetaList = numpy.linspace(-pi, pi, num=51)

        if self.showOutput:
            print "theta range list"
            print self.thetaList

        rospy.sleep(rospy.Duration(1.0))  # Wait for network timing to load TFs

    def get_tf(self, parent, child):
        if self.showOutput:
            print "Looking up TF transform from %s to %s" % (parent, child)
        try:
            (trans, quat) = self.tf.lookupTransform(parent, child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Failed to lookup TF transform from source:%s to target:%s" % (parent, child)
        trans = numpy.asarray(trans)
        quat = numpy.asarray(quat)
        translation = [trans.item(0), trans.item(1), trans.item(2)]
        quaternion = [quat.item(0), quat.item(1), quat.item(2), quat.item(3)]
        transform = BuildMatrix(translation, quaternion)
        return transform

    def get_tf_msg(self, parent, child):
        if self.showOutput:
            print "Looking up TF transform from %s to %s" % (parent, child)
        try:
            (trans, quat) = self.tf.lookupTransform(parent, child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Failed to lookup TF transform from %s to %s" % (parent, child)
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
        for time in range(0, 1000):
            for t in tfList:
                t.header.stamp = rospy.Time.now()
                self.br.sendTransform(t)
            rate.sleep()

    def broadcast_single_tf(self, t):
        rate = rospy.Rate(1000)
        for time in range(0, 10):
            t.header.stamp = rospy.Time.now()
            self.br.sendTransform(t)
            rate.sleep()

    def generate_tf(self, parent, child, position, rotation):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation = position  # needs to be postion tuple
        t.transform.rotation = rotation  # needs to be quat tuple
        self.tfList.append(t)
        return t

    def construct_4Dmatrix(self, trans, rot):
        # print trans, rot
        transform = numpy.zeros([4, 4])
        transform[0:3, 0:3] = rot
        transform[0:3, 3] = trans
        transform[3, 3] = 1
        return transform

    def generate_rotation_matrix(self, theta):
        # Generate matrix to rotation about the z-axis of shelf frame to get bunch of new transforms to use for projection
        transform = numpy.array([[numpy.cos(theta), numpy.sin(theta), 0, 0],
                                [-numpy.sin(theta), numpy.cos(theta), 0, 0],
                                [0, 0, 1, 0],
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
            print "could not find scene xml for object: %s" % req.item

        return numpy.array(size)

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

    def get_obb_points(self, size):
        x = size[0]
        y = size[1]
        z = size[2]

        point1 = (x/2, y/2, z/2)
        point2 = (x/2, -y/2, z/2)
        point3 = (-x/2, y/2, z/2)
        point4 = (-x/2, -y/2, z/2)
        point5 = (x/2, y/2, -z/2)
        point6 = (x/2, -y/2, -z/2)
        point7 = (-x/2, y/2, -z/2)
        point8 = (-x/2, -y/2, -z/2)
        points = numpy.array([point1, point2, point3, point4, point5, point6, point7, point8])
        return points

    def compute_width(self, min_y, max_y):
        return abs(max_y-min_y)

    def compute_depth(self, min_x, max_x, pointList):
        x_lowerbound = min_x - self.fingerlength + self.x_lowerboundoffset
        x_upperbound = max_x - self.fingerlength + self.x_upperboundoffset
        depth = x_lowerbound  # depth currently simply set as the lowerbound later may want to make it as a function of if collision check fail, we can reselect the depth to use to be within the bound. current we don't have this function
        return depth

    def compute_height(self, min_z, max_z):
        z_lowerbound = min_z + self.z_lowerboundoffset
        z_upperbound = max_z + self.z_upperboundoffset
        height = z_lowerbound  # height currently set so gripper won't collide into the shelf lip. later we may want this as a function simliar to the function we may use for the depth calculation with collision check. 
        if self.showOutput:
            print "height: " + str(height)
        return height

    def compute_y_mid(self, miny, maxy):
        y = (miny+maxy)/2
        ymid = numpy.array([[0], [y], [0], [1]])
        return ymid

    def check_width(self, width):
        if width <= self.gripperwidth:
            isSmaller = True
        else:
            isSmaller = False
        return isSmaller

    def compute_score(self, width, rotation):
        fscore = numpy.true_divide(width, self.gripperwidth)[0]
        gscore = abs(rotation)
        weight = 0.5
        score = (1-weight)*fscore + weight*gscore
        # print "%s = 0.5*%s + 0.5*%s" % (score, fscore, gscore)
        return score

    def compute_minmax(self, pointList):
        # Loop through pointList to find the min & max for the x, y, and z
        min_x = 999999999999999
        min_y = 999999999999999
        min_z = 999999999999999
        max_x = -99999999999999
        max_y = -99999999999999
        max_z = -99999999999999

        for point in pointList:
            if point[0] > max_x:  # if x point is larger than current max x then replace with new one
                max_x = point[0]
            if point[0] < min_x:  # if x point is smaller than current min x then replace with new one
                min_x = point[0]
            if point[1] > max_y:  # if y point is larger than current max y then replace with new one
                max_y = point[1]
            if point[1] < min_y:  # if y point is smaller than current min y then replace with new one
                min_y = point[1]
            if point[2] > max_z:  # if z point is larger than current max z then replace with new one
                max_z = point[2]
            if point[2] < min_z:  # if z point is smaller than current min z then replace with new one
                min_z = point[2]
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
        print name + " is unit quaternion: " + str((approach.orientation.x)**2 + (approach.orientation.y)**2 + (approach.orientation.z)**2 + (approach.orientation.w)**2)

    def get_grasp_cb(self, req):

        q_proj_msg = Q.PriorityQueue()
        q_approach_msg = Q.PriorityQueue()
        approachList = []
        projectionList = []
        countbad = 0

        print "************************************************** Request start **************************************************"
        print "Requesting valid grasps for %s" % req.item

        # Request the 8 Oriented bounding box points wrt to the object's frame.
        size = self.get_object_extents(req)
        pointcloud = self.get_obb_points(size)
        # pointcloud = list(pc2.read_points(req.object_points, skip_nans=True,
        #                                   field_names=("x", "y", "z")))

        # Generate TFs to project onto
        for theta in self.thetaList:
            # raw_input("Press Enter to continue...")
            # Get TFs
            Tbaseshelf = self.get_tf('/base_link', '/shelf')
            # Tshelfobj = self.get_tf('/shelf', '/object')
            # Tbaseobj = Tbaseshelf*Tshelfobj
            Tbaseobj = PoseToMatrix(req.object_pose)  # same Trob_obj request from offline planner

            # Tshelfobj = PoseToMatrix(self.tf.transformPose("/shelf", PoseStamped(Header(stamp=rospy.Time.now(), frame_id='/base_link'), req.object_pose)).pose)
            Tshelfobj = numpy.dot(inv(Tbaseshelf), Tbaseobj)
            if self.showOutput:
                print "************************************************** Start loop **************************************************"
                print "OBBPoints: ", pointcloud
                print "Transform from base to shelf: ", Tbaseshelf
                print "Transform from shelf to obj: ", Tshelfobj

            Tshelfproj = self.generate_rotation_matrix(theta)
            Tbaseproj = numpy.dot(Tbaseshelf, Tshelfproj)
            if theta == 0:
                print "projecting to shelf frame"

            if self.showOutput:
                print "Transform from shelf to proj: ", Tshelfproj, type(Tshelfproj)
                # print "Transform from base to proj", Tbaseproj, type(Tbaseproj)

            # Extract Translation component of Tshelfobj
            Trans_shelfobj = Tshelfobj[0:3, 3]
            Rot_shelfproj = Tshelfproj[0:3, 0:3]
            Tshelfproj_new = self.construct_4Dmatrix(Trans_shelfobj, Rot_shelfproj)
            if self.showOutput:
                print Tshelfproj_new

            # Get TF for projection to object
            Tprojshelf = inv(Tshelfproj_new)
            Tprojobj = numpy.dot(Tprojshelf, Tshelfobj)

            # Loop through OBB points and transform them to be wrt to the target projection frame which then is wrt to base
            points = []
            for OBBPoint in pointcloud:
                oldpt = numpy.array([[OBBPoint[0]], [OBBPoint[1]], [OBBPoint[2]], [1]])
                projected_OBBPoint = numpy.dot(Tprojobj, oldpt)
                points.append(projected_OBBPoint)
            if self.showOutput:
                print "List of transformed OBB points after projection to target projection frame"
                print points

            # Get min max points. Pass in transformed OBBPoints list to get min max for target frame. Compute width of projection shadow. Width is the y axis because shelf frame is set that way with y axis as width. Check if width of shadow projection can fit inside gripper width
            min_max = self.compute_minmax(points)
            min_x, max_x, min_y, max_y, min_z, max_z = min_max
            width = self.compute_width(min_y, max_y)
            print "Min-max values: ", min_x, max_x, min_y, max_y, min_z, max_z
            if self.showOutput:
                print "Min-max values [minx,maxx,miny,maxy,minz,maxz]: ", min_x, max_x, min_y, max_y, min_z, max_z
                print "projection width: " + str(width)
                print "min_y: " + str(min_y)
                print "max_y: " + str(max_y)
            isSmaller = self.check_width(width)

            # Compute cost
            if isSmaller:
                score = self.compute_score(width, theta)

                # Compute mid-y point using miny maxy
                mid_y = self.compute_y_mid(min_y, max_y)
                depth = self.compute_depth(min_x, max_x, points)  # select depth to be with a min and max bound
                height = self.compute_height(min_z, max_z)  # select the height so bottom  of object and also hand won't collide wit shelf lip. may need to take into acount the max_z and objects height to see if object will hit top of shelf.
                if self.showOutput:
                    print "mid-y: "+str(mid_y)
                    print "x depth value "+str(depth)
                    print "x height value "+str(height)

                """ Compute midpt is probably not necessary. Seems it should always be centered I believe."""
                # Update projection TF with new y value set to be middle of the projection wrt to the projection frame

                # Trans_shelfproj = numpy.array([Tshelfproj_new[0,3], mid_y[1], Tshelfproj_new[2,3]+zoffset])
                # Trans_shelfproj = numpy.array([Tshelfproj_new[0, 3]+depth, mid_y[1], Tshelfproj_new[2,3]]+height)
                Trans_shelfproj = numpy.array([Tshelfproj_new[0, 3], Tshelfproj_new[1, 3], Tshelfproj_new[2, 3]+height])
                Rot_shelfproj = Tshelfproj_new[0:3, 0:3]
                Tshelfproj_update = self.construct_4Dmatrix(Trans_shelfproj, Rot_shelfproj)

                # Generate TF shelf to projection
                proj = geometry_msgs.msg.Pose()
                [trans, quat] = ExtractFromMatrix(Tshelfproj_update)
                proj.position.x = trans[0]
                proj.position.y = trans[1]
                proj.position.z = trans[2]
                proj.orientation.x = quat[0]
                proj.orientation.y = quat[1]
                proj.orientation.z = quat[2]
                proj.orientation.w = quat[3]
                tf_shelfproj = self.generate_tf('/shelf', '/projection', proj.position, proj.orientation)
                self.broadcast_single_tf(tf_shelfproj)

                # Construct approach TF as projection but further back
                Trans_projapproach = numpy.array([-0.3, 0, 0])
                Rot_projapproach = numpy.eye(3, 3)
                Tprojapproach = self.construct_4Dmatrix(Trans_projapproach, Rot_projapproach)

                # Generate TF for projection to approach pose
                approach = geometry_msgs.msg.Pose()
                [trans, quat] = ExtractFromMatrix(Tprojapproach)
                approach.position.x = trans[0]
                approach.position.y = trans[1]
                approach.position.z = trans[2]
                approach.orientation.x = quat[0]
                approach.orientation.y = quat[1]
                approach.orientation.z = quat[2]
                approach.orientation.w = quat[3]
                tf_projapproach = self.generate_tf('/projection', '/approach', approach.position, approach.orientation)
                self.broadcast_single_tf(tf_projapproach)

                # camera -15 deg offset about z-axis
                # camtheta = 0.174532925
                camtheta = 0.261799
                Tcamera = numpy.array([[1, 0, 0, 0],
                                       [0, numpy.cos(camtheta), -numpy.sin(camtheta), 0],
                                       [0, numpy.sin(camtheta), numpy.cos(camtheta), 0],
                                       [0, 0, 0, 1]])

                # Transform to orient hand to use x as approach direction
                TgraspIK = numpy.array([[0, 0, -1, -0.17],
                                        [-1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]])

                # TgraspIK = numpy.dot(TgraspIK, Tcamera)
                TgraspIK = numpy.dot(Tcamera, TgraspIK)

                Tbasepregrasp = numpy.dot(Tbaseshelf, Tshelfproj_update)
                TbaseIK_pregrasp = numpy.dot(Tbasepregrasp, TgraspIK)

                Tshelfapproach = numpy.dot(Tshelfproj_update, Tprojapproach)
                Tbaseapproach = numpy.dot(Tbaseshelf, Tshelfapproach)
                TbaseIK_approach = numpy.dot(Tbaseapproach, TgraspIK)

                # Construct msg. Then appened to queue with score as the priority in queue. This will put lowest score msg first in list.
                proj_msg = PoseFromMatrix(TbaseIK_pregrasp)
                approach_msg = PoseFromMatrix(TbaseIK_approach)
                q_proj_msg.put((score, proj_msg))
                q_approach_msg.put((score, approach_msg))
                if self.showOutput:
                    print "score is %f. Good approach direction. Gripper is wide enough" % score
            else:
                # poseList.pop() #Removed the pose that has bad cost. projection width doesn't fit in gripper
                score = self.compute_score(width, theta)
                if self.showOutput:
                    print "score is %f. Bad approach direction. Gripper not wide enough" % score
                countbad += 1

        print "number of bad approach directions: " + str(countbad)
        print " ================================================== end loop =================================================="

        while not q_proj_msg.empty():
            projectionList.append(q_proj_msg.get()[1])
        while not q_approach_msg.empty():
            approachList.append(q_approach_msg.get()[1])

        # APC grasp message to return
        graspList = self.generate_apc_grasp_poses(projectionList, approachList)
        grasps = apcGraspArray(
            grasps=graspList
        )

        # From center of palm to edge of it is 6cm then lip is about 2cm up so need to offset robot z-axis to move hand approach vector to be 8cm up from bottom of object.        
        print "================================================== Request end =================================================="
        return apcGraspDBResponse(status=True, grasps=grasps)


def publisher():
    rospy.init_node('online_grasp_server')

    grasp = grasping()
    while not rospy.is_shutdown():
        try:
            rospy.Service('getGrasps_online_server',
                          apcGraspDB, grasp.get_grasp_cb)
            print "Online grasp planner ready"
            rospy.spin()
        except:
            print traceback.format_exc()

if __name__ == '__main__':
    publisher()
