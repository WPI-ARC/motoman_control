import rospy
import smach
import math

from geometry_msgs.msg import *
from moveit_msgs.msg import *

from tf import transformations

from copy import deepcopy
from motoman_moveit.srv import convert_trajectory_server

from std_msgs.msg import *

from apc_util.moveit import follow_path, move
# from apc_util.moveit import goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, bin_pose_tray, add_shelf, remove_shelf, Shelf, get_shelf_pose, NO_SHELF, SIMPLE_SHELF, FULL_SHELF, PADDED_SHELF, add_padded_lab, remove_padded_lab
from apc_util.smach import on_exception
# from constrained_path_generator.msg import *
# from constrained_path_generator.srv import *


class Scoop(smach.State):
    joint_states = []


    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service",
                                       convert_trajectory_server)

        self.rightColumn = False

        self.robot = robot

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        # COMMENT OUT THIS RETURN UNLESS 'PushWithScoop' IS CALLING EVERYTHING
        # return 'Success'

        self.targetBin = userdata.bin
        jointConfigHor = [0, 0, 0, 0, 0, 0, 0, 0]

        rospy.loginfo("Trying to scoop from bin '"+self.targetBin+"' ")

        # SCOOP
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_planning_time(10)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("/shelf")

        # horiztonal pose relative to bin
        horizontalPose = bin_pose_tray(self.targetBin).pose
        horizontalPose.position.x += -0.307581
        horizontalPose.position.y += -0.011221
        # horizontalPose.position.z += 0.05463
        horizontalPose.position.z += 0.18463
        horizontalPose.orientation.x = -0.293106
        horizontalPose.orientation.y = -0.512959
        horizontalPose.orientation.z = 0.403541
        horizontalPose.orientation.w = 0.698654
        # TODO: calibrate orientation?

        # FIX
        # leftOffset = -0.001
        # middleOffset = -0.002
        # rightOffset = 0.050

        leftOffset = -0.150
        middleOffset = -0.120
        rightOffset = 0.170

        if (self.targetBin == "A" or self.targetBin == "D" or
                self.targetBin == "G" or self.targetBin == "J"):
            horizontalPose.position.y += leftOffset

        elif (self.targetBin == "B" or self.targetBin == "E" or
                self.targetBin == "H" or self.targetBin == "K"):
            horizontalPose.position.y += middleOffset

        elif (self.targetBin == "C" or self.targetBin == "F" or
                self.targetBin == "I" or self.targetBin == "L"):
            horizontalPose.position.y += rightOffset
            self.rightColumn = True

        # # TEMPORARY ##################################################
        # horizontalPose.position.z += 0.13
        # if self.isLeftToRight:  # THIS ISN"T A THING IN THIS FILE!!!!
        #     if self.middleColumn:
        #         horizontalPose.position.y += -0.15
        #     else:
        #         horizontalPose.position.y += -0.12
        # else:
        #     horizontalPose.position.y += 0.12
        # ##################################################################

        
        jointConfigHor = [0, 0, 0, 0, 0, 0, 0, 0]

        if self.targetBin == "A":  # 
            # vertical pose
            jointConfigHor = [1.9681722954889078, -0.023200618121380513, 1.5089116451880789, 1.8039264439517484, 1.9849145300443676, 1.248039336029401, 1.620441408283454, -3.13]

            # horiztonal pose  DOESN"T FINISH CARTESIAN PATH INTO BIN (60%)
            jointConfigHor = [-2.6278238770266884, -2.8717882676343707, 1.9, 1.41899550351945, -1.394620876270875, -0.30435794374488895, 1.9, 1.8393546525122273]
            # jointConfigHor = [-2.6266047232928647, -2.8707911662742625, 1.9, 1.4185633027580957, -1.3925214891151116, -0.3055918485431591, 1.9, 1.838375550073248]

            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000
            # ^^ FINE POSITION TUNING FOR INDIVIDUAL BINS

        elif self.targetBin == "B":  # 
            # vertical pose
            jointConfigHor = [-2.691808686026977, -2.8018998306066116, 1.3848981009275314, -2.282453315654881, 1.8152513141302793, -0.6202050989860174, -1.624154936000525, -0.3587748187263247]

            # horizontal pose  DOESN"T FINISH CARTESIAN PATH INTO BIN (69%)
            jointConfigHor = [-2.8819607919477774, -2.5683663777443138, 1.9, -2.241769564971686, 1.1787675479307382, -0.5392291309261772, -1.8531964931997877, -1.8695438375092903]
 
            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "C":  # 
            # vertical pose
            jointConfigHor = [0.1128913227811488, 0.17736465719817437, -1.0755894763756846, 1.734991297482921, 1.9132498375426665, 2.425141013887845, -1.0310688499779752, -2.4997632535514924]

            # horizontal pose 
            jointConfigHor = [0.29026194412817585, 0.461542044508441, -1.594581910530645, 1.7898741139752892, 1.8562945499829162, 2.900906794012061, -1.1051486713512129, -1.0617352188427895]
            # 0.29029384157045307, 0.46193821390527073, -1.594604801319865, 1.7897073727041457, 1.8560188191669673, 2.9010215195414353, -1.1056236281812253, -1.0615710089607395
            # [0.37818669922807013, 0.48325390802784585, -1.868336510770857, 1.5357536397230587, 1.8766570272027714, 3.0659668609349486, -1.124000502851055, -0.81050432002129] WORKED

            self.isLeftToRight = False
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "D":  # 
            # vertical pose 
            jointConfigHor = [-2.223934986801738, 3.13, 1.2092354002259527, 0.9307218279859997, -1.8873873503542566, 2.2149979825293564, -1.2486240605659136, 0.28324722321298806]

            self.shortRow = True
            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "E":  # 
            # vertical pose
            jointConfigHor = [2.9544418528587726, 1.3567262870651748, -1.3266391225690815, -0.22451889273765355, 2.064895928713241, 1.7098359105053893, 1.747522515305617, 2.125112210336924]

            self.shortRow = True
            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "F":  # 
            # old
            jointConfigHor = [0.1128913227811488, 0.17736465719817437, -1.0755894763756846, 1.734991297482921, 1.9132498375426665, 2.425141013887845, -1.0310688499779752, -2.4997632535514924]

            self.shortRow = True
            self.isLeftToRight = False
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "G":  # 
            # vertical pose
            jointConfigHor = [2.4718291585873913, 1.1047984538173783, 1.5290256049994881, -2.1169639224415793, -2.0890748066865283, -2.178313072949579, 1.57456751422334, -1.7351008864298179]

            self.shortRow = True
            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "H":  # 
            # old
            jointConfigHor = [2.4718291585873913, 1.1047984538173783, 1.5290256049994881, -2.1169639224415793, -2.0890748066865283, -2.178313072949579, 1.57456751422334, -1.7351008864298179]


            self.shortRow = True
            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "I":  # 
            # old
            jointConfigHor = [1.3418513542538393, -1.9163393148721648, 1.8999111796476877, 1.9555683274308242, 2.085973354202339, 0.8327696820366999, 1.521983626079816, 0.9235781887349414]

            self.shortRow = True
            self.isLeftToRight = False
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "J":  # 
            # vertical pose
            jointConfigHor = [-2.814859477213427, 1.171284271024935, 1.2964470093710962, -1.8496939730019695, 2.154119940035741, -2.417159189716691, 0.29654290371162795, -3.13]

            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "K":  # 
            # old
            jointConfigHor = [-2.814859477213427, 1.171284271024935, 1.2964470093710962, -1.8496939730019695, 2.154119940035741, -2.417159189716691, 0.29654290371162795, -3.13]

            self.isLeftToRight = True
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000

        elif self.targetBin == "L":  # 
            # vertical pose
            jointConfigHor = [1.2086075801715137, 0.23124532053402494, -1.7309804228879488, -1.2106734273580417, 1.8133929146598422, 1.1998904379674205, 1.7356579754157866, -3.13]

            self.isLeftToRight = False
            horizontalPose.position.x += 0.000
            horizontalPose.position.y += 0.000
            horizontalPose.position.z += 0.000


        add_shelf(Shelf.PADDED)
        # remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED
        self.arm.set_joint_value_target(jointConfigHor)

        rospy.loginfo("planning to jointConfigHor")
        plan = self.arm.plan()
        if not len(plan.joint_trajectory.points) > 0:
            return 'Failure'

        remove_shelf()
        add_shelf(Shelf.FULL)

        rospy.loginfo("Moving to jointConfigHor")
        if not move(self.arm, plan.joint_trajectory):
            rospy.logerr("Failed to get to jointConfigHor")
            return 'Failure'

        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)
        if not self.scoopBin(horizontalPose):
            return 'Failure'
        # rospy.sleep(100)

        # add_shelf(Shelf.FULL)
        rospy.loginfo("planning cartesian path out of bin")

        if self.targetBin == "A":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            # poses[-1].position.x += -0.4586
            poses[-1].position.x += -0.3586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

            # poses = [self.convertFrameRobotToShelf(self.arm.
            #                                        get_current_pose().pose)]

            # poses.append(deepcopy(poses[-1]))
            # poses[-1].position.z += 0.05

            # rospy.loginfo("planning cartesian path to final bin pose")
            # if not follow_path(self.arm, poses):
            #     return 'Failure'

        elif self.targetBin == "B":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.y += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

            rospy.loginfo("moved over")
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            # poses[-1].position.x += -0.4586
            poses[-1].position.x += -0.3586  # MAY BE IN COLLISION WITH SHELF
            # poses[-1].position.y += 0.04
            poses[-1].position.z += 0.19
            # poses[-1].position.z += 0.04

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "C":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.x += -0.3586
            # poses[-1].position.y += 0.05
            # poses[-1].position.z += 0.05
            # poses[-1].position.z += 0.15
            poses[-1].position.z += 0.20

            if not follow_path(self.arm, poses):
                return 'Failure'

            # poses = [self.convertFrameRobotToShelf(self.arm.
            #                                        get_current_pose().pose)]

            # poses.append(deepcopy(poses[-1]))
            # poses[-1].position.z += 0.05

            # rospy.loginfo("planning cartesian path to final bin pose")
            # if not follow_path(self.arm, poses):
            #     return 'Failure'

            # poses = [self.convertFrameRobotToShelf(self.arm.
            #                                        get_current_pose().pose)]            

            # poses.append(deepcopy(poses[-1]))
            # # To right side of shelf
            # poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            # poses[-1].position.z += -0.05
            # poses[-1].orientation.x = -0.36667
            # poses[-1].orientation.y = -0.648119
            # poses[-1].orientation.z = 0.333549
            # poses[-1].orientation.w = 0.578135

            # rospy.loginfo("planning cartesian path to pre-dumping pose")
            # if not follow_path(self.arm, poses):
            #     return 'Failure'

        elif self.targetBin == "D":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "E":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "F":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "G":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "H":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "I":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "J":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "K":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            # poses[-1].position.x += -0.4586
            poses[-1].position.x += -0.3586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif self.targetBin == "L":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        #######################################################################
        rospy.loginfo("made it out")
        return 'Failure'
        #######################################################################


        # bins C, F, I, L
        target_pose = Pose()
        target_pose.position.x = 0.49195
        target_pose.position.y = -0.39594
        target_pose.position.z = 0.64392
        target_pose.orientation.x = 0.16997
        target_pose.orientation.y = -0.63061
        target_pose.orientation.z = 0.73307
        target_pose.orientation.w = 0.18988


        #bins A, D, G, J,
        # target_pose = Pose()
        # target_pose.position.x = 0.24128
        # target_pose.position.y = 0.65743
        # target_pose.position.z = 0.72495
        # target_pose.orientation.x = -0.52171
        # target_pose.orientation.y = -0.28389
        # target_pose.orientation.z = -0.029079
        # target_pose.orientation.w = 0.80398
        rospy.loginfo("Trying to follow constrained path")

        if not self.follow_constrained_path(target_pose):
            rospy.loginfo("FAILED to follow constrained path")
            return 'Failure'

        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # # To order bin
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x = 0.472985  # 0.482178
        # poses[-1].position.y = -0.351667  # -0.335627
        # poses[-1].position.z = 0.753171  # 0.706449
        # poses[-1].orientation.x = -0.164656  # -0.198328
        # poses[-1].orientation.y = 0.766477  # 0.759802
        # poses[-1].orientation.z = -0.591483  # -0.598499
        # poses[-1].orientation.w = -0.188543  # -0.158639
        # poses[-1] = self.convertFrameRobotToShelf(poses[-1])
        #
        # rospy.loginfo("planning cartesian path to dumping pose")
        # if not follow_path(self.arm, poses):
        #     return 'Failure'

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # TODO: FIX THIS STUFF
        # Tilt little by little
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.y += 0.02
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.143945
        poses[-1].orientation.y = -0.605741
        poses[-1].orientation.z = 0.757694
        poses[-1].orientation.w = 0.195594
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.y += 0.02
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.113945
        poses[-1].orientation.y = -0.525741
        poses[-1].orientation.z = 0.827694
        poses[-1].orientation.w = 0.215594
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.y += 0.02
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.112873
        poses[-1].orientation.y = -0.520793
        poses[-1].orientation.z = 0.819904
        poses[-1].orientation.w = 0.209268
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        print "Planning Cartesian Path Dump....."
        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED to dump")
            return 'Failure'

        # return right arm to home position
        add_shelf(Shelf.PADDED)
        jointConfigHome = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        jointConfigHome[0] = 0.0  # Torso
        jointConfigHome[1] = 1.699523295294849
        jointConfigHome[2] = -0.6448955832339801
        jointConfigHome[3] = -0.06852598822491722
        jointConfigHome[4] = -2.3331612363309975
        jointConfigHome[5] = -0.3915515016420941
        jointConfigHome[6] = 0.15148041914194765
        jointConfigHome[7] = 0.4944912570006051

        # this is currently done instead of goto_pose to use joint values
        self.arm.set_planning_time(5)
        self.arm.set_joint_value_target(jointConfigHome)
        plan = self.arm.plan()
        if not len(plan.joint_trajectory.points) > 0:
            return 'Failure'

        remove_shelf()
        add_shelf(Shelf.FULL)

        rospy.loginfo("moving right arm to home position")
        if not self.move(plan.joint_trajectory):
            rospy.loginfo("FAILED to move right arm home")
            return 'Failure'

        return 'Success'

    def convertFrameRobotToShelf(self, pose):
        shelf_stamped_pose = get_shelf_pose()

        pose.position.x += -(shelf_stamped_pose.pose.position.x)
        pose.position.y += -(shelf_stamped_pose.pose.position.y)
        pose.position.z += -(shelf_stamped_pose.pose.position.z)

        return pose

    def convertFrameShelfToRobot(self, pose):
        shelf_position = get_shelf_pose().pose.position

        pose.position.x += (shelf_position.x)
        pose.position.y += (shelf_position.y)
        pose.position.z += (shelf_position.z)

        return pose

    def scoopBin(self, horizontalPose):
        remove_shelf()

        # # rospy.loginfo("planning cartesian path into bin")
        # rospy.loginfo("going to horizontal pose")

        # # # SPLITTING THIS WAYPOINT INTO 2 PARTS IS UNTESTED!!!!!!!!!!!
        # # # START
        # # poses = [self.convertFrameRobotToShelf(self.arm.
        # #                                        get_current_pose().pose)]

        # # poses.append(deepcopy(poses[-1]))
        # # # poses[-1].position.x = horizontalPose.position.x
        # # # poses[-1].position.y = horizontalPose.position.y
        # # # poses[-1].position.z = horizontalPose.position.z
        # # poses[-1].orientation.x = horizontalPose.orientation.x
        # # poses[-1].orientation.y = horizontalPose.orientation.y
        # # poses[-1].orientation.z = horizontalPose.orientation.z
        # # poses[-1].orientation.w = horizontalPose.orientation.w

        # # if not follow_path(self.arm, poses):
        # #     rospy.loginfo("FAILED to dump")
        # #     return False

        # # START
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]

        # poses.append(horizontalPose)

        # if not follow_path(self.arm, poses):
        #     rospy.loginfo("FAILED going to horizontal pose")
        #     return False

        # THIS BLOCK OF CODE FOR OBTAINING JOINT CONFIGURATIONS FOR PLANNING ONLY, SHOULD BE COMMENTED OUT FOR ACTUAL RUNS
        rospy.sleep(1.0)
        rospy.loginfo("going to horiztonal pose")
        self.arm.set_pose_target(horizontalPose)
        plan = self.arm.plan()
        if not len(plan.joint_trajectory.points) > 0:
            return False
        add_shelf(Shelf.FULL)
        if not self.move(plan.joint_trajectory):
            rospy.loginfo("FAILED going to horiztonal pose")
            return False
        
        horizontalPose = self.convertFrameShelfToRobot(horizontalPose)
        rospy.loginfo(horizontalPose)

        raw_input("Hit enter to continue ")
        with open("horizontal_joint_config.txt", "a+") as out_file:
            joint_config = self.arm.get_current_joint_values()
            out_file.write(str(self.targetBin) + "\t" + str(joint_config) + "\n")
        # rospy.sleep(15.0)
        ##################################################################################################

        remove_shelf()
        # IN
        rospy.loginfo("entering bin")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.155

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED entering bin")
            return False

        # DOWN
        rospy.loginfo("lowering tray")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.0555

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED lowering tray")
            return False

        # IN + DOWN
        rospy.loginfo("going part way into bin")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))# poses[-1].position.z += -0.0810
        poses[-1].position.x += 0.184
        # poses[-1].position.z += -0.0810
        # poses[-1].position.z += -0.0160
        poses[-1].position.z += -0.0410

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED going part way into bin")
            return False

        # IN
        rospy.loginfo("going all the way into bin")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.1323
        # poses[-1].position.z += -0.0650
        poses[-1].position.z += -0.0400

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED going all the way into bin")
            return False

        # DOWN
        rospy.loginfo("going down to level tray")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.1018

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED going down to level tray")
            return False

        # # ROTATE BACK/LIFT UP
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x += 0.0059
        # poses[-1].position.y += 0.0
        # poses[-1].position.z += -0.0370
        # poses[-1].orientation.x = -0.36665
        # poses[-1].orientation.y = -0.64811
        # poses[-1].orientation.z = 0.33362
        # poses[-1].orientation.w = 0.57811
        # # TODO: maybe calibrate pose orientation

        # SPLITTING THIS WAYPOINT INTO 2 PARTS IS UNTESTED!!!!!!!!!!!
        # ROTATE BACK/LIFT UP
        rospy.loginfo("adjusting position")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.0059
        poses[-1].position.y += 0.0
        poses[-1].position.z += -0.0370

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED adjusting position")
            return False

        # ROTATE BACK/LIFT UP
        rospy.loginfo("changing orientation")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].orientation.x = -0.36665
        poses[-1].orientation.y = -0.64811
        poses[-1].orientation.z = 0.33362
        poses[-1].orientation.w = 0.57811
        # TODO: maybe calibrate pose orientation

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED changing orientation")
            return False

        # UP
        rospy.loginfo("lifting objects")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += 0.08

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED lifting objects")
            return False

        # AWAY FROM WALL
        rospy.loginfo("moving away from wall")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        if self.rightColumn:
            poses[-1].position.y += -0.05
        elif not self.rightColumn:
            poses[-1].position.y += 0.05
            # STILL NEED TO TEST THIS

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED moving away from wall")
            return False

        return True

    def follow_constrained_path(self, target_pose):

        add_padded_lab()

        tray_orientation = [0, 0, 0.966657, 0.256075]
        target_pose_o = [target_pose.orientation.x,
                         target_pose.orientation.y,
                         target_pose.orientation.z,
                         target_pose.orientation.w]
        transformed_pose = transformations.quaternion_multiply(tray_orientation, target_pose_o)
        rospy.loginfo(transformed_pose)
        link_target_orientation = Pose()
        link_target_orientation.orientation.x = transformed_pose[0]
        link_target_orientation.orientation.y = transformed_pose[1]
        link_target_orientation.orientation.z = transformed_pose[2]
        link_target_orientation.orientation.w = transformed_pose[3]

        constraints = Constraints()
        orientation_constraint = OrientationConstraint( header=Header(stamp=rospy.Time.now(), frame_id="/base_link"),
                                                        orientation=link_target_orientation.orientation,
                                                        link_name="traybody_hand_right",
                                                        absolute_x_axis_tolerance=0.15,
                                                        absolute_y_axis_tolerance=3.14,
                                                        absolute_z_axis_tolerance=0.15,
                                                        weight=10   )

        # position_constraint = PositionConstraint(   header=Header(stamp=rospy.Time.now(), frame_id="/base_link"),
        #                                             link_name="arm_right_link_7_t",
        #                                             target_point_offset=make_vector(0.01, 0.01, 0.01),
        #                                             weight=9    )

        # constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        self.arm.set_path_constraints(constraints)
        add_shelf(Shelf.PADDED)
        # remove_shelf()
        self.arm.set_goal_tolerance(0.01)
        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_pose_reference_frame("/base_link")
        self.arm.set_pose_target(target_pose)
        self.arm.set_planning_time(30)
        rospy.loginfo("planning constrained path")
        plan = self.arm.plan()
        if not len(plan.joint_trajectory.points) > 0:
            return 'Failure'
        rospy.loginfo("moving constrained path")
        remove_padded_lab()
        result = move(self.arm, plan.joint_trajectory)
        if not result:
            return 'Failure'

        rospy.loginfo("Success constrained path")
        self.arm.clear_path_constraints()
        rospy.sleep(50)

        if result:
            rospy.loginfo("constrained path moved successfully")
        else:
            rospy.logerr("constrained path failed to move")

        self.arm.clear_path_constraints()


        return result



def make_pose(px, py, pz, rx, ry, rz, rw):
    new_pose = Pose()
    new_pose.position.x = px
    new_pose.position.y = py
    new_pose.position.z = pz
    new_pose.orientation.x = rx
    new_pose.orientation.y = ry
    new_pose.orientation.z = rz
    new_pose.orientation.w = rw
    return new_pose


def make_pose_stamped(px, py, pz, rx, ry, rz, rw, frame):
    pose_stamped = PoseStamped()
    pose_stamped.pose = make_pose(px, py, pz, rx, ry, rz, rw)
    pose_stamped.header.frame_id = frame
    return pose_stamped


def make_quaternion(w, x, y, z):
    new_quat = Quaternion()
    new_quat.w = w
    new_quat.x = x
    new_quat.y = y
    new_quat.z = z
    return new_quat


def make_vector(x, y, z):
    new_vector = Vector3()
    new_vector.x = x
    new_vector.y = y
    new_vector.z = z
    return new_vector

# def follow_path(group, path, collision_checking=True):
#     """Follows a cartesian path using a linear interpolation of the given
#     `path`. The `collision_checking` parameter controls whether or not
#     to check the path for collisions with the environment."""
#     traj, success = group.compute_cartesian_path(
#         path,
#         0.01,  # 1cm interpolation resolution
#         0.0,  # jump_threshold
#         avoid_collisions=collision_checking,
#     )

#     if success < 1:
#         rospy.logerr(
#             "Cartesian trajectory could not be completed. Only solved for: '"
#             + str(success) + "'..."
#         )
#         return False

#     if move(group, traj.joint_trajectory):
#         return True
#     else:
#         rospy.logerr("Failed to execute cartesian path")
#         return False
