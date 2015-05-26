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
from apc_util.moveit import goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, bin_pose_tray, add_shelf, remove_shelf, Shelf, get_shelf_pose, NO_SHELF, SIMPLE_SHELF, FULL_SHELF, PADDED_SHELF
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
        return 'Success'

        targetBin = userdata.bin
        jointConfigHor = [0, 0, 0, 0, 0, 0, 0, 0]

        # TODO: THESE ARE MOSTLY VERTICAL POSES, CHANGE TO HORIZONTAL POSES IF NEEDED
        if targetBin == "A":  # oldSUCCESS!
            jointConfigHor = [-2.476390993374663, 1.018227984000994,
                              -0.7615361332731927, -0.4143850759424132,
                              1.743826755749119, 1.2232240642678402,
                              0.5953827787445809, 1.7196706206680665]

        elif targetBin == "B":  # oldSUCCESS!  *check that we have actually cleared shelf
            # jointConfigHor = [-2.2074857444262976, -3.045685540584711,
            #                   1.9, -2.0639123895240985,
            #                   1.801592142183052, -0.7345649238710564,
            #                   -1.7975638693339302, -1.6729443283591612]
            jointConfigHor = [-2.20769174274226, -3.044802765450554,
                              1.9, -2.063762136216949,
                              1.8008414892898243, -0.7355662270876514,
                              -1.7985231711726328, -1.6730311904692863]

            # jointConfigHor = [-2.5886653285200394, -2.374620051506527,
            #                1.8133726045423784, -1.8764388649633008,
            #                0.9006129161380904, -0.7376122309261526,
            #                -1.8880190429049368, -1.4743091056932842]

        elif targetBin == "C":
            jointConfigHor = [0.10225791436341165, 0.7835974930475644,
                           -1.6773199945660098, 1.49905452509214,
                           1.3427193003920177, 3.1189284019155186,
                           -1.4674104840922055, -0.9027630644540776]


        elif targetBin == "D":  # pose is vertical
            jointConfigHor = [2.9667019844055176, 2.7412068843841553,
                           0.09522612392902374, -1.1803146600723267,
                           2.2825026512145996, 1.8705755472183228,
                           1.8874949216842651, -2.919917583465576]


        elif targetBin == "E":  # pose is vertical
            jointConfigHor = [2.9667019844055176, 1.4224945306777954,
                           -0.7801656126976013, -0.2995363175868988,
                           2.195582151412964, 1.864424467086792,
                           1.6602683067321777, 2.5383474826812744]

        elif targetBin == "F":  # pose is vertical
            jointConfigHor = [1.5194422006607056,1.810523509979248,
                           -1.2088792324066162, 1.3328773975372314,
                           -1.8696491718292236, 1.8829082250595093,
                           -1.2678426504135132, 1.606799840927124]

        elif targetBin == "G":  # pose is vertical
            jointConfigHor = [1.5194422006607056,1.810523509979248,
                           -1.2088792324066162, 1.3328773975372314,
                           -1.8696491718292236, 1.8829082250595093,
                           -1.2678426504135132, 1.606799840927124]

        elif targetBin == "H":  # pose is vertical
            jointConfigHor = [2.966156482696533, 1.8770301342010498,
                           1.2306787967681885, -0.586269199848175,
                           2.2546935081481934, 1.669684886932373,
                           1.7160991430282593, 0.7149554491043091]

        elif targetBin == "I":  # pose is vertical
            jointConfigHor = [1.5194591283798218, 1.251114845275879,
                           -1.8047455549240112, 2.224393606185913,
                           -1.9810069799423218, 1.1204286813735962,
                           -1.827457070350647, 0.8016403913497925]

        elif targetBin == "J":  # pose is vertical
            jointConfigHor = [2.608074188232422, 0.4578932821750641,
                           1.8810696601867676, -0.5525216460227966,
                           1.9467278718948364, 0.23977181315422058,
                           0.7547944784164429, -0.43715447187423706]

        elif targetBin == "K":  # pose is vertical
            jointConfigHor = [2.9667019844055176, -0.873210072517395,
                           -0.5380352735519409, 2.7276151180267334,
                           -2.2068514823913574, 1.085071086883545,
                           1.8169622421264648, 1.6070705652236938]

        elif targetBin == "L":  # pose is vertical
            jointConfigHor = [1.7551809549331665, 0.04665006324648857,
                           -1.8453619480133057, 1.8693605661392212,
                           -1.189427375793457, 1.5698546171188354,
                           -1.871213436126709, 0.8811066150665283]

        rospy.loginfo("Trying to scoop from bin '"+targetBin+"' ")

        # SCOOP
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_planning_time(10)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("/shelf")

        # horiztonal pose relative to bin
        horizontalPose = bin_pose_tray(targetBin).pose
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

        if (targetBin == "A" or targetBin == "D" or
                targetBin == "G" or targetBin == "J"):
            horizontalPose.position.y += leftOffset

        elif (targetBin == "B" or targetBin == "E" or
                targetBin == "H" or targetBin == "K"):
            horizontalPose.position.y += middleOffset

        elif (targetBin == "C" or targetBin == "F" or
                targetBin == "I" or targetBin == "L"):
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


        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)
        rospy.loginfo("planning to horizontal config")

        add_shelf(Shelf.PADDED)
        # remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED
        # self.arm.set_pose_target(horizontalPose)
        self.arm.set_joint_value_target(jointConfigHor)
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return 'Failure'
        rospy.loginfo("moved to horizontal config")        
        
        if not self.scoopBin(horizontalPose):
            return 'Failure'
        # rospy.sleep(100)

        # add_shelf(Shelf.PADDED)
        rospy.loginfo("planning cartesian path out of bin")

        if targetBin == "A":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
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

        elif targetBin == "B":
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

        elif targetBin == "C":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path to final bin pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]            

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            poses[-1].position.y = -0.686495  # INCLUDES CURRENT SHELF CALIBRATION as of Saturday night
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

            rospy.loginfo("planning cartesian path to pre-dumping pose")
            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "D":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "E":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "F":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "G":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "H":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "I":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "J":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "K":
            poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

        elif targetBin == "L":
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
        # target_pose = Pose()
        # target_pose.position.x = 0.472985
        # target_pose.position.y = -0.351667
        # target_pose.position.z = 0.653171
        # target_pose.orientation.x = -0.164656
        # target_pose.orientation.y = 0.766477
        # target_pose.orientation.z = -0.591483
        # target_pose.orientation.w = -0.188543

        # target_pose = self.arm.get_current_pose().pose
        #
        # target_pose.position.y -= 0.05


        #bins A, D, G, J,
        target_pose = Pose()
        target_pose.position.x = 0.472985
        target_pose.position.y = 0.351667
        target_pose.position.z = 0.553171
        target_pose.orientation.x = -0.188543
        target_pose.orientation.y = -0.591483
        target_pose.orientation.z = -0.766477
        target_pose.orientation.w = 0.164656
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
        if not self.move(plan.joint_trajectory):
            rospy.loginfo("FAILED going to horiztonal pose")
            return False
        
        rospy.loginfo(horizontalPose)
        raw_input("Hit enter to continue ")
        # rospy.sleep(15.0)
        ##################################################################################################

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
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.184
        poses[-1].position.z += -0.0810

        if not follow_path(self.arm, poses):
            rospy.loginfo("FAILED going part way into bin")
            return False

        # IN
        rospy.loginfo("going all the way into bin")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.1323

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
        constraints = Constraints()
        orientation_constraint = OrientationConstraint( header=Header(stamp=rospy.Time.now(), frame_id="/base_link"),
                                                        orientation=target_pose.orientation,
                                                        link_name="arm_right_link_7_t",
                                                        absolute_x_axis_tolerance=0.15,
                                                        absolute_y_axis_tolerance=0.15,
                                                        absolute_z_axis_tolerance=3.14,
                                                        weight=10   )

        # position_constraint = PositionConstraint(   header=Header(stamp=rospy.Time.now(), frame_id="/base_link"),
        #                                             link_name="arm_right_link_7_t",
        #                                             target_point_offset=make_vector(0.01, 0.01, 0.01),
        #                                             weight=9    )

        # constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        self.arm.set_path_constraints(constraints)
        self.arm.set_goal_tolerance(0.01)
        self.arm.set_pose_reference_frame("/base_link")
        self.arm.set_pose_target(target_pose)
        self.arm.set_planning_time(120)
        rospy.loginfo("planning constrained path")
        plan = self.arm.plan()
        result = move(self.arm, plan.joint_trajectory)

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
