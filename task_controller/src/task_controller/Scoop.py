import rospy
import smach

from copy import deepcopy
# from geometry_msgs.msg import Pose, Point, Quaternion
from motoman_moveit.srv import convert_trajectory_server

from apc_util.moveit import follow_path, goto_pose  # ,execute_known_trajectory
from apc_util.shelf import bin_pose, add_shelf, remove_shelf, Shelf
from apc_util.smach import on_exception


class Scoop(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service",
                                       convert_trajectory_server)

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' ")

        # SCOOP
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_planning_time(5)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("/shelf")

        # Pose relative to bin used for robustness
        horizontalPose = bin_pose(userdata.bin).pose
        horizontalPose.position.x += -0.307581
        horizontalPose.position.y += -0.011221
        horizontalPose.position.z += 0.05463
        horizontalPose.orientation.x = -0.293106
        horizontalPose.orientation.y = -0.512959
        horizontalPose.orientation.z = 0.403541
        horizontalPose.orientation.w = 0.698654

        leftOffset = -0.001
        middleOffset = -0.002
        rightOffset = 0.050

        rightColumn = False

        if (userdata.bin == "A" or userdata.bin == "D" or
                userdata.bin == "G" or userdata.bin == "J"):
            horizontalPose.position.y += leftOffset

        elif (userdata.bin == "B" or userdata.bin == "E" or
                userdata.bin == "H" or userdata.bin == "K"):
            horizontalPose.position.y += middleOffset

        elif (userdata.bin == "C" or userdata.bin == "F" or
                userdata.bin == "I" or userdata.bin == "L"):
            horizontalPose.position.y += rightOffset
            rightColumn = True

        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)
        rospy.loginfo("planning to horizontal pose")
        self.arm.set_pose_target(horizontalPose)
        remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)

        remove_shelf()

        rospy.loginfo("planning cartesian path into bin")
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # START
        # poses.append(horizontalPose)

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.155

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.0555

        # IN + DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.184
        poses[-1].position.z += -0.0810

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.1323

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.1018

        # ROTATE BACK/LIFT UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.0059
        poses[-1].position.y += 0.0
        poses[-1].position.z += -0.0370
        poses[-1].orientation.x = -0.36665
        poses[-1].orientation.y = -0.64811
        poses[-1].orientation.z = 0.33362
        poses[-1].orientation.w = 0.57811
        # TODO: maybe calibrate pose orientation

        # UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += 0.08

        # AWAY FROM WALL
        poses.append(deepcopy(poses[-1]))
        if rightColumn:
            poses[-1].position.y += -0.05
        elif not rightColumn:
            poses[-1].position.y += 0.05
            # STILL NEED TO TEST THIS

        if not follow_path(self.arm, poses):
                return 'Failure'

        rospy.loginfo("planning cartesian path out of bin")

        if userdata.bin == "A":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "B":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "C":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "D":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "E":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "F":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "G":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "H":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "I":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "J":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "K":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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

        elif userdata.bin == "L":
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            rospy.loginfo("planning cartesian path out of bin")
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


        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # To order bin
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = -0.164656  # -0.198328
        poses[-1].orientation.y = 0.766477  # 0.759802
        poses[-1].orientation.z = -0.591483  # -0.598499
        poses[-1].orientation.w = -0.188543  # -0.158639
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        rospy.loginfo("planning cartesian path to dumping pose")
        if not follow_path(self.arm, poses):
            return 'Failure'

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # Tilt little by little
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.143945
        poses[-1].orientation.y = -0.605741
        poses[-1].orientation.z = 0.757694
        poses[-1].orientation.w = 0.195594
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.113945
        poses[-1].orientation.y = -0.525741
        poses[-1].orientation.z = 0.827694
        poses[-1].orientation.w = 0.215594
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.472985  # 0.482178
        poses[-1].position.y = -0.351667  # -0.335627
        poses[-1].position.z = 0.753171  # 0.706449
        poses[-1].orientation.x = 0.112873
        poses[-1].orientation.y = -0.520793
        poses[-1].orientation.z = 0.819904
        poses[-1].orientation.w = 0.209268
        poses[-1] = self.convertFrameRobotToShelf(poses[-1])

        print "Planning Cartesian Path Dump....."
        if not follow_path(self.arm, poses):
            return 'Failure'

        # return right arm to home position
        add_shelf()
        jointValues = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        jointValues[0] = 0.0  # Torso
        jointValues[1] = 1.699523295294849
        jointValues[2] = -0.6448955832339801
        jointValues[3] = -0.06852598822491722
        jointValues[4] = -2.3331612363309975
        jointValues[5] = -0.3915515016420941
        jointValues[6] = 0.15148041914194765
        jointValues[7] = 0.4944912570006051

        # this is currently done instead of goto_pose to use joint values
        self.arm.set_planning_time(5)
        self.arm.set_joint_value_target(jointValues)
        plan = self.arm.plan()
        rospy.loginfo("moving right arm to home position")
        if not self.move(plan.joint_trajectory):
            return 'Failure'

        return 'Success'

    # TODO Use calibrated values, not hardcoded
    def convertFrameRobotToShelf(self, pose):
        pose.position.x += -1.40009583376
        pose.position.y += -0.0841733373195
        pose.position.z += 0.045

        # pose.position.x += -1.4026
        # pose.position.y += -0.0797
        # pose.position.z += 0.045   # 0.048???

        return pose
