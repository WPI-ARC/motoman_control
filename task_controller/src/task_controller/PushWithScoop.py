import rospy
import smach

from copy import deepcopy
# from geometry_msgs.msg import Pose, Point, Quaternion
from motoman_moveit.srv import convert_trajectory_server

from apc_util.moveit import follow_path, goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, add_shelf, remove_shelf, Shelf
from apc_util.smach import on_exception


class PushWithScoop(smach.State):

    # section can be "section1" (left), "section2" (default/middle),
    # or "section3" (right)

    def __init__(self, robot):
        # smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
        #                      input_keys=['bin', 'section'], output_keys=[])
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        self.armLeft = robot.arm_left_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service",
                                       convert_trajectory_server)
        self.middleColumn = False

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        shortRow = False
        startBin = "C"

        add_shelf()
        # rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' section '"
        #               + userdata.section+"' with arm '"+userdata.arm+"'...")
        rospy.loginfo("Trying to push with scoop in bin '"+userdata.bin+"' ")

        rospy.loginfo("Moving to home pose")
        jointValues = [0, 0, 0, 0, 0, 0, 0, 0]

        if userdata.bin == "A":
            jointValues = [-2.494322617213551, 0.5236253858745525,
                           -1.58927266388269, -1.5760277304868606,
                           -0.8813624307206954, -0.6095330671746702,
                           1.8744956287775745, 2.176790229329603]
            # jointValues = [2.9664804935455322, 1.6219801902770996,
            #                -0.7816655039787292, 2.7299845218658447,
            #                -1.3203777074813843, -1.796731948852539,
            #                1.7229489088058472, 1.5853843688964844]
            startBin = "C"

        elif userdata.bin == "B":
            jointValues = [-2.5886653285200394, -2.374620051506527,
                           1.8133726045423784, -1.8764388649633008,
                           0.9006129161380904, -0.7376122309261526,
                           -1.8880190429049368, -1.4743091056932842]
            # jointValues = [2.9664804935455322, -1.9493999481201172,
            #                1.2783514261245728, -2.9258782863616943,
            #                1.1115436553955078, -1.4286524057388306,
            #                -1.9131321907043457, -1.917506217956543]
            startBin = "C"

        elif userdata.bin == "C":
            jointValues = [0.10225791436341165, 0.7835974930475644,
                           -1.6773199945660098, 1.49905452509214,
                           1.3427193003920177, 3.1189284019155186,
                           -1.4674104840922055, -0.9027630644540776]
            startBin = "C"

        elif userdata.bin == "D":
            jointValues = [-2.9304159179992215, -1.188145482834881,
                           1.2413950394903648, 1.448796166496277,
                           2.1197289875770853, -0.2957380340693733,
                           0.22356084880954222, 2.779410685704981]
            startBin = "C"
            shortRow = True

        elif userdata.bin == "E":
            jointValues = [-2.617078223150794, -1.1700249924714978,
                           1.8999579332309542, -1.098277334234239,
                           -1.7992468515858204, 2.3379186721820004,
                           0.4868876973376934, 2.4699605634819988]
            startBin = "C"
            shortRow = True

        elif userdata.bin == "F":
            jointValues = [0.24514562009995183, 0.6469494458388144,
                           -1.0091301947299118, 0.9320973548089223,
                           1.992558226545299, 2.471672159247223,
                           -1.5916865933053375, -0.1546854272462241]
            startBin = "C"
            shortRow = True

        elif userdata.bin == "G":
            jointValues = [-2.849710887732425, -1.8455983324002612,
                           0.423630514960432, -0.9114784023155666,
                           -2.217974920357377, 1.7475936923018869,
                           -0.7608672375929674, -1.9079153926250387]
            startBin = "C"
            shortRow = True

        elif userdata.bin == "H":
            jointValues = [-2.8232615277416735, 0.6046811180302917,
                           -1.3199900812523193, -2.9213054464463113,
                           -1.6166655544412436, 1.680257377517132,
                           -1.54861721727236, -2.147159909325972]
            startBin = "C"
            shortRow = True

        elif userdata.bin == "I":
            jointValues = [-0.2684631401271395, -0.5944761002154216,
                           1.025978429821326, 2.5126240735616863,
                           2.163912575069292, 1.741182533849802,
                           -1.895858443577232, 0.38680147950133437]
            startBin = "C"
            shortRow = True

        elif userdata.bin == "J":
            jointValues = [-1.8638487643791364, 0.03964241835102434,
                           -1.3264222668862966, -0.42707390465510126,
                           1.5590911109617618, 1.6030578531357988,
                           0.1145294217592958, 2.0083975636970286]
            startBin = "C"

        elif userdata.bin == "K":
            jointValues = [0.6461938727094164, -1.3830831244973643,
                           1.095973358496806, 0.45177256217465794,
                           -2.2491703946246178, 2.1547659742086287,
                           1.5037163344320572, -2.16208327232433]
            startBin = "C"

        elif userdata.bin == "L":
            jointValues = [-0.2858043091428526, -1.5302684719380377,
                           1.4227702644339293, -2.1745498037104967,
                           2.218658549291765, -0.6976578836771063,
                           0.46488220398513513, 2.908319709480955]
            startBin = "C"

        # if not execute_known_trajectory(self.arm, 'Pick', userdata.bin):
        if not execute_known_trajectory(self.arm, 'Pick', startBin):
            return 'Failure'
            # this is done to achieve a good configuration for cartesian paths
            # self.arm.set_joint_value_target(jointValues)
            # self.arm.set_planning_time(15)
            # self.arm.set_planner_id("RRTConnectkConfigDefault")
            # self.arm.set_pose_reference_frame("/base_link")
            # plan = self.arm.plan()
            # self.move(plan.joint_trajectory)

        remove_shelf()

        if not execute_known_trajectory(self.arm, 'Dump', userdata.bin):
            return 'Failure'

        add_shelf()

        if not execute_known_trajectory(self.arm, 'Lift', userdata.bin):
            return 'Failure'

        if not execute_known_trajectory(self.arm, 'Home', userdata.bin):
            return 'Failure'

        return 'Success'

        horizontalPose = bin_pose(userdata.bin).pose
        # horizontalPose.position.x += -0.267581
        horizontalPose.position.x += -0.307581
        horizontalPose.position.y += -0.011221
        horizontalPose.position.z += 0.05463
        horizontalPose.orientation.x = -0.293106
        horizontalPose.orientation.y = -0.512959
        horizontalPose.orientation.z = 0.403541
        horizontalPose.orientation.w = 0.698654

        leftOffset = -0.001
        middleOffset = -0.002
        # rightOffset = 0.050
        rightOffset = 0.000

        if (userdata.bin == "A" or userdata.bin == "D" or
                userdata.bin == "G" or userdata.bin == "J"):
            horizontalPose.position.y += leftOffset

        elif (userdata.bin == "B" or userdata.bin == "E" or
                userdata.bin == "H" or userdata.bin == "K"):
            horizontalPose.position.y += middleOffset

        elif (userdata.bin == "C" or userdata.bin == "F" or
                userdata.bin == "I" or userdata.bin == "L"):
            horizontalPose.position.y += rightOffset

        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)

        horizontalStartPose = bin_pose(startBin).pose
        # horizontalStartPose.position.x += -0.307581
        # # horizontalStartPose.position.x += -0.297581
        # horizontalStartPose.position.y += -0.011221
        # horizontalStartPose.position.z += 0.05463
        # horizontalStartPose.orientation.x = -0.293106
        # horizontalStartPose.orientation.y = -0.512959
        # horizontalStartPose.orientation.z = 0.403541
        # horizontalStartPose.orientation.w = 0.698654
        # horizontalStartPose = self.convertFrameRobotToShelf(horizontalStartPose)

        # this is done to achieve a good configuration for cartesian paths
        # self.arm.set_joint_value_target(jointValues)
        # self.arm.set_planning_time(15)
        # self.arm.set_planner_id("RRTConnectkConfigDefault")
        # self.arm.set_pose_reference_frame("/base_link")
        # plan = self.arm.plan()
        # self.move(plan.joint_trajectory)

        # add_shelf()

        self.arm.set_planning_time(15)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("/shelf")

        verticalPose = bin_pose(userdata.bin).pose
        # print "bin_pose x: ", verticalPose.position.x
        # print "bin_pose y: ", verticalPose.position.y
        # print "bin_pose z: ", verticalPose.position.z

        # FIX
        leftOffset = -0.001
        middleOffset = -0.002
        rightOffset = 0.001

        if (userdata.bin == "A" or userdata.bin == "D" or
                userdata.bin == "G" or userdata.bin == "J"):
            # verticalPose.position.x += -0.264436
            verticalPose.position.x += -0.364436
            verticalPose.position.y += 0.13305766
            verticalPose.position.z += 0.027
            verticalPose.orientation.x = 0.19924
            verticalPose.orientation.y = -0.69387
            verticalPose.orientation.z = -0.14743
            verticalPose.orientation.w = 0.6761

            verticalPose.position.y += leftOffset
            verticalPose = self.convertFrameRobotToShelf(verticalPose)
            if not self.pushLeftToRight(horizontalStartPose, horizontalPose, verticalPose, shortRow):
                return 'Failure'

            return 'Success'

        if (userdata.bin == "B" or userdata.bin == "E" or
                userdata.bin == "H" or userdata.bin == "K"):
            self.middleColumn = True
            # verticalPose.position.x += -0.264436
            verticalPose.position.x += -0.364436
            verticalPose.position.y += 0.13305766
            verticalPose.position.z += 0.027
            verticalPose.orientation.x = 0.19924
            verticalPose.orientation.y = -0.69387
            verticalPose.orientation.z = -0.14743
            verticalPose.orientation.w = 0.6761

            verticalPose.position.y += middleOffset
            verticalPose = self.convertFrameRobotToShelf(verticalPose)
            if not self.pushLeftToRight(horizontalStartPose, horizontalPose, verticalPose, shortRow):
                return 'Failure'

            return 'Success'

        if (userdata.bin == "C" or userdata.bin == "F" or
                userdata.bin == "I" or userdata.bin == "L"):
            verticalPose.position.x += -0.382929
            verticalPose.position.y += -.12865034
            verticalPose.position.z += 0.032
            verticalPose.orientation.x = 0.686353
            verticalPose.orientation.y = 0.166894
            verticalPose.orientation.z = -0.680383
            verticalPose.orientation.w = -0.195307
            print "verticalPose x: ", verticalPose.position.x
            print "verticalPose y: ", verticalPose.position.y
            print "verticalPose z: ", verticalPose.position.z

            verticalPose.position.y += rightOffset
            verticalPose = self.convertFrameRobotToShelf(verticalPose)
            if not self.pushRightToLeft(horizontalStartPose, horizontalPose, verticalPose, shortRow):
                return 'Failure'

            return 'Success'

        # THIS POSE CURRENTLY JUST FOR BIN C, NOT RELATIVE
        # verticalPose.position.x = 0.37066
        # verticalPose.position.y = -0.052769
        # verticalPose.position.z = 1.612
        # verticalPose.orientation.x = 0.19924
        # verticalPose.orientation.y = -0.69387
        # verticalPose.orientation.z = -0.14743
        # verticalPose.orientation.w = 0.6761

        # verticalPose.position.x = 0.252167
        # verticalPose.position.y = -0.314477
        # verticalPose.position.z = 1.6145
        # verticalPose.orientation.x = 0.688677
        # verticalPose.orientation.y = 0.159918
        # verticalPose.orientation.z = -0.681856
        # verticalPose.orientation.w = -0.187677
        # verticalPose = self.convertFrameRobotToShelf(verticalPose)

    def pushLeftToRight(self, horizontalStartPose, horizontalPose, verticalPose, shortRow):
        # rospy.loginfo("planning to horizontal pose")
        # add_shelf()
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # poses.append(horizontalPose)
        # if not follow_path(self.arm, poses):
        #     return False

        rospy.loginfo("planning to vertical pose")
        # add_shelf()
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # poses.append(verticalPose)
        # if not follow_path(self.arm, poses):
        #     return False

        remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED HERE
        self.arm.set_pose_target(verticalPose)
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return False

        # add_shelf()  # just to check rviz
        # CURRENTLY NO SHELF BECAUSE MOTION PLANNER ALWAYS FAILED WITH SHELF
        # if not goto_pose(self.arm, verticalPose, shelf=Shelf.NONE):
        #     return 'Failure'

        # rospy.sleep(5.0)
        rospy.loginfo("Going along inside left wall")
        remove_shelf()

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.12
        poses[-1].position.y += 0.05
        if shortRow:
            poses[-1].position.z += -0.03

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.35
        poses[-1].position.y += 0.03
        if shortRow:
            poses[-1].position.z += 0.09

        # rospy.sleep(5.0)
        rospy.loginfo("Pushing items to right side")
        # adjust orientation?
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.y += -0.14
        if self.middleColumn:
            # poses[-1].position.y += -0.05
            pass

        # rospy.sleep(5.0)
        rospy.loginfo("Removing from bin ")
        poses.append(deepcopy(poses[-1]))
        # poses[-1].position.y += 0.05
        poses[-1].position.x += -0.50

        if not follow_path(self.arm, poses):
            return False

        return True

    def pushRightToLeft(self, horizontalStartPose, horizontalPose, verticalPose, shortRow):
        # rospy.loginfo("planning to horizontal pose")
        # add_shelf()
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # poses.append(horizontalPose)
        # if not follow_path(self.arm, poses):
        #     return False

        rospy.loginfo("planning to vertical pose")
        # add_shelf()
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]
        # poses.append(verticalPose)
        # if not follow_path(self.arm, poses):
        #     return False

        remove_shelf()  # SHELF SHOULD NOT ACTUALLY BE REMOVED HERE
        self.arm.set_pose_target(verticalPose)
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return False

        # add_shelf()  # just to check rviz
        # CURRENTLY NO SHELF BECAUSE MOTION PLANNER ALWAYS FAILED WITH SHELF
        # if not goto_pose(self.arm, verticalPose, shelf=Shelf.NONE):
        #     return 'Failure'

        # rospy.sleep(5.0)
        rospy.loginfo("Going along inside right wall")
        remove_shelf()

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.12
        poses[-1].position.y += -0.05
        if shortRow:
            poses[-1].position.z += -0.03

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.35
        poses[-1].position.y += -0.03
        if shortRow:
            poses[-1].position.z += 0.09

        # rospy.sleep(5.0)
        rospy.loginfo("Pushing items to left side")
        # adjust orientation?
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.y += 0.14

        # rospy.sleep(5.0)
        rospy.loginfo("Removing from bin ")
        poses.append(deepcopy(poses[-1]))
        # poses[-1].position.y += 0.05
        poses[-1].position.x += -0.50

        if not follow_path(self.arm, poses):
            return False

        return True

    # TODO Use calibrated values, not hardcoded
    def convertFrameRobotToShelf(self, pose):
        pose.position.x += -1.40009583376
        pose.position.y += -0.0841733373195
        pose.position.z += 0.045

        # pose.position.x += -1.4026
        # pose.position.y += -0.0797
        # pose.position.z += 0.045   # 0.048???

        return pose
