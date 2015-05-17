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
        # rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' section '"
        #               + userdata.section+"' ")
        rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' ")

        jointValues = [0, 0, 0, 0, 0, 0, 0, 0]

        if userdata.bin == "A":
            jointValues = [-2.494322617213551, 0.5236253858745525,
                           -1.58927266388269, -1.5760277304868606,
                           -0.8813624307206954, -0.6095330671746702,
                           1.8744956287775745, 2.176790229329603]

        elif userdata.bin == "B":
            jointValues = [-2.5886653285200394, -2.374620051506527,
                           1.8133726045423784, -1.8764388649633008,
                           0.9006129161380904, -0.7376122309261526,
                           -1.8880190429049368, -1.4743091056932842]

        elif userdata.bin == "C":
            jointValues = [0.10225791436341165, 0.7835974930475644,
                           -1.6773199945660098, 1.49905452509214,
                           1.3427193003920177, 3.1189284019155186,
                           -1.4674104840922055, -0.9027630644540776]

        elif userdata.bin == "D":
            jointValues = [-2.9304159179992215, -1.188145482834881,
                           1.2413950394903648, 1.448796166496277,
                           2.1197289875770853, -0.2957380340693733,
                           0.22356084880954222, 2.779410685704981]

        elif userdata.bin == "E":
            jointValues = [-2.617078223150794, -1.1700249924714978,
                           1.8999579332309542, -1.098277334234239,
                           -1.7992468515858204, 2.3379186721820004,
                           0.4868876973376934, 2.4699605634819988]

        elif userdata.bin == "F":
            jointValues = [0.24514562009995183, 0.6469494458388144,
                           -1.0091301947299118, 0.9320973548089223,
                           1.992558226545299, 2.471672159247223,
                           -1.5916865933053375, -0.1546854272462241]

        elif userdata.bin == "G":
            jointValues = [-2.849710887732425, -1.8455983324002612,
                           0.423630514960432, -0.9114784023155666,
                           -2.217974920357377, 1.7475936923018869,
                           -0.7608672375929674, -1.9079153926250387]

        elif userdata.bin == "H":
            jointValues = [-2.8232615277416735, 0.6046811180302917,
                           -1.3199900812523193, -2.9213054464463113,
                           -1.6166655544412436, 1.680257377517132,
                           -1.54861721727236, -2.147159909325972]

        elif userdata.bin == "I":
            jointValues = [-0.2684631401271395, -0.5944761002154216,
                           1.025978429821326, 2.5126240735616863,
                           2.163912575069292, 1.741182533849802,
                           -1.895858443577232, 0.38680147950133437]

        elif userdata.bin == "J":
            jointValues = [-1.8638487643791364, 0.03964241835102434,
                           -1.3264222668862966, -0.42707390465510126,
                           1.5590911109617618, 1.6030578531357988,
                           0.1145294217592958, 2.0083975636970286]

        elif userdata.bin == "K":
            jointValues = [0.6461938727094164, -1.3830831244973643,
                           1.095973358496806, 0.45177256217465794,
                           -2.2491703946246178, 2.1547659742086287,
                           1.5037163344320572, -2.16208327232433]

        elif userdata.bin == "L":
            jointValues = [-0.2858043091428526, -1.5302684719380377,
                           1.4227702644339293, -2.1745498037104967,
                           2.218658549291765, -0.6976578836771063,
                           0.46488220398513513, 2.908319709480955]

        # this is done to achieve a good configuration for cartesian paths
        self.arm.set_joint_value_target(jointValues)
        self.arm.set_planning_time(5)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("/base_link")
        # add_shelf()
        remove_shelf()
        plan = self.arm.plan()
        if not self.move(plan.joint_trajectory):
            return 'Failure'
        # remove_shelf()

        # SCOOP
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_pose_reference_frame("/shelf")

        # Pose relative to bin used for robustness
        horizontalPose = bin_pose(userdata.bin).pose
        # horizontalPose.position.x += -0.25
        # horizontalPose.position.y += 0.03
        # horizontalPose.position.z += 0.08
        # 0.3851; -0.15482; 1.665

        # horizontalPose.position.x += -0.32416
        # horizontalPose.position.y += -0.00735
        # horizontalPose.position.z += 0.0976
        # horizontalPose.orientation.x = -0.26656
        # horizontalPose.orientation.y = -0.47462
        # horizontalPose.orientation.z = 0.41851
        # horizontalPose.orientation.w = 0.727
        # 0.31094; -0.19217; 1.6826
        # -0.27018; -0.45943; 0.42452; 0.73192

        # horizontalPose.position.x += -0.297581
        horizontalPose.position.x += -0.307581
        # horizontalPose.position.x += -0.267581
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
        # self.arm.set_pose_target(horizontalPose)
        # plan = self.arm.plan()
        # self.move(plan.joint_trajectory)

        # CURRENTLY NO SHELF BECAUSE MOTION PLANNER ALWAYS FAILED WITH SHELF
        # if not goto_pose(self.arm, horizontalPose, shelf=Shelf.NONE):
        #     return 'Failure'

        # THIS POSE CURRENTLY JUST FOR BIN C, NOT RELATIVE
        # horizontalPose = self.convertFrameRobotToShelf(
        # self.arm.get_current_pose().pose)
        # horizontalPose.position.x = 0.337519
        # horizontalPose.position.y = -0.196041
        # horizontalPose.position.z = 1.63963

        # horizontalPose.orientation.x = -0.293106
        # horizontalPose.orientation.y = -0.512959
        # horizontalPose.orientation.z = 0.403541
        # horizontalPose.orientation.w = 0.698654
        # horizontalPose = self.convertFrameRobotToShelf(horizontalPose)

        # rospy.loginfo("planning to horizontal pose")
        # self.arm.set_pose_target(horizontalPose)
        # planHor = self.arm.plan()
        # self.move(planHor.joint_trajectory)

        # remove_shelf()

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # # START (currently for bin C)
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x = -1.0659   #shelf frame
        # poses[-1].position.y = -0.2400   #shelf frame
        # poses[-1].position.z = 1.7419   #shelf frame

        # START
        poses.append(horizontalPose)

        # IN
        poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x += 0.125
        poses[-1].position.x += 0.155
        # poses[-1].position.x += 0.115

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.0555

        # IN + DOWN
        poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x += 0.174
        poses[-1].position.x += 0.184
        poses[-1].position.z += -0.0810

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.1323

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.1018

        # # BACK (so screws on bottom of tray are past lip)
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x += -0.01

        # # DOWN
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.z += -0.0318

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
        poses[-1].position.z += 0.10

        # AWAY FROM WALL
        poses.append(deepcopy(poses[-1]))
        if rightColumn:
            # poses[-1].position.y += -0.02
            poses[-1].position.y += -0.05
        elif not rightColumn:
            poses[-1].position.y += 0.02
            # poses[-1].position.y += 0.005

        if userdata.bin == "A":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.10

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "B":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            # poses[-1].position.z += 0.10
            poses[-1].position.z += 0.08

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "C":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4586
            # poses[-1].position.y += 0.05
            poses[-1].position.z += 0.05

            if not follow_path(self.arm, poses):
                return 'Failure'

            # rospy.sleep(100);

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]

            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            poses.append(deepcopy(poses[-1]))
            # To right side of shelf
            # poses[-1].position.y = -0.602322
            poses[-1].position.y = -0.686495
            poses[-1].position.z += -0.05
            poses[-1].orientation.x = -0.36667
            poses[-1].orientation.y = -0.648119
            poses[-1].orientation.z = 0.333549
            poses[-1].orientation.w = 0.578135

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

        elif userdata.bin == "D":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "E":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "F":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "G":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "A":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "H":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "I":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "J":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "K":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "L":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        print "Planning Cartesian Path Dump....."
        if not follow_path(self.arm, poses):
            return 'Failure'

        # print "Dumping"
        # # add_shelf()
        # poses = [self.convertFrameRobotToShelf(self.arm.
        #                                        get_current_pose().pose)]

        # if userdata.bin == "A":
        #     return 'Failure'

        # elif userdata.bin == "B":
        #     return 'Failure'

        # elif userdata.bin == "C":
        #     poses[-1].position.y += -0.01
        #     # UNCALIBRATED SHELF, THIS IS A GUESS

        #     # UP
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.z += 0.15

        #     # ROTATE
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.y += -0.10
        #     poses[-1].orientation.x = -0.19625
        #     poses[-1].orientation.y = 0.71656
        #     poses[-1].orientation.z = -0.64673
        #     poses[-1].orientation.w = -0.17254

        #     # FORWARD
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.x += 0.20

        #     # this is currently done instead of follow_path
        #     # to avoid excessive segment errors
        #     traj, success = self.arm.compute_cartesian_path(poses, 0.01, 0.0)
        #     # if success < 1:
        #     #     rospy.logwarn("Cartesian trajectory could not be completed."
        #     #                   + "Only solved for: '" + str(success) + "'...")
        #     #     return 'Failure'
        #     # for point in traj.joint_trajectory.points:
        #     #     for vel in point.velocities:
        #     #         vel *= 1/1.2
        #     #     # for accel in point.accelerations:
        #     #     #     accel = 0.0
        #     #     point.time_from_start *= 1.2
        #     self.move(traj.joint_trajectory)

        #     poses = [self.convertFrameRobotToShelf(self.arm.
        #                                            get_current_pose().pose)]
        #     # poses[-1].position.y += -0.01
        #     # UNCALIBRATED SHELF, THIS IS A GUESS

        #     # DOWN
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.z += -0.75

        #     DUMP
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.z += -0.20
        #     poses[-1].orientation.x = 0.12539
        #     poses[-1].orientation.y = -0.46229
        #     poses[-1].orientation.z = 0.84801
        #     poses[-1].orientation.w = 0.22679

        #     # # UP
        #     # poses.append(deepcopy(poses[-1]))
        #     # poses[-1].position.z += 0.30

        #     # # OVER
        #     # poses.append(deepcopy(poses[-1]))
        #     # poses[-1].position.y += 0.30

        # elif userdata.bin == "D":
        #     return 'Failure'

        # elif userdata.bin == "E":
        #     return 'Failure'

        # elif userdata.bin == "F":
        #     return 'Failure'

        # elif userdata.bin == "G":
        #     return 'Failure'

        # elif userdata.bin == "H":
        #     return 'Failure'

        # elif userdata.bin == "I":
        #     return 'Failure'

        # elif userdata.bin == "J":
        #     return 'Failure'

        # elif userdata.bin == "K":
        #     return 'Failure'

        # elif userdata.bin == "L":
        #     return 'Failure'

        # if not follow_path(self.arm, poses):
        #     return 'Failure'

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
