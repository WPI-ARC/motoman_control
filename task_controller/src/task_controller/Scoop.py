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

        # SCOOP
        self.arm.set_planning_time(5)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_pose_reference_frame("/shelf")

        horizontalPose = bin_pose(userdata.bin).pose
        horizontalPose.position.x += -0.25
        horizontalPose.position.y += 0.03
        horizontalPose.position.z += 0.08
        horizontalPose.orientation.x = -0.26656
        horizontalPose.orientation.y = -0.47462
        horizontalPose.orientation.z = 0.41851
        horizontalPose.orientation.w = 0.727

        leftOffset = -0.001
        middleOffset = -0.002
        rightOffset = 0.001

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

        remove_shelf()
        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)
        self.arm.set_pose_target(horizontalPose)
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)
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

        remove_shelf()

        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        # # START (currently for bin C)
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x = -1.0659   #shelf frame
        # poses[-1].position.y = -0.2400   #shelf frame
        # poses[-1].position.z = 1.7419   #shelf frame

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.125

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.0555

        # IN + DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.174
        poses[-1].position.z += -0.0810

        # IN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.1323

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.07

        # BACK (so screws on bottom of tray are past lip)
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += -0.01

        # DOWN
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.0318

        # ROTATE BACK/LIFT UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.0159
        poses[-1].position.y += 0.0155
        poses[-1].position.z += -0.0370
        poses[-1].orientation.x = -0.36665
        poses[-1].orientation.y = -0.64811
        poses[-1].orientation.z = 0.33362
        poses[-1].orientation.w = 0.57811
        # TODO: maybe calibrate pose orientation

        # AWAY FROM WALL
        poses.append(deepcopy(poses[-1]))
        if rightColumn:
            poses[-1].position.y += -0.02
        elif not rightColumn:
            poses[-1].position.y += 0.02

        if userdata.bin == "A":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "B":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.z += 0.05

        elif userdata.bin == "C":
            # THIS RELATIVE PATH WAS FOR DEMO, PROBABLY GOES TOO HIGH
            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.05

            # OUT + UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += -0.4086
            poses[-1].position.y += 0.02
            poses[-1].position.z += 0.05

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

        if not follow_path(self.arm, poses):
            return 'Failure'

        print "Dumping"
        # add_shelf()
        poses = [self.convertFrameRobotToShelf(self.arm.
                                               get_current_pose().pose)]

        if userdata.bin == "A":
            return 'Failure'

        elif userdata.bin == "B":
            return 'Failure'

        elif userdata.bin == "C":
            poses[-1].position.y += -0.01
            # UNCALIBRATED SHELF, THIS IS A GUESS

            # UP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += 0.15

            # ROTATE
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.y += -0.10
            poses[-1].orientation.x = -0.19625
            poses[-1].orientation.y = 0.71656
            poses[-1].orientation.z = -0.64673
            poses[-1].orientation.w = -0.17254

            # FORWARD
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.x += 0.20

            # this is currently done instead of follow_path
            # to avoid excessive segment errors
            traj, success = self.arm.compute_cartesian_path(poses, 0.01, 0.0)
            # if success < 1:
            #     rospy.logwarn("Cartesian trajectory could not be completed."
            #                   + "Only solved for: '" + str(success) + "'...")
            #     return 'Failure'
            # for point in traj.joint_trajectory.points:
            #     for vel in point.velocities:
            #         vel *= 1/1.2
            #     # for accel in point.accelerations:
            #     #     accel = 0.0
            #     point.time_from_start *= 1.2
            self.move(traj.joint_trajectory)

            poses = [self.convertFrameRobotToShelf(self.arm.
                                                   get_current_pose().pose)]
            # poses[-1].position.y += -0.01
            # UNCALIBRATED SHELF, THIS IS A GUESS

            # DOWN
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += -0.75

            # DUMP
            poses.append(deepcopy(poses[-1]))
            poses[-1].position.z += -0.20
            poses[-1].orientation.x = 0.12539
            poses[-1].orientation.y = -0.46229
            poses[-1].orientation.z = 0.84801
            poses[-1].orientation.w = 0.22679

            # # UP
            # poses.append(deepcopy(poses[-1]))
            # poses[-1].position.z += 0.30

            # # OVER
            # poses.append(deepcopy(poses[-1]))
            # poses[-1].position.y += 0.30

        elif userdata.bin == "D":
            return 'Failure'

        elif userdata.bin == "E":
            return 'Failure'

        elif userdata.bin == "F":
            return 'Failure'

        elif userdata.bin == "G":
            return 'Failure'

        elif userdata.bin == "H":
            return 'Failure'

        elif userdata.bin == "I":
            return 'Failure'

        elif userdata.bin == "J":
            return 'Failure'

        elif userdata.bin == "K":
            return 'Failure'

        elif userdata.bin == "L":
            return 'Failure'

        if not follow_path(self.arm, poses):
            return 'Failure'

        # return right arm to home position
        add_shelf()
        jointValues = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # jointValues[0] = 0.000   # TORSO
        # jointValues[1] = 1.612   # S
        # jointValues[2] = 1.391   # L
        # jointValues[3] = 0.000   # E
        # jointValues[4] = 1.466   # U
        # jointValues[5] = 0.111   # R
        # jointValues[6] = 1.711   # B
        # jointValues[7] = -1.167   # T

        jointValues[0] = 0.0
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
