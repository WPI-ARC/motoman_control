import rospy
import smach

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from motoman_moveit.srv import convert_trajectory_server

from apc_util.moveit import follow_path, goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, add_shelf, remove_shelf
from apc_util.smach import on_exception

from motoman_moveit.srv import convert_trajectory_server


class Scoop(smach.State):

    # section can be "section1" (left), "section2" (default/middle), or "section3" (right) 

    def __init__(self, robot):
        #    smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
        #                          input_keys=['bin', 'section'], output_keys=[])
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        self.armLeft = robot.arm_left_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
        add_shelf()

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        outsideRight = False
        outsideLeft = False

        # rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' section '"+userdata.section+"' with arm '"+userdata.arm+"'...")
        rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' ")

        # MOVE LEFT ARM HOME, THIS BLOCK OF CODE CAN BE REMOVED ONCE STATE MACHINE CHECKS LEFT ARM
        # jointValuesLeft = [0, 0, 0, 0, 0, 0, 0, 0]
        # jointValuesLeft[0] = 0.0
        # jointValuesLeft[1] = -1.0030564513590334
        # jointValuesLeft[2] = -1.49978651413566
        # jointValuesLeft[3] = 0.457500317369117
        # jointValuesLeft[4] = -2.1772162870743323
        # jointValuesLeft[5] = 0.4509681667487428
        # jointValuesLeft[6] = -1.2043397683221861
        # jointValuesLeft[7] = -1.5581499385881046

        # self.armLeft.set_joint_value_target(jointValuesLeft)
        # self.armLeft.set_planning_time(20)
        # self.armLeft.set_planner_id("RRTConnectkConfigDefault")
        # planLeft = self.armLeft.plan()
        # self.move(planLeft.joint_trajectory)

        jointValues = [0, 0, 0, 0, 0, 0, 0, 0]
        jointValues[0] = 0.0
        jointValues[1] = 1.699523295294849
        jointValues[2] = -0.6448955832339801
        jointValues[3] = -0.06852598822491722
        jointValues[4] = -2.3331612363309975
        jointValues[5] = -0.3915515016420941
        jointValues[6] = 0.15148041914194765
        jointValues[7] = 0.4944912570006051

        self.arm.set_joint_value_target(jointValues)
        self.arm.set_planning_time(20)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)


        self.arm.set_pose_reference_frame( "/shelf" )


        rospy.loginfo("Moving to bin pose")
        if not execute_known_trajectory(self.arm, "Pick", userdata.bin):
            return "Failure"

        # THIS POSE CURRENTLY JUST FOR BIN C, NOT RELATIVE
        verticalPose = self.convertFrameRobotToShelf(
            self.arm.get_current_pose().pose)
        verticalPose.position.x = 0.37066
        verticalPose.position.y = -0.052769
        verticalPose.position.z = 1.612
        verticalPose.orientation.x = 0.19924
        verticalPose.orientation.y = -0.69387
        verticalPose.orientation.z = -0.14743
        verticalPose.orientation.w = 0.6761
        verticalPose = self.convertFrameRobotToShelf(verticalPose)

        rospy.loginfo("planning to vertical pose")
        self.arm.set_pose_target(verticalPose)
        planVert = self.arm.plan()
        self.move(planVert.joint_trajectory)

        # rospy.sleep(5.0)
        rospy.loginfo( "Going along inside left wall" )
        remove_shelf()

        poses = [ self.convertFrameRobotToShelf(
                    self.arm.get_current_pose().pose ) ]

        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.x += 0.12
        poses[-1].position.y += 0.03

        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.x += 0.25
        poses[-1].position.y += 0.01

        # rospy.sleep(5.0)
        rospy.loginfo( "Pushing items to right side" )
        #adjust orientation?
        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.y -= 0.10

        # rospy.sleep(5.0)
        rospy.loginfo( "Removing from bin " )
        poses.append( deepcopy( poses[-1] ) )
        # poses[-1].position.y += 0.05
        poses[-1].position.x -= 0.4

        if not follow_path(self.arm, poses):
            return 'Failure'

        # SCOOP
        # self.arm.set_planner_id("RRTstarkConfigDefault")
        # self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        # self.arm.set_pose_reference_frame("/shelf")
        # self.arm.set_goal_orientation_tolerance(0.01)
        # self.arm.set_goal_position_tolerance(0.005)

        # offset for both directions for middle column bins (B, E, H, K)
        sectionOffsetMiddle = 0.00

        # offset for direction toward corner post (post blocks part of bin opening)
        sectionOffsetOutside = 0.00

        # offset for direction away from corner post
        sectionOffsetInside = 0.00

        # TODO: implement this pre-computed trajectory
        # execute_known_trajectory(self.arm, "Tray", userdata.bin)

        # ###########################################################
        # # FOR NOW DO THIS INSTEAD
        # jointValues = self.arm.get_current_joint_values()

        # if userdata.bin == "A":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "B":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "C":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "D":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "E":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "F":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "G":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "H":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "I":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "J":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "K":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # elif userdata.bin == "L":
        #     jointValues[0] =  0.000   # TORSO
        #     jointValues[1] = -1.557   # S
        #     jointValues[2] =  1.032   # L
        #     jointValues[3] =  0.000   # E
        #     jointValues[4] = -1.448   # U
        #     jointValues[5] = -1.238   # R
        #     jointValues[6] = -1.808   # B
        #     jointValues[7] = -0.087   # T

        # else:
        #     return Failure

        # pose = [self.arm.get_current_pose().pose]
        # # TODO: MAKE THIS BASED ON CALIBRATION VALUES AND NOT HARD-CODED
        # pose[-1].position.x += -1.4026
        # pose[-1].position.y += -0.0797
        # pose[-1].position.z +=  0.045

        # pose.append(deepcopy(pose[-1]))
        # pose[-1] = bin_pose(userdata.bin).pose
        # pose[-1].position.x += -0.3009
        # pose[-1].position.y += 0.03
        # pose[-1].position.z += 0.0619
        # pose[-1].orientation.x = -0.26656
        # pose[-1].orientation.y = -0.47462
        # pose[-1].orientation.z = 0.41851
        # pose[-1].orientation.w = 0.727

        # # convert to shelf frame coords
        # pose[-1].position.x += -1.4026
        # pose[-1].position.y += -0.0797
        # pose[-1].position.z += 0.045

        # if left section specified, adjust pose in positive Y direction
        # if userdata.section == "section1":
        #     if userdata.bin == "A" | "D" | "G" | "J":
        #         pose.position.y += sectionOffsetOutside
        #         outsideLeft = True

        #     if userdata.bin == "B" | "E" | "H" | "I":
        #         pose.position.y += sectionOffsetMiddle

        #     if userdata.bin == "C" | "F" | "I" | "K":
        #         pose.position.y += sectionOffsetInside

        # # if right section specified, adjust pose in negative Y direction
        # if userdata.section == "section3":
        #     if userdata.bin == "A" | "D" | "G" | "J":
        #         pose.position.y += -sectionOffsetInside

        #     if userdata.bin == "B" | "E" | "H" | "I":
        #         pose.position.y += -sectionOffsetMiddle

        #     if userdata.bin == "C" | "F" | "I" | "K":
        #         pose.position.y += -sectionOffsetOutside
        #         outsideRight = True

        # follow_path(self.arm, pose)

        # jointValues[0] = 0.000   # TORSO
        # jointValues[1] = -1.557   # S
        # jointValues[2] = 1.032   # L
        # jointValues[3] = 0.000   # E
        # jointValues[4] = -1.448   # U
        # jointValues[5] = -1.238   # R
        # jointValues[6] = -1.808   # B
        # jointValues[7] = -0.087   # T

        # self.arm.set_joint_value_target(jointValues)
        # self.arm.set_planning_time(20)
        # plan = self.arm.plan()
        # self.move(plan.joint_trajectory)

        print "Executing scoop"
        # poses = [self.convertFrameRobotToShelf(self.arm.get_current_pose().pose)]

        # # START (currently for bin C)
        # poses.append(deepcopy(poses[-1]))
        # poses[-1].position.x = -1.0659   #shelf frame
        # poses[-1].position.y = -0.2400   #shelf frame
        # poses[-1].position.z = 1.7019   #shelf frame
        # poses[-1].orientation.x = -0.26656
        # poses[-1].orientation.y = -0.47462
        # poses[-1].orientation.z = 0.41851
        # poses[-1].orientation.w = 0.727
        # # TODO: maybe calibrate pose orientation

        add_shelf()

        # THIS POSE CURRENTLY JUST FOR BIN C, NOT RELATIVE
        horizontalPose = self.convertFrameRobotToShelf(
            self.arm.get_current_pose().pose)
        horizontalPose.position.x = 0.337519
        horizontalPose.position.y = -0.196041
        horizontalPose.position.z = 1.63963
        horizontalPose.orientation.x = -0.293106
        horizontalPose.orientation.y = -0.512959
        horizontalPose.orientation.z = 0.403541
        horizontalPose.orientation.w = 0.698654
        horizontalPose = self.convertFrameRobotToShelf(horizontalPose)

        rospy.loginfo("planning to horizontal pose")
        self.arm.set_pose_target(horizontalPose)
        planHor = self.arm.plan()
        self.move(planHor.joint_trajectory)

        remove_shelf()
        
        poses = [self.convertFrameRobotToShelf(self.arm.get_current_pose().pose)]

        # # START (currently for bin C)
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = -1.0659   #shelf frame
        poses[-1].position.y = -0.2400   #shelf frame
        poses[-1].position.z = 1.7419   #shelf frame
        poses[-1].orientation.x = -0.26656
        poses[-1].orientation.y = -0.47462
        poses[-1].orientation.z = 0.41851
        poses[-1].orientation.w = 0.727

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
        poses[-1].position.z += -0.1018

        # ROTATE BACK/LIFT UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.0059
        poses[-1].position.y += 0.0155
        poses[-1].position.z += -0.0370
        poses[-1].orientation.x = -0.36665
        poses[-1].orientation.y = -0.64811
        poses[-1].orientation.z = 0.33362
        poses[-1].orientation.w = 0.57811
        # TODO: maybe calibrate pose orientation

        # if scooping outer section of outer column, move tray inward before
        # pulling out to prevent corner post from knocking items off tray
        # if outsideLeft:
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.y += -0.03

        # elif outsideRight:
        #     poses.append(deepcopy(poses[-1]))
        #     poses[-1].position.y += 0.03

        # UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += 0.05

        # OUT + UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += -0.4086
        poses[-1].position.z += 0.05

        follow_path(self.arm, poses)

        add_shelf()

        print "Dumping"
        poses = [self.convertFrameRobotToShelf(self.arm.get_current_pose().pose)]
        poses[-1].position.y += -0.01   # UNCALIBRATED SHELF, THIS IS A GUESS

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

        traj, success = self.arm.compute_cartesian_path(poses, 0.01, 0.0)
        if success < 1:
            rospy.logwarn("Cartesian trajectory could not be completed. Only solved for: '" + str(success) + "'...")
        for point in traj.joint_trajectory.points:
            for vel in point.velocities:
                vel *= 1/1.2
            for accel in point.accelerations:
                accel = 0.0
            point.time_from_start *= 1.2
        self.move(traj.joint_trajectory)

        poses = [self.convertFrameRobotToShelf(self.arm.get_current_pose().pose)]
        poses[-1].position.y += -0.01   # UNCALIBRATED SHELF, THIS IS A GUESS

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

        # UP
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += 0.30

        # OVER
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.y += 0.30

        follow_path(self.arm, poses)

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

        self.arm.set_planning_time(20)
        self.arm.set_joint_value_target(jointValues)
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)

        return 'Success'

    #TODO Use calibrated values, not hardcoded
    def convertFrameRobotToShelf( self, pose ):
        pose.position.x += -1.4026
        pose.position.y += -0.0797
        pose.position.z +=  0.045   # 0.048???

        return pose