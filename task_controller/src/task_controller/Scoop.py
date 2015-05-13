import rospy
import smach

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion
from motoman_moveit.srv import convert_trajectory_server

from apc_util.moveit import follow_path, goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, add_shelf, remove_shelf
from apc_util.smach import on_exception


class Scoop(smach.State):

    # section can be "section1" (left), "section2" (default/middle), or "section3" (right) 

    def __init__(self, robot):
    #     smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
    #                          input_keys=['bin', 'section'], output_keys=[])
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

        self.arm = robot.arm_right_torso
        self.move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
        add_shelf()

    @on_exception(failure_state="Failed")
    def execute(self, userdata):
        outsideRight = False
        outsideLeft = False

        # rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' section '"+userdata.section+"' with arm '"+userdata.arm+"'...")
        rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' ")
        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_pose_reference_frame("/shelf")
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.005)

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

        jointValues = self.arm.get_current_joint_values()

        jointValues[0] = 0.000   # TORSO
        jointValues[1] = -1.557   # S
        jointValues[2] = 1.032   # L
        jointValues[3] = 0.000   # E
        jointValues[4] = -1.448   # U
        jointValues[5] = -1.238   # R
        jointValues[6] = -1.808   # B
        jointValues[7] = -0.087   # T

        self.arm.set_joint_value_target(jointValues)
        self.arm.set_planning_time(20)
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)

        remove_shelf()

        print "Executing scoop"
        poses = [self.arm.get_current_pose().pose]
        # TODO: MAKE THIS BASED ON CALIBRATION VALUES AND NOT HARD-CODED
        poses[-1].position.x += -1.4026
        poses[-1].position.y += -0.09
        poses[-1].position.z += 0.045

        # START (currently for bin C)
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = -1.0659
        poses[-1].position.y = -0.2400
        poses[-1].position.z = 1.7019
        poses[-1].orientation.x = -0.26656
        poses[-1].orientation.y = -0.47462
        poses[-1].orientation.z = 0.41851
        poses[-1].orientation.w = 0.727
        # TODO: maybe calibrate pose orientation

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
        poses = [self.arm.get_current_pose().pose]
        # TODO: MAKE THIS BASED ON CALIBRATION VALUES AND NOT HARD-CODED
        poses[-1].position.x += -1.4026
        poses[-1].position.y += -0.09   # UNCALIBRATED SHELF, THIS IS A GUESS
        poses[-1].position.z += 0.045

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

        poses = [self.arm.get_current_pose().pose]
        # TODO: MAKE THIS BASED ON CALIBRATION VALUES AND NOT HARD-CODED
        poses[-1].position.x += -1.4026
        poses[-1].position.y += -0.09   # UNCALIBRATED SHELF, THIS IS A GUESS
        poses[-1].position.z += 0.045

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

        jointValues[0] = 0.000   # TORSO
        jointValues[1] = -1.557   # S
        jointValues[2] = 1.032   # L
        jointValues[3] = 0.000   # E
        jointValues[4] = -1.448   # U
        jointValues[5] = -1.238   # R
        jointValues[6] = -1.808   # B
        jointValues[7] = -0.087   # T

        self.arm.set_planning_time(20)
        self.arm.set_joint_value_target(jointValues)
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)

        return 'Success'

# import roslib; roslib.load_manifest('task_controller')
# import rospy
# import smach

# from copy import deepcopy
# from geometry_msgs.msg import Pose, Point, Quaternion

# from apc_util.moveit import follow_path, goto_pose
# from apc_util.shelf import bin_pose

# class Scoop(smach.State):
#     """
#     Scoop the items out of a bin
#     """

#     def __init__(self, robot):
#         smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
#                              input_keys=['bin'], output_keys=[])
#         self.arm = robot.arm_left

#     def execute(self, userdata):
#         rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"'...")
#         self.arm.set_planner_id("RRTstarkConfigDefault")
#         self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

#         # 
#         intermediate_pose = Pose(
#             position=Point(x=0.17788, y=0.13899, z=1.8238),
#             orientation=Quaternion(x=0.18827, y=0.67103, z=0.7036, w=-0.13862),
#         )
#         if not goto_pose(self.arm, intermediate_pose, [1, 5, 30, 60]):
#             return 'Failure'

#         pose = bin_pose(userdata.bin).pose
#         pose.position.x += -0.22127
#         pose.position.y += 0.23
#         pose.position.z += 0.0785
#         pose.orientation = Quaternion(x=0.042959, y=0.70606, z=0.70488, w=-0.052714)
#         print "Pose: ", pose
#         if not goto_pose(self.arm, pose, [1, 5, 30, 60]):
#             return 'Failure'

#         print "Executing scoop"
#         poses = [self.arm.get_current_pose().pose]
#         poses.append(deepcopy(poses[-1]))
#         poses[-1].position.x += 0.206
#         poses[-1].position.z += -0.048
#         poses.append(deepcopy(poses[-1]))
#         poses[-1].position.z += -0.162
#         poses.append(deepcopy(poses[-1]))
#         poses[-1].position.x += 0.212
#         poses.append(deepcopy(poses[-1]))
#         poses[-1].position.x += -0.019
#         poses[-1].position.z += -0.057
#         poses.append(deepcopy(poses[-1]))
#         poses[-1].position.x += -0.027
#         poses[-1].position.z += -0.042
#         poses.append(deepcopy(poses[-1]))
#         poses[-1].position.x += -0.277
#         poses[-1].position.z += -0.103
#         if not follow_path(self.arm, poses):
#             return 'Failure'

#         return 'Success'
