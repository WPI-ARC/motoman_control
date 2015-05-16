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
        add_shelf()
        # rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"' section '"
        #               + userdata.section+"' with arm '"+userdata.arm+"'...")
        rospy.loginfo("Trying to push with scoop in bin '"+userdata.bin+"' ")

        # MOVE LEFT ARM TO SIDE HOME,
        # THIS BLOCK OF CODE CAN BE REMOVED ONCE STATE MACHINE CHECKS LEFT ARM
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
        # self.armLeft.set_planning_time(5)
        # self.armLeft.set_planner_id("RRTConnectkConfigDefault")
        # planLeft = self.armLeft.plan()
        # self.move(planLeft.joint_trajectory)

        rospy.loginfo("Moving to home pose")
        # MOVE RIGHT ARM HOME,
        # THIS BLOCK OF CODE CAN BE REMOVED ONCE STATE MACHINE CHECKS RIGHT ARM
        # jointValues = [0, 0, 0, 0, 0, 0, 0, 0]
        # jointValues[0] = 0.0
        # jointValues[1] = 1.699523295294849
        # jointValues[2] = -0.6448955832339801
        # jointValues[3] = -0.06852598822491722
        # jointValues[4] = -2.3331612363309975
        # jointValues[5] = -0.3915515016420941
        # jointValues[6] = 0.15148041914194765
        # jointValues[7] = 0.4944912570006051
        # group_variable_values = [0, 0, 0, 0, 0, 0, 0, 0]
        # group_variable_values[0] = 0.0
        # group_variable_values[1] = -1.0030564513590334
        # group_variable_values[2] = -1.49978651413566
        # group_variable_values[3] = 0.457500317369117
        # group_variable_values[4] = -2.1772162870743323
        # group_variable_values[5] = 0.4509681667487428
        # group_variable_values[6] = -1.2043397683221861
        # group_variable_values[7] = -1.5581499385881046

        # this is currently done instead of goto_pose to use joint values
        # self.arm.set_joint_value_target(group_variable_values)
        # self.arm.set_planning_time(5)
        # self.arm.set_planner_id("RRTConnectkConfigDefault")
        # self.arm.set_pose_reference_frame("/base_link")
        # plan = self.arm.plan()
        # self.move(plan.joint_trajectory)

        # rospy.loginfo("Moving to horizontal pose")
        if not execute_known_trajectory(self.arm, "Pick", userdata.bin):
            return 'Failure'

        # add_shelf()

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
            if not self.pushLeftToRight(verticalPose):
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
            if not self.pushLeftToRight(verticalPose):
                return 'Failure'

            return 'Success'

        if (userdata.bin == "C" or userdata.bin == "F" or
                userdata.bin == "I" or userdata.bin == "L"):
            verticalPose.position.x += -0.382929
            verticalPose.position.y += -.12865034
            verticalPose.position.z += 0.0295
            verticalPose.orientation.x = 0.686353
            verticalPose.orientation.y = 0.166894
            verticalPose.orientation.z = -0.680383
            verticalPose.orientation.w = -0.195307
            print "verticalPose x: ", verticalPose.position.x
            print "verticalPose y: ", verticalPose.position.y
            print "verticalPose z: ", verticalPose.position.z

            verticalPose.position.y += rightOffset
            verticalPose = self.convertFrameRobotToShelf(verticalPose)
            if not self.pushRightToLeft(verticalPose):
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

    def pushLeftToRight(self, verticalPose):
        rospy.loginfo("moving to vertical pose")
        self.arm.set_pose_target(verticalPose)
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)
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

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.35
        poses[-1].position.y += 0.03

        # rospy.sleep(5.0)
        rospy.loginfo("Pushing items to right side")
        # adjust orientation?
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.y += -0.14
        if self.middleColumn:
            poses[-1].position.y += -0.05

        # rospy.sleep(5.0)
        rospy.loginfo("Removing from bin ")
        poses.append(deepcopy(poses[-1]))
        # poses[-1].position.y += 0.05
        poses[-1].position.x += -0.50

        if not follow_path(self.arm, poses):
            return False

        return True

    def pushRightToLeft(self, verticalPose):
        rospy.loginfo("planning to vertical pose")
        self.arm.set_pose_target(verticalPose)
        plan = self.arm.plan()
        self.move(plan.joint_trajectory)
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

        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.35
        poses[-1].position.y += -0.03

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
