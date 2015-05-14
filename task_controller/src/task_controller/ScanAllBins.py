import rospy
import smach
import numpy
from math import cos, sin, pi
from apc_util.transformation_helpers import PoseToMatrix, PoseFromMatrix

from apc_vision.srv import *
from apc_vision.msg import *
from apc_util.moveit import follow_path
from apc_util.shelf import bin_pose
from apc_util.smach import on_exception


class ScanAllBins(smach.State):
    """
    Scan all of the bins to find the items they contain.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin', 'item'],
                             output_keys=['pose', 'points'])
        self.arm = robot.arm_left_torso
        self.sample = rospy.ServiceProxy("sample_vision", SampleVision)

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        for bin in "ABCDEFGHIJKL":
            if not self.goto_bin(bin):
                return 'Failure'
            poses = self.get_poses()
            try:
                if not self.sample_bin(bin, poses):
                    return 'Failure'
            except rospy.ServiceException as e:
                rospy.logwarn("Error process: "+str(e))
                return 'Failure'
        return 'Success'

    def goto_bin(self, bin, poses):
        for pose in poses:
            current_pose = self.arm.get_current_pose().pose
            if not follow_path(self.arm, [current_pose, bin_pose(bin)]):
                return False
        return True

    def get_poses(self):
        center_pose = self.arm.get_current_pose().pose
        center_matrix = PoseToMatrix(center_pose)
        angle, offset = pi/12, 0.175
        move_left = numpy.array([[ cos(-angle), 0, sin(-angle), -offset],
                                 [           0, 1,           0,       0],
                                 [-sin(-angle), 0, cos(-angle),       0],
                                 [           0, 0,           0,       1]])
        move_right = numpy.array([[ cos(angle), 0, sin(angle), offset],
                                  [          0, 1,          0,      0],
                                  [-sin(angle), 0, cos(angle),      0],
                                  [          0, 0,          0,      1]])

        return [center_pose,
                PoseFromMatrix(numpy.dot(center_matrix, move_left)),
                PoseFromMatrix(numpy.dot(center_matrix, move_right))]

    def sample_bin(self, bin, poses):
        for pose in poses:
            current_pose = self.arm.get_current_pose().pose
            print current_pose, pose
            if not follow_path(self.arm, [current_pose, pose]):
                return False
            print "Sample:", self.sample(command=bin)
        return True
