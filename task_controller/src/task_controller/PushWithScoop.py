# PushWithScoop.py
# Smach state for using the tray 'vertically' to
# push all items in a bin from one side to the other.
#
# The state takes inputs of:  bin
# where the bin signifies which bin to do the task,


import rospy
import smach

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion

from apc_util.moveit import follow_path, goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, BIN
from apc_util.smach import on_exception
from motoman_moveit.srv import convert_trajectory_server


class PushWithScoop( smach.State ):

    def __init__( self, robot ):
        smach.State.__init__(   self, outcomes=[ 'Success', 'Failure', 'Fatal' ],
        input_keys=[ 'bin' ], output_keys=[] )
        self.arm = robot.arm_right_torso
        self.move = rospy.ServiceProxy(  "/convert_trajectory_server",
        convert_trajectory_server )

    @on_exception(failure_state="Failure")
    def execute( self, userdata ):
        rospy.loginfo( "Trying to push with scoop bin " + userdata.bin )

        jointValues = [0.0, -1.0030564513590334, -1.49978651413566, 0.457500317369117, -2.1772162870743323, 0.4509681667487428, -1.2043397683221861, -1.5581499385881046]

        self.arm.set_joint_value_target(jointValues)
        self.arm.set_planning_time(20)
        plan = self.arm.plan()
        self.move(plan)


        self.arm.set_pose_reference_frame( "/shelf" )
        self.arm.set_goal_orientation_tolerance( 0.1 )
        self.arm.set_goal_position_tolerance( 0.005 )


        rospy.loginfo("Moving to bin pose")
        if not execute_known_trajectory(self.arm, "Pick", userdata.bin):
            return "Failure"

        rospy.sleep(5.0)
        rospy.loginfo( "Going along inside left wall" )

        poses = [ self.convertFrameRobotToShelf(
                    self.arm.get_current_pose().pose ) ]

        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.x += 0.12
        poses[-1].position.y += 0.12

        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.x += 0.25
        poses[-1].position.y += 0.05

        rospy.sleep(5.0)
        rospy.loginfo( "Pushing items to right side" )
        #adjust orientation?
        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.y -= 0.2

        rospy.sleep(5.0)
        rospy.loginfo( "Removing from bin " )
        poses.append( deepcopy( poses[-1] ) )
        poses[-1].position.y += 0.05
        poses[-1].position.x -= 0.3


        if not follow_path(self.arm, poses):
            return 'Failure'

        return 'True'


    #TODO Use calibrated values, not hardcoded
    def convertFrameRobotToShelf( self, pose ):
        pose.position.x += -1.4026
        pose.position.y += -0.0797
        pose.position.z +=  0.048

        return pose
