# PushWithScoop.py
# Smach state for using the tray 'vertically' to
# push all items in a bin from one side to the other.
#
# The state takes inputs of:  bin, side
# where the bin signifies which bin to do the task,
# and the side is which side the tray will start pushing from
# e.g. 'Left' implies going along the left wall, and pushing to the right.


import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion

from apc_util.moveit import follow_path, goto_pose, execute_known_trajectory
from apc_util.shelf import bin_pose, BIN
from motoman_moveit.srv import convert_trajectory_server

class PushWithScoop( smach.State ):

    def __init__( self, robot ):
        smach.State.__init__(   self, outcomes=[ 'Success', 'Failure', 'Fatal' ],
        input_keys[ 'bin', 'side' ] )
        self.arm = robot.arm_right_torso
        self.move = rospy.ServiceProxy(  "/convert_trajectory_server",
        convert_trajectory_server )

    def execute( self, userdata ):
        rospy.loginfo( "Trying to push with scoop bin " + userdata.bin +
        " from side " + userdata.side )



        with bin_pose(userdata.bin).pose as initial_bin_pose:
            self.arm.set_planner_id( "RRTstarkConfigDefault" )
            self.arm.set_workspace( [-3, -3, -3, 3, 3, 3] )
            self.arm.set_pose_reference_frame( "/shelf" )
            self.arm.set_goal_orientation_tolerance( 0.1 )
            self.arm.set_goal_position_tolerance( 0.005 )


            if userdata.side == 'Left':

                # Get current pose, and convert to shelf frame
                pose = [ convertFrameRobotToShelf(
                                self.arm.get_current_pose().pose ) ]

                pose.append(deepcopy(pose[-1]))
                pose[-1] = initial_bin_pose
                pose[-1].orientation.x =  0.26324
                pose[-1].orientation.y = -0.66221
                pose[-1].orientation.z = -0.099005
                pose[-1].orientation.w =  0.69454
                #convert to shelf frame
                pose[-1] = convertFrameRobotToShelf( pose[-1] )

                #move in front of the bin
                if not follow_path(self.arm, pose):
                    return 'Failure'


                rospy.loginfo( "Going along inside left wall" )

                poses = [ convertFrameRobotToShelf(
                            self.arm.get_current_pose().pose ) ]

                poses.append( deepcopy( poses[-1] ) )
                poses[-1].position.x += 0.12
                poses[-1].position.y += 0.12

                poses.append( deepcopy( poses[-1] ) )
                poses[-1].position.x += 0.25
                poses[-1].position.y += 0.05

            elif userdata.side == 'Right':
                # Get current pose, and convert to shelf frame
                pose = [ convertFrameRobotToShelf(
                                            self.arm.get_current_pose().pose ) ]
                pose.append(deepcopy(pose[-1]))
                pose[-1] = initial_bin_pose
                #Add Right orientation here

                if not follow_path(self.arm, pose):
                    return 'Failure'

                rospy.loginfo( "Going along inside right wall" )

                poses = [ convertFrameRobotToShelf(
                            self.arm.get_current_pose().pose ) ]

                poses.append( deepcopy( poses[-1] ) )
                poses[-1].position.x += 0.12
                poses[-1].position.y -= 0.12

                poses.append( deepcopy( poses[-1] ) )
                poses[-1].position.x += 0.25
                poses[-1].position.y -= 0.05


    #TODO Use calibrated values, not hardcoded
    def convertFrameRobotToShelf( self, pose ):
        pose.position.x += -1.4026
        pose.position.y += -0.0797
        pose.position.z +=  0.048

        return pose
