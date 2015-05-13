import rospy
import smach

from apc_util.moveit import execute_known_trajectory
from apc_util.smach import on_exception


class MoveToBin(smach.State):
    """
    Moves to the desired bin, so that relevant tasks can be performed.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.arm = robot.arm_left_torso

    @on_exception(failure_state="Failed")
    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")

        # from apc_util.moveit import goto_pose
        # from apc_util.shelf import bin_pose
        # target = bin_pose(userdata.bin).pose
        # target.position.x -= 0.15
        # target.position.z += 0.1
        # if not goto_pose(self.arm, target, [1, 10, 30, 60, 120]):
        #     return 'Failure'
        # return 'Success'

        if execute_known_trajectory(self.arm, "Forward", userdata.bin):
            return "Success"

        return "Failure"
