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

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")

        from apc_util.moveit import goto_pose
        from apc_util.shelf import bin_pose
        target = bin_pose(userdata.bin).pose
        target.position.z += 0.02
        if userdata.bin in "IKL":
            target.position.x -= 0.1
            target.position.z += 0.1
            target.orientation.x = 0.036548
            target.orientation.y = 0.025034
            target.orientation.z = 0.72137
            target.orientation.w = 0.69113
            pass
        else:
            target.position.x -= 0.05
            if userdata.bin in "CF":
                target.position.x -= 0.05
            target.position.z += 0.1
            target.orientation.x = -0.69096
            target.orientation.y = 0.71708
            target.orientation.z = 0.0782
            target.orientation.w = -0.047583
        if not goto_pose(self.arm, target, [2, 2, 10, 30]):
            return 'Failure'
        return 'Success'

        # if execute_known_trajectory(self.arm, "Forward", userdata.bin):
        #     return "Success"

        return "Failure"
