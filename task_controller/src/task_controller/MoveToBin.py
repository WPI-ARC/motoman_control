import rospy
import smach

from apc_util.moveit import execute_known_trajectory


class MoveToBin(smach.State):
    """
    Moves to the desired bin, so that relevant tasks can be performed.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.arm = robot.arm_left_torso

    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        if execute_known_trajectory(self.arm, "Forward", userdata.bin):
            return "Success"

        return "Failure"
