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

        from apc_util.moveit import goto_pose
        from apc_util.shelf import bin_pose
        self.arm.set_planner_id("KPIECEkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        target = bin_pose(userdata.bin).pose
        target.position.x -= 0.15
        target.position.z += 0.1
        if not goto_pose(self.arm, target, [1, 10, 30, 60, 120]):
            return 'Failure'

        return 'Success'

        # self.arm.set_planner_id("RRTstarkConfigDefault")
        # self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        # if execute_known_trajectory(self.arm, "Forward", userdata.bin):
        #     return "Success"

        # return "Failure"
