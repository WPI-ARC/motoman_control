import rospy
import smach

from gripper_srv.srv import gripper

from apc_util.moveit import execute_known_trajectory


class PlaceItem(smach.State):
    """
    Place the item in the bin for shipping
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.arm = robot.arm_left_torso
        self.gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)

    def execute(self, userdata):
        rospy.loginfo("Trying to place from bin '"+userdata.bin+"'...")

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        if not execute_known_trajectory(self.arm, "Drop", userdata.bin):
            return "Failure"

        print "Open Gripper:", self.gripper_control(command="open")
        return 'Success'
