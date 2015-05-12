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

        from apc_util.moveit import goto_pose
        from geometry_msgs.msg import Pose, Point, Quaternion
        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        target = Pose(
            position=Point(x=0.50071, y=0.048405, z=0.97733),
            orientation=Quaternion(x=-0.0042023, y=-0.008732, z=-0.80657, w=0.59106)
        )
        if not goto_pose(self.arm, target, [1, 10, 30, 60, 120]):
            return 'Failure'

        return 'Success'

        # self.arm.set_planner_id("RRTstarkConfigDefault")
        # self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        # if not execute_known_trajectory(self.arm, "Drop", userdata.bin):
        #     return "Failure"

        print "Open Gripper:", self.gripper_control(command="open")
        return 'Success'
