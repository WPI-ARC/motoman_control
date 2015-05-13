import rospy
import smach

from gripper_srv.srv import gripper

from apc_util.moveit import scene
from apc_util.moveit import execute_known_trajectory
from apc_util.shelf import BIN


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

        # rospy.sleep(4)
        # from apc_util.moveit import goto_pose
        # from geometry_msgs.msg import Pose, Point, Quaternion
        # self.arm.set_planner_id("RRTstarkConfigDefault")
        # self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        # target = Pose(
        #     position=Point(x=0.4062, y=0.43, z=0.91521),
        #     orientation=Quaternion(x=-0.12117, y=0.49833, z=0.85847, w=0.0020136)
        #     # position=Point(x=0.50071, y=0.048405, z=0.97733),
        #     # orientation=Quaternion(x=-0.0042023, y=-0.008732, z=-0.80657, w=0.59106)
        # )
        # if not goto_pose(self.arm, target, [30, 60, 120], shelf=BIN(userdata.bin)):
        #     return 'Failure'

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        if not execute_known_trajectory(self.arm, "Drop", userdata.bin):
            return "Failure"

        print "Open Gripper:", self.gripper_control(command="open")
        scene.remove_attached_object("arm_left_link_7_t")
        return 'Success'
