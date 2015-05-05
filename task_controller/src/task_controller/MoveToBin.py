import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

import moveit_msgs
import moveit_commander

from motoman_moveit.srv import convert_trajectory_server
from trajlib.srv import GetTrajectory
from apc_util.moveit import goto_pose

move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)


class MoveToBin(smach.State):
    """
    Moves to the desired bin, so that relevant tasks can be performed.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.arm = robot.arm_left_torso
        self.trajlib = rospy.ServiceProxy("/trajlib", GetTrajectory)

    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")

        response = self.trajlib(task="Forward", bin_num=userdata.bin)
        # plan = self.trajlib(task="Drop", bin_num="A")
        print self.arm.get_active_joints()
        print response.plan.joint_trajectory.joint_names
        print self.arm.get_current_joint_values()
        response.plan.joint_trajectory.points.pop(0)
        print response.plan.joint_trajectory.points[0].positions
        print response.plan.joint_trajectory.points[1].positions
        print response.plan.joint_trajectory.points[2].positions

        start = list(response.plan.joint_trajectory.points[0].positions)
        print start

        if self.arm.get_current_joint_values() != start:
            self.arm.set_planner_id("RRTstarkConfigDefault")
            self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
            if not goto_pose(self.arm, start, [1, 10, 30, 60, 120]):
                return 'Failure'

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        robot = moveit_commander.RobotCommander()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(response.plan)
        display_trajectory_publisher.publish(display_trajectory)
        print response.plan

        print move(response.plan.joint_trajectory)
        return 'Success'
