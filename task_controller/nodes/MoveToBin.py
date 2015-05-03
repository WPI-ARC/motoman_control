import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

import moveit_msgs
import moveit_commander

from motoman_moveit.srv import convert_trajectory_server
from trajlib.srv import GetTrajectory
from util.shelf import add_shelf, remove_shelf
# from util.moveit import goto_pose

move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)


class MOVETOBIN(smach.State):

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
            if not goto_pose(self.arm, start, [10, 30, 60, 120]):
                return 'Failure'

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        robot = moveit_commander.RobotCommander()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(response.plan)
        display_trajectory_publisher.publish(display_trajectory)
        print response.plan
        raw_input("Continue? ")

        print move(response.plan.joint_trajectory)
        return 'Success'


def goto_pose(group, pose, times=[5, 20, 40, 60], with_shelf=True):
    """Moves the hand to a given `pose`, using the configured `group`. The
    planning time is modified based on the passed in `times` to try to
    plan quickly if possible, but fall back on longer plans if
    necessary. If `add_shelf` is true, a box model of the shelf is
    added to the environment to avoid collisions."""
    if with_shelf:
        add_shelf()
    for t in times:
        group.set_planning_time(t)
        rospy.loginfo("Planning for "+str(t)+" seconds...")
        plan = group.plan(pose)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory();    
        robot = moveit_commander.RobotCommander();
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
        print plan
        raw_input("Continue? ")

        if plan:
            print move(plan.joint_trajectory)
            if with_shelf:
                remove_shelf()
            return True
    if with_shelf:
        remove_shelf()
    return False
