import rospy
import smach
from geometry_msgs.msg import Pose, Quaternion

from gripper_srv.srv import gripper
from motoman_moveit.srv import convert_trajectory_server

from apc_util.scoop import allow_scoop_collision, disallow_scoop_collision
from apc_util.smach import on_exception
from copy import deepcopy


class PickScoop(smach.State):
    """
    Support for picking up the scoop from a known location.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose', 'bin'],
                             output_keys=['output'])
        self.arm = robot.arm_left_torso
        self.gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
        self.move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)

        # TODO: Handle response error
        response = self.gripper_control.call(command="activate")
        print "Activate Gripper:", response

    @on_exception(failure_state="Failed")
    def execute(self, userdata):
        rospy.loginfo("Trying to pick scoop...")
        userdata.output = userdata.input

        print self.gripper_control(command="70")

        self.arm.detach_object("Scoop")
        raw_input("Continue?")

        orientation = Quaternion(x=-0.083657, y=0.71336, z=0.69142, w=0.077827)
        initial = Pose(orientation=orientation)
        approach = Pose(orientation=orientation)
        target = Pose(orientation=orientation)
        initial.position.x = 0.49221
        initial.position.y = 0.4087
        initial.position.z = 1.15
        approach.position.x = 0.77
        approach.position.y = 0.40794
        approach.position.z = 1.15
        target.position.x = 0.77
        target.position.y = 0.25672
        target.position.z = 1.17

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_planning_time(1)
        plan = self.arm.plan(initial)
        print self.move(plan.joint_trajectory)

        allow_scoop_collision()

        poses = [self.arm.get_current_pose().pose, approach, target]
        traj, success = self.arm.compute_cartesian_path(
            poses,
            0.01,  # 1cm interpolation resolution
            0.0,  # jump_threshold disabled
            avoid_collisions=True,
        )
        if success < 1:
            rospy.logwarn("Cartesian trajectory could not be completed. Only solved for: '"+str(success)+"'...")

        raw_input("Continue?")
        print self.move(traj.joint_trajectory)
        print self.gripper_control(command="close")

        attach_scoop(self.arm)

        raw_input("Continue?")
        print "Picking scoop"
        poses = [self.arm.get_current_pose().pose]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x -= 0.5
        poses[-1].position.z += 0.25
        traj, success = self.arm.compute_cartesian_path(
            poses,
            0.01,  # 1cm interpolation resolution
            0.0,  # jump_threshold disabled
            avoid_collisions=True,
        )
        #print traj
        if success < 1:
            rospy.logwarn("Cartesian trajectory could not be completed. Only solved for: '"+str(success)+"'...")

        print self.move(traj.joint_trajectory)

        return "Success"
