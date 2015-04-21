import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose

from gripper_srv.srv import gripper
from motoman_moveit.srv import convert_trajectory_server

from util import filterGrasps, execute_grasp
from copy import deepcopy


class PICKSCOOP(smach.State):

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

    def execute(self, userdata):
        rospy.loginfo("Trying to pick scoop...")
        userdata.output = userdata.input

        print self.gripper_control(command="open")

        targetPose = Pose()
        targetPose.position.x = 0.735592
        targetPose.position.z = 0.446805
        targetPose.position.y = 1.18608
        targetPose.orientation.x = -0.108456
        targetPose.orientation.y = 0.708157
        targetPose.orientation.z = 0.693177
        targetPose.orientation.w = 0.0790984
        targetPose.position.x = 0.509;
        targetPose.position.y = 0.873;
        targetPose.position.z = 1.25832;
        targetPose.orientation.x = -0.673667;
        targetPose.orientation.y = 0.207501;
        targetPose.orientation.z = 0.706735;
        targetPose.orientation.w = -0.0603374;

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        self.arm.set_planning_time(1)
        plan = self.arm.plan(targetPose)
        print plan
        print self.move(plan.joint_trajectory)

        print self.gripper_control(command="70")

        poses = [self.arm.get_current_pose().pose]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.206
        poses[-1].position.z += -0.048
        traj, success = self.arm.compute_cartesian_path(
            poses,
            0.01,  # 1cm interpolation resolution
            0.0,  # jump_threshold disabled
            avoid_collisions=True,
        )
        print traj
        if success < 1:
            rospy.logwarn("Cartesian trajectory could not be completed. Only solved for: '"+str(success)+"'...")

        print self.move(traj.joint_trajectory)

        return "Success"