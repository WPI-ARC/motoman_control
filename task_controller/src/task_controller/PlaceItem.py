import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from gripper_srv.srv import gripper
from motoman_moveit.srv import convert_trajectory_server
from trajlib.srv import GetTrajectory
from trajectory_verifier.srv import CheckTrajectoryValidity
from trajectory_verifier.msg import CheckTrajectoryValidityQuery, CheckTrajectoryValidityResult

from apc_util.moveit import goto_pose

move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
check_collisions = rospy.ServiceProxy("/check_trajectory_validity", CheckTrajectoryValidity)


class PlaceItem(smach.State):
    """
    Place the item in the bin for shipping
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.robot = robot
        self.arm = robot.arm_left_torso
        self.gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
        self.trajlib = rospy.ServiceProxy("/trajlib", GetTrajectory)

    def execute(self, userdata):
        rospy.loginfo("Trying to place from bin '"+userdata.bin+"'...")

        response = self.trajlib(task="Drop", bin_num=userdata.bin)
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

        collisions = check_collisions(CheckTrajectoryValidityQuery(
            initial_state=JointState(
                header=Header(stamp=rospy.Time.now()),
                name=self.robot.sda10f.get_joints(),
                position=self.robot.sda10f.get_current_joint_values()
            ),
            trajectory=response.plan.joint_trajectory,
            check_type=CheckTrajectoryValidityQuery.CHECK_ENVIRONMENT_COLLISION,
        ))

        if collisions.result.status != CheckTrajectoryValidityResult.SUCCESS:
            rospy.logwarn("Can't execute path from trajectory library, status=%s" % collisions.result.status)
            return 'Failure'

        print move(response.plan.joint_trajectory)
        print "Open Gripper:", self.gripper_control(command="open")
        return 'Success'
