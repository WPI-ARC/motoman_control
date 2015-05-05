import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from copy import deepcopy
from geometry_msgs.msg import Pose, Point, Quaternion

from apc_util.moveit import follow_path, goto_pose
from apc_util.shelf import bin_pose

class SCOOP(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.arm = robot.arm_left

    def execute(self, userdata):
        rospy.loginfo("Trying to scoop from bin '"+userdata.bin+"'...")
        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

        # 
        intermediate_pose = Pose(
            position=Point(x=0.17788, y=0.13899, z=1.8238),
            orientation=Quaternion(x=0.18827, y=0.67103, z=0.7036, w=-0.13862),
        )
        if not goto_pose(self.arm, intermediate_pose, [1, 5, 30, 60]):
            return 'Failure'

        pose = bin_pose(userdata.bin).pose
        pose.position.x += -0.22127
        pose.position.y += 0.23
        pose.position.z += 0.0785
        pose.orientation = Quaternion(x=0.042959, y=0.70606, z=0.70488, w=-0.052714)
        print "Pose: ", pose
        if not goto_pose(self.arm, pose, [1, 5, 30, 60]):
            return 'Failure'

        print "Executing scoop"
        poses = [self.arm.get_current_pose().pose]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.206
        poses[-1].position.z += -0.048
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += -0.162
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += 0.212
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += -0.019
        poses[-1].position.z += -0.057
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += -0.027
        poses[-1].position.z += -0.042
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x += -0.277
        poses[-1].position.z += -0.103
        if not follow_path(self.arm, poses):
            return 'Failure'

        return 'Success'
