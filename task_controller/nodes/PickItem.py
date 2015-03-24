import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from copy import deepcopy
from grasp_logic.srv import grasp, graspRequest
from gripper_srv.srv import gripper, gripperRequest

from util import follow_path, bin_pose

class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose', 'bin'], output_keys=['output'])
        self.arm = robot.arm_left
        self.grasp_generator = rospy.ServiceProxy("grasp_logic", grasp)
        self.gripper_control = rospy.ServiceProxy("command_gripper", gripper)

        request = gripperRequest(command="activate")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Activate Gripper:", response

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")
        userdata.output = userdata.input

        request = graspRequest(
            object=userdata.item,
            obj_pose=userdata.pose
        )
        # TODO: Handle response error
        response = self.grasp_generator.call(request)
        pose = response.arm_pose.pose
        print "Grasp Pose:", response

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        
        print "Moving to grasp pose"
        up = deepcopy(self.arm.get_current_pose().pose)
        up.position.z += 0.03 # 3cm
        if not follow_path(self.arm, [self.arm.get_current_pose().pose, up, pose]):
            return 'Failure'

        request = gripperRequest(command="close")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Close Gripper:", response

        print "Retreat from grasp"
        up = deepcopy(self.arm.get_current_pose().pose)
        up.position.z += 0.03 # 3cm
        pose = bin_pose(userdata.bin).pose
        pose.position.x -= 0.1
        if not follow_path(self.arm, [self.arm.get_current_pose().pose, up, pose]):
            return 'Failure'

        return 'Success'
