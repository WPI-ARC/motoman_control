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
                             input_keys=['input', 'item', 'pose', 'bin'],
                             output_keys=['output'])
        self.arm = robot.arm_left
        self.grasp_generator = rospy.ServiceProxy("grasp_logic", grasp)
        self.gripper_control = rospy.ServiceProxy("command_gripper", gripper)

        #request = gripperRequest(command="activate")
        # TODO: Handle response error
        #response = self.gripper_control.call(request)
        #print "Activate Gripper:", response

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
        #pose.orientation = self.arm.get_current_pose().pose.orientation
        print "Grasp Pose:", response

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

        print "Moving to grasp pose"
        out = self.arm.get_current_pose().pose
        up = deepcopy(out)
        up.position.z += 0.03  # 3cm
        target = deepcopy(pose)
        target.position.x = up.position.x + 0.3  # 2cm
        target.position.z = up.position.z - 0.02  # 2cm
        if not follow_path(self.arm, [out, up, target]):
            return 'Failure'

        request = gripperRequest(command="close")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Close Gripper:", response

        rospy.sleep(rospy.Duration(1, 0)) # TODO: Shouldn't need

        print "Retreat from grasp"
        lift = deepcopy(self.arm.get_current_pose().pose)
        lift.position.z += 0.03
        up.position.x -= 0.15
        up.position.z += 0.02
        if not follow_path(self.arm, [self.arm.get_current_pose().pose, lift, up]):
            return 'Failure'

        rospy.sleep(rospy.Duration(1, 0)) # TODO: Shouldn't need

        return 'Success'
