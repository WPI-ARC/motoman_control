import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from grasp_logic.srv import grasp, graspRequest
from gripper_srv.srv import gripper, gripperRequest

from util import goto_pose, bin_pose

class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose', 'bin'], output_keys=['output'])
        self.arm = robot.arm_left
        self.grasp_generator = rospy.ServiceProxy("grasp_logic", grasp)
        self.gripper_control = rospy.ServiceProxy("command_gripper", gripper)

        request = gripperRequest(command="ACTIVATE")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Activate Gripper:", response

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")

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
        if not goto_pose(self.arm, pose, [10, 30, 60]):
            return 'Failure'

        request = gripperRequest(command="close")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Close Gripper:", response

        print "Lifting Item"
        pose.position.z += 0.03
        if not goto_pose(self.arm, pose, [10, 30, 60]):
            return 'Failure'

        pose = bin_pose(userdata.bin)
        pose.pose.position.x -= 0.1
        print "Backing Out"
        print "Pose: ", pose
        if not goto_pose(self.arm, pose.pose, [10, 30, 60]):
            return 'Failure'

        userdata.output = userdata.input
        return 'Success'
