import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from grasp_logic.srv import grasp, graspRequest
from gripper_srv.srv import gripper, gripperRequest


class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose'], output_keys=['output'])
        self.arm = robot.arm_left
        self.grasp_generator = rospy.ServiceProxy("grasp_logic", grasp)
        self.gripper_control = rospy.ServiceProxy("command_gripper", gripper)

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")

        request = graspRequest(
            object=userdata.item,
            pose=userdata.pose
        )
        # TODO: Handle response error
        response = self.grasp_generator.call(request)
        print "Grasp Pose:", response

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_planning_time(30)
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

        result = self.arm.go(response.arm_pose)
        print "Move: ", result

        request = gripperRequest(command="close")
        # TODO: Handle response error
        response = self.grasp_generator.call(request)
        print "Close Gripper:", response

        # TODO: Cartesian jog up by 3cm

        userdata.output = userdata.input
        return 'Success'
