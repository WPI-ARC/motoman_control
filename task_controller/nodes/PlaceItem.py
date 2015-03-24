import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from geometry_msgs.msg import Pose
from gripper_srv.srv import gripper, gripperRequest

from util import goto_pose, bin_pose

class PLACEITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input'], output_keys=['output'])
        self.arm = robot.arm_left
        self.gripper_control = rospy.ServiceProxy("command_gripper", gripper)

    def execute(self, userdata):
        rospy.loginfo("Trying to place...")
        print "Input data: " + str(userdata.input)

        # TODO: Remove?
        output = {}
        output['data'] = userdata.input
        output['error'] = "None"
        userdata.output = output

        pose = Pose()
        pose.position.x = 0.212677
        pose.position.y = 0.492155
        pose.position.z = 0.507941
        pose.orientation.x = -0.030455
        pose.orientation.y = -0.0479809
        pose.orientation.z = -0.790298
        pose.orientation.w = 0.610082

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        if not goto_pose(self.arm, pose, [10, 30, 60, 120]):
            return 'Failure'
        
        request = gripperRequest(command="open")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Open Gripper:", response
        return 'Success'
