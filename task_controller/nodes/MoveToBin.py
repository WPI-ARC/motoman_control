import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from util import goto_pose, bin_pose

class MOVETOBIN(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'bin'], output_keys=['output'])
        self.arm = robot.arm_left

    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")
        print "Input data: " + str(userdata.input)
        pose = bin_pose(userdata.bin)
        print "Pose: ", pose

        # TODO: Remove?
        output = {}
        output['data'] = userdata.input
        output['error'] = "None"
        userdata.output = output

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])
        if goto_pose(self.arm, pose.pose, [1, 5, 30, 60]):
            return 'Success'
        else:
            return 'Failure'


