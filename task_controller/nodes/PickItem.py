import roslib; roslib.load_manifest('task_controller')
import rospy
import smach


class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose'], output_keys=['output'])
        self.arm = robot.arm_left
        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_planning_time(30)
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")
        # TODO: Get real grasps
        result = self.arm.pick(userdata.item, [userdata.pose])
        print "Result: ", result
        userdata.output = userdata.input
        return 'Success'
