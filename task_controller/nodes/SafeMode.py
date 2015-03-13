import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

import sys

class SAFEMODE(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Safed'], input_keys=['input'], output_keys=['output'])

    def execute(self, userdata):
        if (userdata is None):
            rospy.logfatal("Forced safing")
        elif (userdata == []):
            rospy.loginfo("Safing after task completion")
        else:
            output = {}
            output['error'] = "Safed"
            output['data'] = None
            userdata.output = output
        rospy.loginfo("Switching Motoman to SAFEMODE")
        return 'Safed'
