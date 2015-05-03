import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

import sys

class SAFEMODE(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Safed'])

    def execute(self, userdata):
        if (userdata is None):
            rospy.logfatal("Forced safing")
        elif (userdata == []):
            rospy.loginfo("Safing after task completion")
        rospy.loginfo("Switching Motoman to SAFEMODE")
        return 'Safed'
