import rospy
import smach

from apc_util.smach import on_exception


class SafeMode(smach.State):
    """
    Ensure that the robot is in a safe state before we shut down due
    to failure.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Safed'])

    @on_exception(failure_state="Failed")
    def execute(self, userdata):
        if (userdata is None):
            rospy.logfatal("Forced safing")
        elif (userdata == []):
            rospy.loginfo("Safing after task completion")
        rospy.loginfo("Switching Motoman to SAFEMODE")
        return 'Safed'
