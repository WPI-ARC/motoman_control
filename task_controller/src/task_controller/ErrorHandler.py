import roslib; roslib.load_manifest('task_controller')
import rospy
import smach


class ErrorHandler(smach.State):
    """
    Decide how to handle errors. Currently, it just gives up; we
    should probably act more intelligently.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['ReMove', 'ReScan', 'RePick', 'ReFinish', 'Failed', 'Fatal'])

    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        return 'Fatal'
