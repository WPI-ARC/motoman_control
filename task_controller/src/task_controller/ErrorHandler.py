import roslib; roslib.load_manifest('task_controller')
import rospy
import smach


class ERRORHANDLER(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ReMove', 'ReScan', 'RePick', 'ReFinish', 'Failed', 'Fatal'],)

    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        return 'Fatal'
