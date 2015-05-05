import roslib; roslib.load_manifest('task_controller')
import rospy
import smach


class FinishTask(smach.State):
    """
    Cleanup after a successful execution before finishing.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'])

    def execute(self, userdata):
        rospy.loginfo("Trying to finish up the task...")
        return 'Success'
