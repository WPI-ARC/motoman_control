import rospy
import smach

from apc_util.smach import on_exception


class FinishTask(smach.State):
    """
    Cleanup after a successful execution before finishing.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'])

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        rospy.loginfo("Trying to finish up the task...")
        return 'Success'
