import rospy
import smach

from apc_util.moveit import go_home
from apc_util.smach import on_exception


class MoveToHome(smach.State):
    """
    Moves to the desired bin, so that relevant tasks can be performed.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        if not go_home():
            return 'Failure'
        return 'Success'
