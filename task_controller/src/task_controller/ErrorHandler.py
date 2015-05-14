import rospy
import smach

from apc_util.collision import remove_object
from apc_util.moveit import go_home
from apc_util.smach import on_exception
from apc_util.grasping import gripper
from apc_util.shelf import BIN


class ErrorHandler(smach.State):
    """
    Decide how to handle errors. Currently, it just gives up; we
    should probably act more intelligently.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Continue', 'Failed', 'Fatal'],
                             input_keys=['bin'])

    @on_exception(failure_state="Failed")
    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        remove_object()
        remove_object("pointcloud_voxels")
        gripper.open()
        if not go_home(BIN(userdata.bin)):
            return 'Failed'
        return 'Continue'
