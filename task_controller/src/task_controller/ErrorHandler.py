import rospy
import smach

from apc_util.collision import remove_object

class ErrorHandler(smach.State):
    """
    Decide how to handle errors. Currently, it just gives up; we
    should probably act more intelligently.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Continue', 'Failed', 'Fatal'])

    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        remove_object()
        remove_object("pointcloud_voxels")
        return 'Continue'
