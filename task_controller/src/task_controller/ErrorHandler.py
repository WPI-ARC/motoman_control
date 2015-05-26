import rospy
import smach

from apc_util.collision import remove_object
from apc_util.moveit import go_home, robot_state
from apc_util.smach import on_exception
from apc_util.grasping import gripper
from apc_util.shelf import BIN
from apc_msgs.msg import ScheduleItem


class ErrorHandler(smach.State):
    """
    Decide how to handle errors. Currently, it just gives up; we
    should probably act more intelligently.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['Continue', 'Failed', 'Fatal'],
                             input_keys=['action', 'item', 'bin', 'contents'], output_keys=['new_information'])

    @on_exception(failure_state="Failed")
    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")

        # Suggest scooping if grasping failed
        if userdata.action == "grab":
            userdata.new_information = ScheduleItem(
                action="scoop",
                name=userdata.name,
                bin=userdata.bin,
                contents=userdata.contents,
            )

        while robot_state.is_stopped():
            rospy.logwarn("Waiting until e-stop is removed to handle errors")
            rospy.sleep(1)

        rospy.loginfo("Removing objects that could be causing problems")
        remove_object()
        remove_object("pointcloud_voxels")

        rospy.loginfo("Setting gripper to open state for safety")
        gripper.open()

        if robot_state.has_error():
            rospy.logerr("Robot alarm is set, going home should clear it")
        if not go_home(shelf=BIN(userdata.bin)):
            rospy.logfatal("Robot should continue going home forever")
            return 'Failed'
        if robot_state.has_error():
            rospy.logfatal("Robot alarm is set, should be cleared by now")
        return 'Continue'
