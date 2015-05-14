import rospy
import smach
from tf import TransformListener

from apc_msgs.msg import APCItem
from apc_util.collision import publish_pointcloud_collision
from apc_util.vision import take_sample, get_samples, process_samples
from apc_util.grasping import gripper
from apc_util.smach import on_exception


class ScanForItem(smach.State):
    """
    Scan the bin to find the items it contains.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin', 'item', 'contents'],
                             output_keys=['pose', 'points'])
        self.arm = robot.arm_left_torso
        self.tf = TransformListener(True, rospy.Duration(10.0))
        rospy.sleep(rospy.Duration(1.0))  # Wait for network timting

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        if not gripper.vision():
            return "Failure"

        for i in range(5):
            try:
                if not self.sample_bin(userdata.bin):
                    continue

                samples, success = get_samples(userdata.bin)
                if not success:
                    continue

                response, success = process_samples(
                    samples,
                    APCItem(
                        name=userdata.item,
                        bin=userdata.bin,
                        contents=userdata.contents
                    )
                )
                if not success:
                    continue

                if not publish_pointcloud_collision(response.result.collision_cloud):
                    rospy.logwarn("Failed to publish pointcloud collisions")
                    continue

                if not gripper.open():
                    return "Failure"

                userdata.pose = response.result.pose
                userdata.points = response.result.pointcloud
                return 'Success'
            except rospy.ServiceException as e:
                rospy.logwarn("Error process: "+str(e))
        rospy.logwarn("Can't find "+userdata.item+"...")
        return 'Failure'

    def sample_bin(self, bin):
        if not take_sample(command="reset", bin=bin):
            return False
        return take_sample(command="sample", bin=bin)
