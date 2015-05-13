import rospy
import smach
import numpy
from math import cos, sin, pi
from tf import TransformListener
from apc_util.transformation_helpers import PoseToMatrix, PoseFromMatrix

from apc_vision.srv import *
from apc_vision.msg import *
from apc_msgs.msg import *
from apc_util.srv import *
from apc_util.moveit import follow_path


class ScanForItem(smach.State):
    """
    Scan the bin to find the items it contains.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin', 'item', 'contents'],
                             output_keys=['pose', 'points'])
        self.arm = robot.arm_left_torso
        self.take_sample = rospy.ServiceProxy("take_sample", TakeSample)
        self.get_samples = rospy.ServiceProxy("get_samples", GetSamples)
        self.process_samples = rospy.ServiceProxy("process_samples", ProcessSamples)
        self.publish_pointcloud_collision = rospy.ServiceProxy("publish_pointcloud_collision", PublishPointcloudCollision)
        self.tf = TransformListener(True, rospy.Duration(10.0))
        rospy.sleep(rospy.Duration(1.0))  # Wait for network timting

    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        from gripper_srv.srv import gripper
        self.gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
        response = self.gripper_control.call(command="close")
        print "Activate Gripper:", response

        for i in range(5):
            try:
                self.sample_bin(userdata.bin)
                samples = self.get_samples(bin=userdata.bin)
                if samples.status != GetSamplesResponse.SUCCESS:
                    return 'Failure'
                response = self.process_samples(SampleArray(
                    samples=samples.samples,
                    order=APCItem(
                        name=userdata.item,
                        bin=userdata.bin,
                        contents=userdata.contents
                    )
                ))
                if response.result.status != ProcessedObject.SUCCESS:
                    return 'Failure'
                userdata.pose = response.result.pose
                userdata.points = response.result.pointcloud
                self.publish_pointcloud_collision(response.result.collision_cloud)
                return 'Success'
            except rospy.ServiceException as e:
                rospy.logwarn("Error process: "+str(e))
        rospy.logwarn("Can't find "+userdata.item+"...")
        return 'Failure'

    def sample_bin(self, bin):

        print "Reset:", self.take_sample(command="reset", bin=bin)
        result = self.take_sample(command="sample", bin=bin)
        if result.status != TakeSampleResponse.SUCCESS:
            print "Sample:", result
            return False
        return True
