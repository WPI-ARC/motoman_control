import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
from tf import TransformListener

from apc_vision.srv import *
from apc_vision.msg import *


class SCANFORITEM(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'bin', 'item'],
                             output_keys=['output', 'pose', 'points'])
        self.sample = rospy.ServiceProxy("sample_vision", SampleVision)
        self.process = rospy.ServiceProxy("process_vision", ProcessVision)
        self.tf = TransformListener(True, rospy.Duration(10.0))
        rospy.sleep(rospy.Duration(1.0))  # Wait for network timting

    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        # TODO: Remove?
        output = {}
        output['data'] = userdata.input
        output['error'] = "None"
        userdata.output = output

        for i in range(5):
            print "Reset:", self.sample(command="reset")
            print "Sample:", self.sample(command=userdata.bin)
            try:
                response = self.process(
                    bin=userdata.bin,
                    target=APCObject(name=userdata.item, number=1),
                    objects=[],
                )
                userdata.pose = self.tf.transformPose("/base_link", response.pose)
                userdata.points = response.object_points
                print "Pose:", self.tf.transformPose("/base_link", response.pose)
                userdata.output = userdata.input
                return 'Success'
            except rospy.ServiceException as e:
                rospy.logwarn("Error process: "+str(e))
        rospy.logwarn("Can't find "+userdata.item+"...")
        return 'Failure'
