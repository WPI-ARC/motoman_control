import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from geometry_msgs.msg import PoseStamped

from apc_vision.srv import *

class SCANFORITEM(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'bin', 'item'], output_keys=['output', 'pose'])
        self.sample = rospy.ServiceProxy("sample_vision", SampleVision)
        self.process = rospy.ServiceProxy("process_vision", ProcessVision)

    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        # TODO: Remove?
        output = {}
        output['data'] = userdata.input
        output['error'] = "None"
        userdata.output = output

        sample_request = SimpleVisionRequest(
            command=userdata.bin
        )
        process_request = ProcessVisionRequest(
            bin=userdata.bin,
            object=userdata.item
        )
        for i in range(5):
            response = self.sample.call(sample_request)
            print "Sample:", response
            response = self.process.call(process_request)
            print "Process:", response
            if response.found:
                response.pose.pose.orientation.x = -0.484592
                response.pose.pose.orientation.y = 0.384602
                response.pose.pose.orientation.z = 0.615524
                response.pose.pose.orientation.w = -0.488244
                userdata.pose = response.pose
                userdata.output = userdata.input
                return 'Success'
        rospy.logwarn("Can't find "+userdata.item+"...")
        return 'Failure'

