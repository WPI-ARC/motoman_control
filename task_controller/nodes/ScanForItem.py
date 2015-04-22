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

        for i in range(5):
            response = self.sample.call(command=userdata.bin)
            print "Sample:", response
            response = self.process.call(
                bin=userdata.bin,
                object1=userdata.item,
                object2=userdata.item,
            )
            print "Process:", response
            if True or response.found:
                response.pose1.pose.orientation.x = -0.484592
                response.pose1.pose.orientation.y = 0.384602
                response.pose1.pose.orientation.z = 0.615524
                response.pose1.pose.orientation.w = -0.488244
                userdata.pose = response.pose1
                userdata.output = userdata.input
                return 'Success'
        rospy.logwarn("Can't find "+userdata.item+"...")
        return 'Failure'

