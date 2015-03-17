import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from geometry_msgs.msg import PoseStamped

from apc_vision.srv import *

class SCANFORITEM(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'bin', 'item'], output_keys=['output', 'pose'])
        self.detector = rospy.ServiceProxy("object_detect", ObjectDetect)

    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        request = ObjectDetectRequest(
            bin=userdata.bin,
            object=userdata.item
        )
        # TODO: Handle response error
        response = self.detector.call(request)
        print response
        userdata.pose = response.pose
        userdata.output = userdata.input
        return 'Success'

