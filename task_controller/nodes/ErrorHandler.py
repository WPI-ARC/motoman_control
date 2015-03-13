import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros

class ERRORHANDLER(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ReMove','ReScan','RePick','ReFinish','Failed','Fatal'],
                             input_keys=['input'], output_keys=['output'])

    def execute(self, userdata):
        rospy.loginfo("ErrorHandler executing...")
        userdata.output = userdata.input
        rospy.sleep(5.0)
        return 'Fatal'
