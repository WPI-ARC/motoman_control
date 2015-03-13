import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from geometry_msgs.msg import PoseStamped


class SCANFORITEM(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'bin', 'item'], output_keys=['output', 'pose'])

    def execute(self, userdata):
        rospy.loginfo("Trying to find "+userdata.item+"...")

        rospy.sleep(10.0)

        # TODO: Get real pose from vision
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 0.25
        pose.pose.position.z = 1.675
        pose.pose.orientation.x = 1
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        userdata.pose = pose

        userdata.output = userdata.input
        return 'Success'
