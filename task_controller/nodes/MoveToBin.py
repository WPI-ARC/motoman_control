import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from util import goto_pose, bin_pose
from motoman_moveit.srv import convert_trajectory_server
from trajlib.srv import task

class MOVETOBIN(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['bin'], output_keys=[])
        self.arm = robot.arm_left
        self.move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
        self.trajlib = rospy.ServiceProxy("/trajectory_execute", task)

    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")
        pose = bin_pose(userdata.bin)
        print "Pose: ", pose

        plan = self.trajlib(task="Forward", bin_num=userdata.bin)
        # plan = self.trajlib(task="Drop", bin_num="A")
        print plan
        # print self.move(plan.joint_trajectory)
        return 'Success'
