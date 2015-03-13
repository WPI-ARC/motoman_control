import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from geometry_msgs.msg import PoseStamped


class MOVETOBIN(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'bin'], output_keys=['output'])
        self.arm = robot.arm_left
        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_planning_time(30)
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

    def execute(self, userdata):
        rospy.loginfo("Trying to move to bin '"+userdata.bin+"'...")
        print "Input data: " + str(userdata.input)
        pose = bin_pose(userdata.bin)
        print "Pose: ", pose
        result = self.arm.go(pose.pose)
        print "Result: ", result

        # TODO: Remove?
        output = {}
        output['data'] = userdata.input
        output['error'] = "None"
        userdata.output = output

        if not result:
            return 'Failure'
        return 'Success'


# Currently only returns bin A
# TODO: Support all bins
def bin_pose(bin, bin_x=1.32, bin_y=0, bin_z=0):
    # Gripper dimension
    GripperLength = 0.2

    # Bin dimension Unit m
    Bin_depth = 0.430
    Start_Gap = 0.100

    LeftBin_width = 0.240
    MiddleBin_width = 0.300
    RightBin_width = 0.240

    WorkBase_Height = 0.820
    BottomLay_Height = 0.230
    SecndLayer_Height = 0.230
    ThirdLayer_Height = 0.220
    TopLayer_Height = 0.260

    Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2
    Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2

    TopLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height + TopLayer_Height/2
    ThirdLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height/2
    SecndLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height/2
    BottomLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height/2

    Entry_X_shiftvalue = bin_x - Bin_depth - Start_Gap - GripperLength

    # Setting Configuration:

    # 	1		2		3
    # 	4		5		6
    # 	7		8		9
    # 	10		11		12
    # 		   Base	

    pose = PoseStamped()
    pose.pose.position.x = Entry_X_shiftvalue
    pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
    pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    pose.pose.orientation.x = 0.5
    pose.pose.orientation.y = -0.5
    pose.pose.orientation.z = -0.5
    pose.pose.orientation.w = 0.5

    return pose
