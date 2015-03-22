import roslib; roslib.load_manifest('task_controller')
import rospy
import smach

from grasp_logic.srv import grasp, graspRequest
from gripper_srv.srv import gripper, gripperRequest

from MoveToBin import bin_pose

class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose', 'bin'], output_keys=['output'])
        self.arm = robot.arm_left
        self.grasp_generator = rospy.ServiceProxy("grasp_logic", grasp)
        self.gripper_control = rospy.ServiceProxy("command_gripper", gripper)

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")

        request = graspRequest(
            object=userdata.item,
            obj_pose=userdata.pose
        )
        # TODO: Handle response error
        response = self.grasp_generator.call(request)
        pose = response.arm_pose.pose
        print "Grasp Pose:", response

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

        for t in [1, 5, 30, 60]:
            self.arm.set_planning_time(t)
            rospy.loginfo("Planning for "+str(t)+" seconds...")
            result = self.arm.go(pose)
            print "Move: ", result
            if result:
                break
        if not result:
            return 'Failure'

        request = gripperRequest(command="close")
        # TODO: Handle response error
        response = self.gripper_control.call(request)
        print "Close Gripper:", response

        pose.position.z += 0.03
        for t in [1, 5, 30, 60]:
            self.arm.set_planning_time(t)
            rospy.loginfo("Planning for "+str(t)+" seconds...")
            result = self.arm.go(pose)
            print "Lift: ", result
            if result:
                break
        if not result:
            return 'Failure'

        pose = bin_pose(userdata.bin)
        pose.pose.position.x -= 0.1
        print "Pose: ", pose
        for t in [1, 5, 30, 60]:
            self.arm.set_planning_time(t)
            rospy.loginfo("Planning for "+str(t)+" seconds...")
            result = self.arm.go(pose.pose)
            print "Backout: ", result
            if result:
                break
        if not result:
            return 'Failure'


        userdata.output = userdata.input
        return 'Success'
