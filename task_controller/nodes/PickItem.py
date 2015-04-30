import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import tf2_ros

from gripper_srv.srv import gripper
from grasp_planner.srv import apcGraspDB
from geometry_msgs.msg import TransformStamped

from util.grasping import filterGrasps, execute_grasp


class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['input', 'item', 'pose', 'bin'],
                             output_keys=['output'])
        self.arm = robot.arm_left_torso
        # self.grasp_generator = rospy.ServiceProxy('getGrasps" apcGraspDB)
        self.grasp_generator = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
        self.gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)

        # TODO: Handle response error
        response = self.gripper_control.call(command="activate")
        print "Activate Gripper:", response

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")
        userdata.output = userdata.input

        # TODO: Handle response error
        response = self.grasp_generator(
            item=userdata.item,
            Trob_obj=userdata.pose.pose
        )
        # for grasp in response.apcGraspArray.grasps:
        #     grasp.poseapproach = deepcopy(grasp.posegrasp)
        #     grasp.poseapproach.position.x -= 0.2
        # random.shuffle(response.apcGraspArray.grasps)

        grasps = filterGrasps(self.arm, response.apcGraspArray.grasps)
        grasp = grasps.next()
        grasps = [grasp]
        print "Grasp:", grasps[0]

        # grasps = response.apcGraspArray.grasps
        # grasps = list(filterGrasps(self.arm, response.apcGraspArray.grasps))
        # # grasps = response.apcGraspArray.grasps
        # print "Grasp:", grasps[0]
        # tfs = []
        # for i in range(len(grasps)):
        #     grasp = grasps[i].posegrasp
        #     approach = grasps[i].poseapproach
        #     t = TransformStamped()
        #     t.header.stamp = rospy.Time.now()
        #     t.header.frame_id = "base_link"
        #     t.child_frame_id = "grasp "+str(i)
        #     t.transform.translation = grasp.position
        #     t.transform.rotation = grasp.orientation
        #     tfs.append(t)
        #     t = TransformStamped()
        #     t.header.stamp = rospy.Time.now()
        #     t.header.frame_id = "base_link"
        #     t.child_frame_id = "approach "+str(i)
        #     t.transform.translation = approach.position
        #     t.transform.rotation = approach.orientation
        #     tfs.append(t)

        # i = 0
        # grasp = grasps[i].posegrasp
        # approach = grasps[i].poseapproach
        # t = TransformStamped()
        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "base_link"
        # t.child_frame_id = "grasp "+str(i)
        # t.transform.translation = grasp.position
        # t.transform.rotation = grasp.orientation
        # tfs.append(t)
        # t = TransformStamped()
        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "base_link"
        # t.child_frame_id = "approach "+str(i)
        # t.transform.translation = approach.position
        # t.transform.rotation = approach.orientation
        # tfs.append(t)

        # br = tf2_ros.TransformBroadcaster()
        # rate = rospy.Rate(250.0)
        # while (not rospy.is_shutdown()):
        #     for t in tfs:
        #         t.header.stamp = rospy.Time.now()
        #         br.sendTransform(t)
        #     rate.sleep()

        self.arm.set_planner_id("RRTstarkConfigDefault")
        self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

        if not execute_grasp(self.arm, grasps[0], userdata.pose.pose):
            return "Failure"

        return 'Success'
