import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import tf2_ros

from gripper_srv.srv import gripper
from grasp_planner.srv import apcGraspDB
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2

from apc_util.grasping import filterGrasps, execute_grasp
from apc_util.shelf import Shelf, FULL_SHELF


class PICKITEM(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['item', 'pose', 'points', 'bin'])
        self.arm = robot.arm_left_torso
        # self.arm = robot.arm_left
        # self.grasp_generator = rospy.ServiceProxy('getGrasps" apcGraspDB)
        self.grasp_generator = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
        self.gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
        self.points = rospy.Publisher("/grasp_points", PointCloud2)

        # TODO: Handle response error
        response = self.gripper_control.call(command="activate")
        print "Activate Gripper:", response

    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")
        self.points.publish(userdata.points)

        # TODO: Handle response error
        response = self.grasp_generator(
            item=userdata.item,
            object_pose=userdata.pose.pose,
            object_points=userdata.points,
            bin=userdata.bin,
        )
        # for grasp in response.grasps.grasps:
        #     grasp.poseapproach = deepcopy(grasp.pregrasp)
        #     grasp.poseapproach.position.x -= 0.2
        # random.shuffle(response.grasps.grasps)

        grasps = response.grasps.grasps
        grasps = list(filterGrasps(self.arm, response.grasps.grasps))
        print "Grasp:", grasps[0]
        tfs = []
        for i in range(len(grasps)):
            grasp = grasps[i].pregrasp
            approach = grasps[i].approach
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "grasp "+str(i)
            t.transform.translation = grasp.position
            t.transform.rotation = grasp.orientation
            tfs.append(t)
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "approach "+str(i)
            t.transform.translation = approach.position
            t.transform.rotation = approach.orientation
            tfs.append(t)

        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(250.0)
        while (not rospy.is_shutdown()):
            for t in tfs:
                t.header.stamp = rospy.Time.now()
                br.sendTransform(t)
            rate.sleep()

        with Shelf(FULL_SHELF):
            grasps = filterGrasps(self.arm, response.grasps.grasps)
            try:
                grasp = grasps.next()
            except StopIteration:
                rospy.logwarn("No online grasps found.")
                return "Failure"
            grasps = [grasp]
            print "Grasp:", grasps[0]

            self.arm.set_planner_id("RRTstarkConfigDefault")
            self.arm.set_workspace([-3, -3, -3, 3, 3, 3])

            if not execute_grasp(self.arm, grasps[0], userdata.pose.pose):
                return "Failure"

            return 'Success'