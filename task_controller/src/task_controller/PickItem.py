import rospy
import smach

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

from apc_util.collision import attach_sphere
from apc_util.grasping import plan_grasps, execute_grasp, gripper, generate_grasps
from apc_util.shelf import NO_SHELF, BIN, PADDED_SHELF, get_shelf_pose
from apc_util.smach import on_exception


class PickItem(smach.State):
    """
    Figure out how to grasp an item and execute the appropriate grasp.
    """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['Success', 'Failure', 'Fatal'],
                             input_keys=['item', 'pose', 'points', 'bin'])
        self.arm = robot.arm_left_torso

        self.points = rospy.Publisher("/grasp_points", PointCloud2)

    @on_exception(failure_state="Failure")
    def execute(self, userdata):
        rospy.loginfo("Trying to pick '"+userdata.item+"'...")
        self.points.publish(userdata.points)

        pose = get_shelf_pose().pose
        # Note: Orientation for STL shelf differs from true shelf
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        grasps, success = generate_grasps(userdata.item, userdata.pose, pose,
                                          userdata.points, userdata.bin)
        if not success:
            return 'Failure'

        self.show_grasps(grasps)

        with BIN(userdata.bin):
            grasps = plan_grasps(self.arm, grasps)

            try:
                grasp, plan = grasps.next()
                rospy.loginfo("Grasp: %s" % grasp)
            except StopIteration:
                rospy.logwarn("No online grasps found.")
                return "Failure"

            if not execute_grasp(self.arm, grasp, plan, shelf=NO_SHELF):
                return "Failure"

        pose = PoseStamped()
        pose.header.frame_id = "/arm_left_link_7_t"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = -0.35
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        attach_sphere("arm_left_link_7_t", "Object", pose, 0.17, 
            ["hand_left_finger_1_link_1", "hand_left_finger_1_link_2", "hand_left_finger_1_link_3", "hand_left_finger_1_link_3_tip",
             "hand_left_finger_2_link_1", "hand_left_finger_2_link_2", "hand_left_finger_2_link_3", "hand_left_finger_2_link_3_tip",
             "hand_left_finger_middle_link_1", "hand_left_finger_middle_link_2", "hand_left_finger_middle_link_3", "hand_left_finger_middle_link_3_tip"])

        return 'Success'

    def show_grasps(self, grasps, filter=False):
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        if filter:
            with PADDED_SHELF:
                grasps = list(grasp for grasp, _ in plan_grasps(self.arm, grasps))
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
