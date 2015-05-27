import rospy
import smach
import numpy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from math import sqrt

from apc_util.collision import scene
from apc_util.grasping import plan_grasps, execute_grasp, gripper, generate_grasps, execute_wallgrasp_left, execute_wallgrasp_right
from apc_util.shelf import NO_SHELF, BIN, PADDED_SHELF, get_shelf_pose
from apc_util.smach import on_exception
from apc_util.transformation_helpers import *

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

        # self.show_grasps(grasps)

        #with PADDED_SHELF:
        with BIN(userdata.bin):
            # ischeezit = userdata.item == "cheezit_big_original"
            # grasps = plan_grasps(self.arm, grasps, ischeezit)

            grasps = plan_grasps(self.arm, grasps)

            try:
                grasp, plan_to_approach, plan_to_grasp, plan_to_retreat = grasps.next()
                rospy.loginfo("Grasp: %s" % grasp)
            except StopIteration:
                rospy.logwarn("No online grasps found.")
                # 
                min_x, max_x, min_y, max_y, min_z, _ = rospy.get_param(prefix+"/bins/"+userdata.bin)
                # TODO: Convert to base frame
                Tbase_shelf = numpy.array([PoseToMatrix(pose)]) # Transform form base to shelf
                min_y_base = numpy.dot(Tbase_shelf, numpy.array([[0], [min_y], [0], [1]]))[2]
                max_y_base = numpy.dot(Tbase_shelf, numpy.array([[0], [max_y], [0], [1]]))[2]
                min_x_base = numpy.dot(Tbase_shelf, numpy.array([[min_x], [0], [0], [1]]))[1]
                max_x_base = numpy.dot(Tbase_shelf, numpy.array([[max_x], [0], [0], [1]]))[1]
                min_z_base = numpy.dot(Tbase_shelf, numpy.array([[0], [0], [min_z], [1]]))[3]
                
                cutoff1, cutoff2 = min_y_base + (0.14 * sqrt(2)/2), max_y_base - (0.14 * sqrt(2)/2)
                if userdata.pose.position.y < cutoff1:
                    success = execute_wallgrasp_left(min_x_base, max_x_base, min_y_base, max_y_base, min_z_base)
                    if not success:
                        return 'Failure'
                elif userdata.pose.position.y < cutoff1:                    
                    pass  # TODO: center grasp
                else:
                    success = execute_wallgrasp_right(min_x_base, max_x_base, min_y_base, max_y_base, min_z_base)
                    if not success:
                        return 'Failure'
                return "Failure"

            if not execute_grasp(self.arm, grasp, plan_to_approach, plan_to_grasp, plan_to_retreat, shelf=NO_SHELF):
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
        scene.attach_box("arm_left_link_7_t", "Object", pose, [0.15, 0.23, 0.16], 
            ["hand_left_finger_1_link_1", "hand_left_finger_1_link_2", "hand_left_finger_1_link_3", "hand_left_finger_1_link_3_tip",
             "hand_left_finger_2_link_1", "hand_left_finger_2_link_2", "hand_left_finger_2_link_3", "hand_left_finger_2_link_3_tip",
             "hand_left_finger_middle_link_1", "hand_left_finger_middle_link_2", "hand_left_finger_middle_link_3", "hand_left_finger_middle_link_3_tip"])

        return 'Success'

    def show_grasps(self, grasps, filter=False):
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        if filter:
            # with PADDED_SHELF:
            with BIN(userdata.bin):
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
