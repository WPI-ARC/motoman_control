#!/usr/bin/python

import sys
from copy import deepcopy

import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import moveit_commander
import tf
import tf2_ros
import geometry_msgs.msg

sys.path.append("/home/wpi-apc/catkin_ws/src/task_controller/nodes")
from copy import deepcopy
from grasp_logic.srv import grasp, graspRequest
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
from geometry_msgs.msg import Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray

from apc_vision.srv import *
from util import goto_pose, follow_path, bin_pose, filterGrasps, execute_grasp, add_object, remove_object

item = "crayola_64_ct"
# item = "cheezit_big_original"
# item = "feline_greenies_dental_treats"
item = "mark_twain_huckleberry_finn"

rospy.loginfo("Initializing...")
robot = moveit_commander.RobotCommander()
arm = robot.arm_left
rospy.init_node("motoman_apc_controller")
remove_object()

tfs = []

rospy.loginfo("Trying to move to initial pose...")
pose = Pose(position=Point(x=-0.2463, y=0.68031, z=0.92698),
            orientation=Quaternion(x=0.37608, y=-0.11588, z=-0.4737, w=0.78787))
print "Pose: ", pose

arm.set_planner_id("RRTstarkConfigDefault")
arm.set_workspace([-3, -3, -3, 3, 3, 3])
if not goto_pose(arm, pose, [1, 5, 30, 60]):
    sys.exit(1)


rospy.loginfo("Trying to find "+item+"...")
sample = rospy.ServiceProxy("sample_vision", SampleVision)
process = rospy.ServiceProxy("process_vision", ProcessVision)

sample_request = SampleVisionRequest(
    command="Table"
)
process_request = ProcessVisionRequest(
    bin="Table",
    object=item
)
response = sample.call(sample_request)
print "Sample:", response
response = process.call(process_request)
print "Process:", response

object_pose = response.pose.pose
print object_pose
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "object"
t.transform.translation = object_pose.position
t.transform.rotation = object_pose.orientation
tfs.append(t)

client = rospy.ServiceProxy('getGrasps', apcGraspDB)
response = client.call(item=item, Trob_obj=object_pose)
print "Response:", response.status
print response.apcGraspArray

grasps = filterGrasps(arm, response.apcGraspArray.grasps)
# grasps = response.apcGraspArray.grasps

# for i in range(len(grasps)):
# 	grasp = grasps[i].posegrasp
# 	approach = grasps[i].poseapproach
# 	t = geometry_msgs.msg.TransformStamped()
# 	t.header.stamp = rospy.Time.now()
# 	t.header.frame_id = "base_link"
# 	t.child_frame_id = "grasp "+str(i)
# 	t.transform.translation = grasp.position
# 	t.transform.rotation = grasp.orientation
# 	tfs.append(t)
# 	t = geometry_msgs.msg.TransformStamped()
# 	t.header.stamp = rospy.Time.now()
# 	t.header.frame_id = "base_link"
# 	t.child_frame_id = "approach "+str(i)
# 	t.transform.translation = approach.position
# 	t.transform.rotation = approach.orientation
# 	tfs.append(t)


# br = tf2_ros.TransformBroadcaster()
# rate = rospy.Rate(250.0)
# while (not rospy.is_shutdown()):
#     for t in tfs:
#         t.header.stamp = rospy.Time.now()
#         br.sendTransform(t)
#     rate.sleep()

i = 0
print "Showing grasp %s of %s" %(i, len(grasps))
grasp = grasps[i].posegrasp
approach = grasps[i].poseapproach
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "grasp "+str(i)
t.transform.translation = grasp.position
t.transform.rotation = grasp.orientation
tfs.append(t)
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "approach "+str(i)
t.transform.translation = approach.position
t.transform.rotation = approach.orientation
tfs.append(t)



if not rospy.is_shutdown():
	raw_input("Continue? ")
	print "Success:", execute_grasp(arm, grasps[i], object_pose)