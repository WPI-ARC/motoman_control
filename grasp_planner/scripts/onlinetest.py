#!/usr/bin/env python

import tf
import tf2_ros
import math
import rospy
import sys
import traceback
import time
import openravepy
import geometry_msgs.msg
import moveit_commander
from numpy import *
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray

def goto_pose(group, pose, times=[5, 20, 40, 60], with_shelf=False):
    """Moves the hand to a given `pose`, using the configured `group`. The
    planning time is modified based on the passed in `times` to try to
    plan quickly if possible, but fall back on longer plans if
    necessary. If `add_shelf` is true, a box model of the shelf is
    added to the environment to avoid collisions."""
    if with_shelf:
        add_shelf()
    for t in times:
        group.set_planning_time(t)
        rospy.loginfo("Planning for "+str(t)+" seconds...")
        result = group.go(pose)
        if result:
            if with_shelf:
                remove_shelf()
            return True
    if with_shelf:
        remove_shelf()
    return False

def main():
    try:
        rospy.loginfo("Initializing...")
        robot = moveit_commander.RobotCommander()
        arm = robot.arm_left
        rospy.init_node("online_planner_test")
        client = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
        item = 'expo_dry_erase_board_eraser' # Set response item
        item = 'cheezit_big_original'
        tfs = []

        msg = geometry_msgs.msg.Pose()
        msg.position.x = 0
        msg.position.y = 0
        msg.position.z = 0
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 1        

        response = client(item, msg)
        print "returned pose"
        print response.status
        print response.apcGraspArray

        grasps = response.apcGraspArray.grasps
        # for i in range(len(grasps)):
        #     grasp = grasps[i].posegrasp
        #     approach = grasps[i].poseapproach
        #     t = geometry_msgs.msg.TransformStamped()
        #     t.header.stamp = rospy.Time.now()
        #     t.header.frame_id = "base_link"
        #     t.child_frame_id = "grasp "+str(i)
        #     t.transform.translation = grasp.position
        #     t.transform.rotation = grasp.orientation
        #     tfs.append(t)
        #     t = geometry_msgs.msg.TransformStamped()
        #     t.header.stamp = rospy.Time.now()
        #     t.header.frame_id = "base_link"
        #     t.child_frame_id = "approach "+str(i)
        #     t.transform.translation = approach.position
        #     t.transform.rotation = approach.orientation
        #     tfs.append(t)

        # br = tf2_ros.TransformBroadcaster()
        # rate = rospy.Rate(250.0)
        # while (not rospy.is_shutdown()):
        #     for t in tfs:
        #         t.header.stamp = rospy.Time.now()
        #         br.sendTransform(t)
        #     rate.sleep()

        i = 0
        grasp = grasps[i].posegrasp
        approach = grasps[i].poseapproach
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "grasp "+str(i)
        t.transform.translation = grasp.position
        t.transform.rotation = grasp.orientation

        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(1000)        
        for time in range(0,10000):
            t.header.stamp = rospy.Time.now()
            br.sendTransform(t)                
            rate.sleep()

        rospy.loginfo("Trying to move to initial pose...")
        grasp = grasps[i].poseapproach
        pose = geometry_msgs.msg.Pose()
        pose.position = grasp.position
        pose.orientation = grasp.orientation
        print "Pose: ", pose

        
        arm.set_planner_id("RRTstarkConfigDefault")
        arm.set_workspace([-3, -3, -3, 3, 3, 3])
        if not goto_pose(arm, pose, [1, 5, 30, 60]):
            sys.exit(1)
        

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print traceback.format_exc()
                
if __name__ == '__main__':    
    main()
