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
import sensor_msgs.msg
import moveit_commander
from numpy import *
from grasp_planner.srv import apcGraspDB, apcGraspDBResponse
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from grasp_planner.msg import apcGraspPose, apcGraspArray
from sensor_msgs.msg import PointCloud2

def goto_pose(group, pose, times=[5, 20, 30], with_shelf=False):
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

def follow_path(group, path, collision_checking=False):
    """Follows a cartesian path using a linear interpolation of the given
    `path`. The `collision_checking` parameter controls whether or not
    to check the path for collisions with the environment."""
    traj, success = group.compute_cartesian_path(
        path,
        0.01,  # 1cm interpolation resolution
        0.0,  # jump_threshold disabled
        avoid_collisions=collision_checking,
    )
    if success < 1:
        rospy.logwarn("Cartesian trajectory could not be completed. Only solved for: '"+str(success)+"'...")
        #return False
    return group.execute(traj)

def main():
    try:
        rospy.loginfo("Initializing...")
        robot = moveit_commander.RobotCommander()
        arm = robot.arm_left
        rospy.init_node("online_planner_test")
        client = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
        item = 'expo_dry_erase_board_eraser' # Set response item
        # item = 'cheezit_big_original'
        tfs = []
        pts = []

        msg = geometry_msgs.msg.Pose()
        msg.position.x = 0.885315
        msg.position.y = 0.413907
        msg.position.z = 0.787417
        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 1



        points = sensor_msgs.msg.PointCloud2()
        points.data = pts
        binnum = "B"

        response = client(item, binnum, msg, points)
        print "returned pose"
        print response.status
        print response.grasps


        grasps = response.grasps.grasps
        for i in range(len(grasps)):
            grasp = grasps[i].pregrasp        
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "grasp"+str(i)
            t.transform.translation = grasp.position
            t.transform.rotation = grasp.orientation
            tfs.append(t)
            approach = grasps[i].approach
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "approach"+str(i)
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

        i = 0
        grasp = grasps[i].posegrasp        
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "grasp"+str(i)
        t.transform.translation = grasp.position
        t.transform.rotation = grasp.orientation
        tfs.append(t)
        approach = grasps[i].poseapproach
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "approach"+str(i)
        t.transform.translation = approach.position
        t.transform.rotation = approach.orientation
        tfs.append(t)
        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(1000)        
        for time in range(0,1000):
            for tf in tfs:
                tf.header.stamp = rospy.Time.now()
                br.sendTransform(tf)                
            rate.sleep()

        rospy.loginfo("Trying to move to initial pose...")
        approachpose = grasps[i].poseapproach
        pregrasppose = grasps[i].posegrasp
        
        arm.set_planner_id("RRTstarkConfigDefault")
        arm.set_workspace([-3, -3, -3, 3, 3, 3])
        # if not goto_pose(arm, approachpose, [1, 5, 30]): #move to approach
        #     sys.exit(1)

        # if not goto_pose(arm, pregrasppose, [1, 5, 30]): #move to pregrasp
        #     sys.exit(1)
        

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print traceback.format_exc()
                
if __name__ == '__main__':    
    main()
