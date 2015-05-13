#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy

import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld

from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    scene = moveit_commander.PlanningSceneInterface()
    scene_topic = rospy.Publisher("planning_scene", PlanningScene)
    rospy.init_node('publish_boxshelf', anonymous=True)
    # while scene._pub_co.get_num_connections() == 0:
    #     rospy.sleep(1)
    rospy.sleep(10)

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = -0.505
    pose.pose.position.y = 0
    pose.pose.position.z = 1.5
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="back wall",
        pose=pose,
        size=(0.01, 6, 3)
    )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = -1.4
    pose.pose.position.z = 1.5
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="side wall",
        pose=pose,
        size=(6, 0.01, 3)
    )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = -0.051
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="floor",
        pose=pose,
        size=(6, 6, 0.1)
    )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0.5
    pose.pose.position.y = 0.1
    pose.pose.position.z = 0.25
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="dropoff",
        pose=pose,
        size=(0.4, 0.65, 0.53)
    )


    print "Published lab"
    # rospy.spin()
    moveit_commander.roscpp_shutdown()
