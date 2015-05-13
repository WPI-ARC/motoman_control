#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy

import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld

from geometry_msgs.msg import PoseStamped
from apc_util.collision import add_object, scene

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive


def attach_sphere(link, name, pose, radius, touch_links=[]):
    aco = AttachedCollisionObject()

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = name
    co.header = pose.header
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [radius]
    co.primitives = [sphere]
    co.primitive_poses = [pose.pose]
    aco.object = co

    aco.link_name = link
    if len(touch_links) > 0:
        aco.touch_links = touch_links
    else:
        aco.touch_links = [link]
    print aco
    scene._pub_aco.publish(aco)

if __name__ == '__main__':
    scene = moveit_commander.PlanningSceneInterface()
    scene_topic = rospy.Publisher("planning_scene", PlanningScene)
    rospy.init_node('sadfsfdasdfasdf', anonymous=True)
    # while scene._pub_co.get_num_connections() == 0:
    #     rospy.sleep(1)
    rospy.sleep(10)

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
    attach_sphere("arm_left_link_7_t", "Object", pose, 0.17, ["hand_left_finger_1_link_2", "hand_left_finger_1_link_3", "hand_left_finger_1_link_3_tip", "hand_left_finger_2_link_2", "hand_left_finger_2_link_3", "hand_left_finger_2_link_3_tip", "hand_left_finger_middle_link_2", "hand_left_finger_middle_link_3", "hand_left_finger_middle_link_3_tip"])



    print "Published object"
    # rospy.spin()
    moveit_commander.roscpp_shutdown()
