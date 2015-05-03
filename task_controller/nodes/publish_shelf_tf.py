#!/usr/bin/python

import roslib; roslib.load_manifest('task_controller')

import rospy
import tf2_ros
import geometry_msgs.msg
from util.shelf import get_shelf_pose

if __name__ == '__main__':
    rospy.init_node("publish_shelf_tf")

    pose = get_shelf_pose()
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = "shelf"
    tf.transform.translation = pose.pose.position
    # Should be:
    # tf.transform.rotation = pose.pose.orientation
    # but vision wants x, y, z, w = 0, 0, 0, 1
    tf.transform.rotation.x = 0
    tf.transform.rotation.y = 0
    tf.transform.rotation.z = 0
    tf.transform.rotation.w = 1

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(250.0)
    while (not rospy.is_shutdown()):
        tf.header.stamp = rospy.Time.now()
        br.sendTransform(tf)
        rate.sleep()
