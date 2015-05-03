
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject

scene = moveit_commander.PlanningSceneInterface()


def add_object(center, name="Object", radius=0.17):
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose = center
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.01)
    scene.add_sphere(
        name=name,
        pose=pose,
        radius=radius,
    )


def remove_object(name="Object"):
    co = CollisionObject()
    co.operation = CollisionObject.REMOVE
    co.id = name
    co.header.frame_id = "/base_link"
    co.header.stamp = rospy.Time.now()
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.01)
    scene._pub_co.publish(co)
