
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

scene = moveit_commander.PlanningSceneInterface()


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
