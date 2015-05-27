#!/usr/bin/python

import rospy

import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion


def add_order_bin(x, y):
    # Necessary parameters
    bin_width, bin_depth, bin_height = 0.65, 0.4, 0.43
    interior_width, interior_depth, interior_height = 0.45, 0.29, 0.18
    wall_width, wall_depth = (bin_width-interior_width)/2, (bin_depth-interior_depth)/2
    base_height = bin_height-interior_height
    objects = []
    poses = []

    # Create Bottom
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[bin_depth, bin_width, base_height],
    ))
    poses.append(Pose(
        position=Point(x=x, y=y, z=base_height/2),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create left side
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[bin_depth, wall_width, interior_height],
    ))
    poses.append(Pose(
        position=Point(
            x=x,
            y=y-interior_width/2-wall_width/2,
            z=base_height+interior_height/2
        ), orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create right side
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[bin_depth, wall_width, interior_height],
    ))
    poses.append(Pose(
        position=Point(
            x=x,
            y=y+interior_width/2+wall_width/2,
            z=base_height+interior_height/2
        ),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create near side
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[wall_depth, interior_width, interior_height],
    ))
    poses.append(Pose(
        position=Point(
            x=x-interior_depth/2-wall_depth/2,
            y=y,
            z=base_height+interior_height/2
        ),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    # Create far side
    objects.append(SolidPrimitive(
        type=SolidPrimitive.BOX,
        dimensions=[wall_depth, interior_width, interior_height],
    ))
    poses.append(Pose(
        position=Point(
            x=x+interior_depth/2+wall_depth/2,
            y=y,
            z=base_height+interior_height/2
        ),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    ))

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "order_bin"
    co.header.frame_id = "/base_link"
    co.header.stamp = rospy.Time.now()
    co.primitives = objects
    co.primitive_poses = poses

    scene._pub_co.publish(co)


if __name__ == '__main__':
    scene = moveit_commander.PlanningSceneInterface()
    scene_topic = rospy.Publisher("planning_scene", PlanningScene)
    rospy.init_node('publish_boxshelf', anonymous=True)
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.1)

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = -1
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
    pose.pose.position.x = 1.2
    pose.pose.position.y = 0
    pose.pose.position.z = 1.5
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    # scene.add_box(
    #     name="front wall",
    #     pose=pose,
    #     size=(0.01, 6, 3)
    # )

    # pose = PoseStamped()
    # pose.header.frame_id = "/base_link"
    # pose.header.stamp = rospy.Time.now()
    # pose.pose.position.x = 1.4732
    # pose.pose.position.y = 0
    # pose.pose.position.z = 0.4 - 0.127
    # pose.pose.orientation.x = 0
    # pose.pose.orientation.y = 0
    # pose.pose.orientation.z = 0
    # pose.pose.orientation.w = 1
    #
    # scene.add_box(
    #     name="table",
    #     pose=pose,
    #     size=(0.762, 2.44, 0.8)
    # )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = -0.55
    pose.pose.position.y = 0
    pose.pose.position.z = 0.15
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="back_wiring",
        pose=pose,
        size=(0.5, 0.6, 0.3)
    )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = -0.40
    pose.pose.position.y = 0.70
    pose.pose.position.z = 0.2
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="box holding wires",
        pose=pose,
        size=(0.8, 0.8, 0.6)
    )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = 1
    pose.pose.position.z = 1.5
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="left wall",
        pose=pose,
        size=(6, 0.01, 3)
    )

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = -1
    pose.pose.position.z = 1.5
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="right wall",
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
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2.3
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    # scene.add_box(
    #     name="ceiling",
    #     pose=pose,
    #     size=(6, 6, 0.1)
    # )

    add_order_bin(0.5, 0.2)

    print "Published lab"
    # rospy.spin()
    moveit_commander.roscpp_shutdown()
