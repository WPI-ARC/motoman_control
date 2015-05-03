
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from motoman_moveit.srv import convert_trajectory_server


scene = moveit_commander.PlanningSceneInterface()
move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)


def goto_pose(group, pose, times=[5, 20, 40, 60], with_shelf=True):
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
        plan = group.plan(pose)
        print "Plan:", plan
        print "Move:", move(plan.joint_trajectory)
        # if result:
        #     if with_shelf:
        #         remove_shelf()
        remove_shelf()
        rospy.sleep(1)
        return True
    if with_shelf:
        remove_shelf()
    return False


def follow_path(group, path, collision_checking=True):
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
        rospy.logwarn(
            "Cartesian trajectory could not be completed. Only solved for: '"
            + str(success) + "'..."
        )
        # return False
    print move(traj.joint_trajectory)
    rospy.sleep(1)
    return True


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


from shelf import add_shelf, remove_shelf
