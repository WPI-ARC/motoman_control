
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from motoman_moveit.srv import convert_trajectory_server
from shelf import Shelf, NO_SHELF, SIMPLE_SHELF, FULL_SHELF
from collision import scene


move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)


def goto_pose(group, pose, times=[5, 20, 40, 60], shelf=SIMPLE_SHELF):
    """Moves the hand to a given `pose`, using the configured `group`. The
    planning time is modified based on the passed in `times` to try to
    plan quickly if possible, but fall back on longer plans if
    necessary. If `add_shelf` is true, a box model of the shelf is
    added to the environment to avoid collisions."""
    with shelf:
        for t in times:
            group.set_planning_time(t)
            rospy.loginfo("Planning for "+str(t)+" seconds...")
            plan = group.plan(pose)
            if len(plan.joint_trajectory.points) > 0:
                print "Move:", move(plan.joint_trajectory)
                return True
        return False


def follow_path(group, path, collision_checking=True):
    """Follows a cartesian path using a linear interpolation of the given
    `path`. The `collision_checking` parameter controls whether or not
    to check the path for collisions with the environment."""
    traj, success = group.compute_cartesian_path(
        path,
        0.001,  # 1cm interpolation resolution
        0.0,  # jump_threshold disabled
        avoid_collisions=collision_checking,
    )
    if success < 1:
        rospy.logwarn(
            "Cartesian trajectory could not be completed. Only solved for: '"
            + str(success) + "'..."
        )
        # TODO: return False
    print move(traj.joint_trajectory)
    return True
