import rospy

import moveit_commander

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from motoman_moveit.srv import convert_trajectory_server
from shelf import Shelf, NO_SHELF, SIMPLE_SHELF, FULL_SHELF, PADDED_SHELF
from collision import scene
from trajectory_verifier.srv import CheckTrajectoryValidity
from trajectory_verifier.msg import CheckTrajectoryValidityQuery, CheckTrajectoryValidityResult
from trajlib.srv import GetTrajectory

move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
check_collisions = rospy.ServiceProxy("/check_trajectory_validity", CheckTrajectoryValidity)
trajlib = rospy.ServiceProxy("/trajlib", GetTrajectory)

robot = moveit_commander.RobotCommander()

def goto_pose(group, pose, times=[5, 20, 40, 60], shelf=SIMPLE_SHELF):
    """Moves the hand to a given `pose`, using the configured `group`. The
    planning time is modified based on the passed in `times` to try to
    plan quickly if possible, but fall back on longer plans if
    necessary. If `add_shelf` is true, a box model of the shelf is
    added to the environment to avoid collisions."""
    with shelf:
        for t in times:
            group.set_planning_time(t)
            # group.set_start_state_to_current_state()
            rospy.loginfo("Planning for "+str(t)+" seconds...")
            print pose
            plan = group.plan(pose)
            if len(plan.joint_trajectory.points) > 0:
                result = move(plan.joint_trajectory)
                print "Move:", result
                # print "Move:", move(plan.joint_trajectory)
                if result.success:
                    return True
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
        # TODO: return False
    print move(traj.joint_trajectory)
    return True


def execute_known_trajectory(group, task, bin):
    response = trajlib(task=task, bin_num=bin)

    start = list(response.plan.joint_trajectory.points[0].positions)
    print start

    if group.get_current_joint_values() != start:
        rospy.logwarn("execute_known_trajectory(%s, %s): Not starting at the beginning." % (task, bin))
        if not goto_pose(group, start, [1, 2, 10]):
            return False

    with SIMPLE_SHELF:
        collisions = check_collisions(CheckTrajectoryValidityQuery(
            initial_state=JointState(
                header=Header(stamp=rospy.Time.now()),
                name=robot.sda10f.get_joints(),
                position=robot.sda10f.get_current_joint_values()
            ),
            trajectory=response.plan.joint_trajectory,
            check_type=CheckTrajectoryValidityQuery.CHECK_ENVIRONMENT_COLLISION,
        ))

    if collisions.result.status != CheckTrajectoryValidityResult.SUCCESS:
        rospy.logwarn("Can't execute path from trajectory library, status=%s" % collisions.result.status)
        return False

    print move(response.plan.joint_trajectory)
    return True
