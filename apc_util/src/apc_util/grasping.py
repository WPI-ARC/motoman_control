
import rospy

from copy import deepcopy
from gripper_srv.srv import gripper

from moveit import goto_pose, follow_path, move
from shelf import FULL_SHELF


def plan_grasps(group, grasps):
    for i, grasp in enumerate(grasps):
        rospy.loginfo("%s/%s" % (i+1, len(grasps)))
        traj, success = group.compute_cartesian_path(
            [grasp.approach, grasp.pregrasp],
            0.01,  # 1cm interpolation resolution
            0.0,  # jump_threshold disabled
            avoid_collisions=True,
        )
        if success >= 1:
            rospy.loginfo("Found grasp")
            yield grasp, traj


gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)


def execute_grasp(group, grasp, plan, shelf=FULL_SHELF):
    rospy.loginfo("Moving to approach pose")
    start = list(plan.joint_trajectory.points[0].positions)
    if not goto_pose(group, start, [1, 5, 30, 60], shelf=shelf):
        return False

    rospy.loginfo("Executing cartesian approach")
    if not move(plan.joint_trajectory).success:
        rospy.logerr("Failed to execute approach")
        return False

    rospy.loginfo("Grabbing: %s" % gripper_control(command="close"))
    rospy.sleep(4)

    rospy.loginfo("Executing cartesian retreat")
    poses = [group.get_current_pose().pose]
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.z += 0.032
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.x = 0.40
    if not follow_path(group, poses):
        return False
    return True
