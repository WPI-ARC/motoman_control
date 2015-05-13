
import rospy

from copy import deepcopy
from gripper_srv.srv import gripper
from grasp_planner.srv import apcGraspDB

from moveit import goto_pose, follow_path, move
from shelf import FULL_SHELF

_gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
_grasp_generator = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)


def control_gripper(command):
    for i in range(5):
        try:
            _gripper_control(command=command)
            rospy.sleep(2)  # Wait for gripper to move
            # TODO: Make gripper assertions
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with control_gripper(%s): %s" % (command, str(e)))
    rospy.logerr("Failed to %s gripper" % command)
    return False


def generate_grasps(item, pose, pointcloud, bin):
    for i in range(5):
        try:
            response = _grasp_generator(
                item=item,
                object_pose=pose,
                object_points=pointcloud,
                bin=bin,
            )
            return response.grasps.grasps, True
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with generate_grasps(%s, %s, <<pointcloud>>, %s): %s" % (item, pose, bin, str(e)))
    rospy.logerr("Failed to get online grasps for (%s, %s, <<pointcloud>>, %s)"
                 % (item, pose, bin))
    return None, False


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


def execute_grasp(group, grasp, plan, shelf=FULL_SHELF):
    rospy.loginfo("Moving to approach pose")
    start = list(plan.joint_trajectory.points[0].positions)
    if not goto_pose(group, start, [1, 5, 30, 60], shelf=shelf):
        return False

    rospy.loginfo("Executing cartesian approach")
    if not move(plan.joint_trajectory):
        rospy.logerr("Failed to execute approach")
        return False

    if not control_gripper("close"):
        return False

    rospy.loginfo("Executing cartesian retreat")
    poses = [group.get_current_pose().pose]
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.z += 0.032
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.x = 0.40
    if not follow_path(group, poses):
        return False
    return True
