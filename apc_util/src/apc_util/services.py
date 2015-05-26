import rospy

from trajectory_verifier.srv import CheckTrajectoryValidity
from trajlib.srv import GetTrajectory
from apc_util.srv import PublishPointcloudCollision
from apc_vision.srv import TakeSample, GetSamples, ProcessSamples
from grasp_planner.srv import apcGraspDB
from gripper_srv.srv import gripper
from motoman_moveit.srv import convert_trajectory_server
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetPositionIK

_publish_pointcloud_collision = rospy.ServiceProxy("publish_pointcloud_collision", PublishPointcloudCollision)
_take_sample = rospy.ServiceProxy("take_sample", TakeSample)
_get_samples = rospy.ServiceProxy("get_samples", GetSamples)
_process_samples = rospy.ServiceProxy("process_samples", ProcessSamples)
_move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
_check_collisions = rospy.ServiceProxy("/check_trajectory_validity", CheckTrajectoryValidity)
_get_known_trajectory = rospy.ServiceProxy("/trajlib", GetTrajectory)
_grasp_generator = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
_gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
_get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
_get_cartesian_path = rospy.ServiceProxy('/compute_cartesian_path', GetCartesianPath)
_compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

services = [
    _publish_pointcloud_collision,
    _take_sample,
    _get_samples,
    _process_samples,
    _move,
    _check_collisions,
    _get_known_trajectory,
    _grasp_generator,
    _gripper_control,
    _get_planning_scene,
    _get_cartesian_path,
    _compute_ik,
]


def wait_for_services(services=services):
    rospy.sleep(5.0)  # Give time for services to start
    missing_services = []

    # Initial pass
    for service in services:
        rospy.loginfo("Waiting for service `%s`..." % service.resolved_name)
        try:
            service.wait_for_service(0.2)
            rospy.loginfo("... service `%s` is available" % service.resolved_name)
        except rospy.exceptions.ROSException:
            rospy.loginfo(".. service `%s` not yet available" % service.resolved_name)
            missing_services.append(service)

    # Second pass
    for service in missing_services:
        rospy.logwarn("Waiting for service `%s`..." % service.resolved_name)
        service.wait_for_service()
        rospy.logwarn("... service `%s` is available" % service.resolved_name)


def get_planning_scene(components):
    for i in range(5):
        try:
            result = _get_planning_scene(components)
            return result, True
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with get_planning_scene(%s): %s" % (components, str(e)))
    rospy.logerr("Failed to get planning scene components")
    return None, False


def get_cartesian_path(group_name, start_state, waypoints, max_step, jump_threshold, avoid_collisions):
    for i in range(5):
        try:
            result = _get_cartesian_path(group_name, start_state, waypoints, max_step, jump_threshold, avoid_collisions)
            return result, True
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with get_cartesian_path(%s, %s, %s, %s, %s, %s): %s"
                          % (group_name, start_state, waypoints, max_step, jump_threshold, avoid_collisions, str(e)))
    rospy.logerr("Failed to get cartesian path")
    return None, False
