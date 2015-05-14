import rospy

from trajectory_verifier.srv import CheckTrajectoryValidity
from trajlib.srv import GetTrajectory
from apc_util.srv import PublishPointcloudCollision
from apc_vision.srv import TakeSample, GetSamples, ProcessSamples
from grasp_planner.srv import apcGraspDB
from gripper_srv.srv import gripper
from motoman_moveit.srv import convert_trajectory_server


_publish_pointcloud_collision = rospy.ServiceProxy("publish_pointcloud_collision", PublishPointcloudCollision)
_take_sample = rospy.ServiceProxy("take_sample", TakeSample)
_get_samples = rospy.ServiceProxy("get_samples", GetSamples)
_process_samples = rospy.ServiceProxy("process_samples", ProcessSamples)
_move = rospy.ServiceProxy("/convert_trajectory_service", convert_trajectory_server)
_check_collisions = rospy.ServiceProxy("/check_trajectory_validity", CheckTrajectoryValidity)
_get_known_trajectory = rospy.ServiceProxy("/trajlib", GetTrajectory)
_grasp_generator = rospy.ServiceProxy('getGrasps_online_server', apcGraspDB)
_gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)

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
        rospy.loginfo("Waiting for service `%s`..." % service.resolved_name)
        service.wait_for_service()
        rospy.loginfo("... service `%s` is available" % service.resolved_name)
