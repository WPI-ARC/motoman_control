
import rospy

import moveit_commander

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from gripper_srv.srv import gripper, gripperRequest

from moveit import add_object, remove_object, goto_pose, follow_path

position_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
def check_ik(group, pose, collision_checking=True):
    request = GetPositionIKRequest()
    request.ik_request.group_name = group.get_name();
    request.ik_request.pose_stamped.pose = pose
    request.ik_request.avoid_collisions = True
    response = position_ik.call(request)
    print response
    return response.error_code.val == 1

def filterGrasps(group, grasps):
    filtered = []
    for grasp in grasps:
        if check_ik(group, grasp.posegrasp) and check_ik(group, grasp.poseapproach):
            filtered.append(grasp)
    return filtered

gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
def execute_grasp(group, grasp, object_pose):
    add_object(object_pose)
    if not goto_pose(group, grasp.poseapproach, [1, 5, 30, 60]):
        remove_object()
        return False
    remove_object()
    if not follow_path(group, [group.get_current_pose().pose, grasp.posegrasp]):
        return False
    request = gripperRequest(command="close")
    response = gripper_control.call(request)
    if not follow_path(group, [group.get_current_pose().pose, grasp.poseapproach]):
        return False
    return True