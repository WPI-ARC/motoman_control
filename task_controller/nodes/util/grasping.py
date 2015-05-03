
import rospy

from copy import deepcopy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from gripper_srv.srv import gripper, gripperRequest

from moveit import goto_pose, follow_path, NO_SHELF
from collision import add_object, remove_object

position_ik = rospy.ServiceProxy("compute_ik", GetPositionIK, persistent=True)


def check_ik(group, pose, collision_checking=True):
    request = GetPositionIKRequest()
    request.ik_request.group_name = group.get_name()
    request.ik_request.pose_stamped.pose = pose
    request.ik_request.avoid_collisions = collision_checking
    # request.ik_request.robot_state.joint_state = group.get_current_joint_values()
    # request.ik_request.timeout.secs = 0
    # request.ik_request.timeout.nsecs = 500000 # 5ms
    # request.ik_request.timeout = rospy.Duration(0.005)
    response = position_ik(request)
    print response.error_code.val
    return response.error_code.val == 1


def filterGrasps(group, grasps):
    for i, grasp in enumerate(grasps):
        print "%s/%s" % (i+1, len(grasps))
        if check_ik(group, grasp.pregrasp) and check_ik(group, grasp.approach):
            print "Success"
            yield grasp

gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)


def execute_grasp(group, grasp, object_pose):
    # add_object(object_pose)
    if not goto_pose(group, grasp.approach, [1, 5, 30, 60], shelf=NO_SHELF):
        # remove_object()
        return False
    # remove_object()
    if not follow_path(group, [group.get_current_pose().pose, grasp.pregrasp]):
        return False

    request = gripperRequest(command="pinch")
    print "Change to pinch mode:", gripper_control(request)

    request = gripperRequest(command="close")
    print "Grabbing:", gripper_control(request)

    poses = [group.get_current_pose().pose]
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.z += 0.032
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.x = 0.50
    if not follow_path(group, poses):
        return False
    return True
