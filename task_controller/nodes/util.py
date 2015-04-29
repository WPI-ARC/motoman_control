
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from gripper_srv.srv import gripper, gripperRequest

scene = moveit_commander.PlanningSceneInterface()


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
        result = group.go(pose)
        if result:
            if with_shelf:
                remove_shelf()
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
        rospy.logwarn("Cartesian trajectory could not be completed. Only solved for: '"+str(success)+"'...")
        #return False
    return group.execute(traj)

position_ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
def check_ik(group, pose, collision_checking=True):
    request = GetPositionIKRequest()
    request.ik_request.group_name = group.get_name();
    request.ik_request.pose_stamped.pose = pose
    request.ik_request.avoid_collisions = True
    #request.ik_request.robot_state.joint_state = group.get_current_joint_values()
    # request.ik_request.timeout.secs = 0
    # request.ik_request.timeout.nsecs = 500000 # 5ms
    request.ik_request.timeout = rospy.Duration(0.005)
    response = position_ik.call(request)
    return response.error_code.val == 1

def filterGrasps(group, grasps):
    for i, grasp in enumerate(grasps):
        print "%s/%s" % (i+1, len(grasps))
        if check_ik(group, grasp.posegrasp) and check_ik(group, grasp.poseapproach):
            print "Success"
            yield grasp

gripper_control = rospy.ServiceProxy("/left/command_gripper", gripper)
def execute_grasp(group, grasp, object_pose):
    # add_object(object_pose)
    if not goto_pose(group, grasp.poseapproach, [1, 5, 30, 60], with_shelf=False):
        # remove_object()
        return False
    # remove_object()
    if not follow_path(group, [group.get_current_pose().pose, grasp.posegrasp]):
        return False
    request = gripperRequest(command="close")
    response = gripper_control.call(request)
    if not follow_path(group, [group.get_current_pose().pose, grasp.poseapproach]):
        return False
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

def add_shelf():
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 1.25
    pose.pose.position.y = 0
    pose.pose.position.z = 1.25
    pose.pose.orientation.x = 0.5
    pose.pose.orientation.y = 0.5
    pose.pose.orientation.z = 0.5
    pose.pose.orientation.w = 0.5
    print "Adding shelf", scene._pub_co.get_num_connections()
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.01)
        print "Waiting..."
    scene.add_box(
        name="shelf",
        pose=pose,
        size=(0.86, 2.5, 0.86)
    )
    print "Added"


def remove_shelf():
    remove_object("shelf")

def bin_pose(bin, bin_x=1.32, bin_y=0, bin_z=-0.01):
    # Setting Configuration:
    # 	A		B		C
    # 	D		E		F
    # 	G		H		I
    # 	J		K		L
    # 		   Base

    # Gripper dimension
    GripperLength = 0.2

    # Bin dimension Unit m
    Bin_depth = 0.430
    Start_Gap = 0.100

    LeftBin_width = 0.240
    MiddleBin_width = 0.300
    RightBin_width = 0.240

    WorkBase_Height = 0.820
    BottomLay_Height = 0.230
    SecndLayer_Height = 0.230
    ThirdLayer_Height = 0.220
    TopLayer_Height = 0.260

    Left_horizontal_ShiftValue = MiddleBin_width/2 + LeftBin_width/2
    Right_horizontal_ShiftValue = MiddleBin_width/2 + RightBin_width/2

    TopLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height + TopLayer_Height/2
    ThirdLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height + ThirdLayer_Height/2
    SecndLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height + SecndLayer_Height/2
    BottomLayer_vertical_shiftvalue = WorkBase_Height + BottomLay_Height/2

    Entry_X_shiftvalue = bin_x - Bin_depth - Start_Gap - GripperLength - 0.035

    pose = PoseStamped()
    pose.pose.position.x = Entry_X_shiftvalue

    if bin == "A":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "B":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "C":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + TopLayer_vertical_shiftvalue
    elif bin == "D":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "E":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "F":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + ThirdLayer_vertical_shiftvalue
    elif bin == "G":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "H":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "I":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + SecndLayer_vertical_shiftvalue
    elif bin == "J":
        pose.pose.position.y = bin_y + Left_horizontal_ShiftValue
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    elif bin == "K":
        pose.pose.position.y = bin_y
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    elif bin == "L":
        pose.pose.position.y = bin_y - Right_horizontal_ShiftValue
        pose.pose.position.z = bin_z + BottomLayer_vertical_shiftvalue
    else:
        raise Exception("Bin `%s` not supported."%bin)
        # TODO: Throw exception

    #pose.pose.orientation.x = -0.12384;
    #pose.pose.orientation.y = 0.0841883;
    #pose.pose.orientation.z = -0.730178;
    #pose.pose.orientation.w = 0.666646;
    #pose.pose.orientation.x = 0.5
    #pose.pose.orientation.y = -0.5
    #pose.pose.orientation.z = -0.5
    #pose.pose.orientation.w = 0.5
    pose.pose.orientation.x = -0.484592
    pose.pose.orientation.y = 0.384602
    pose.pose.orientation.z = 0.615524
    pose.pose.orientation.w = -0.488244

    return pose
