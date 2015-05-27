
import rospy

from copy import deepcopy
from threading import Lock
from robotiq_s_model_articulated_msgs.msg import SModelRobotInput

from moveit import goto_pose, follow_path, move, robot_state, check_collision
from shelf import FULL_SHELF
from services import _grasp_generator, _gripper_control, _compute_ik, get_cartesian_path
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
import traceback
from apc_util.shelf import get_shelf_pose
from apc_util.transformation_helpers import *
import numpy
from geometry_msgs.msg import Pose
from apc_util.transformation_helpers import *
from copy import deepcopy

class Gripper(object):
    def __init__(self):
        self._safety = 10
        self._latest_gripper_value = None
        self._m = Lock()
        rospy.Subscriber("/left/SModelRobotInput", SModelRobotInput, self.gripper_callback)

    def gripper_callback(self, msg):
        self._m.acquire()
        self._latest_gripper_value = msg
        self._m.release()

    def check_gripper_values(self, command):
        """
        Check that the gripper doesn't close too much.

        TODO: In the future, we should check for faults and other problems.
        """
        if command == "close":
            return True  # Don't check, it's closing all the way
        elif command == "open":
            return True  # TODO: What do we do
        elif command in ["activate", "reset", "pinch", "basic"]:
            return True  # Don't check, Shouldn't really mess with the gripper
        else:
            self._m.acquire()
            msg = self._latest_gripper_value
            self._m.release()
            if msg is None:
                rospy.logerr("No SModelRobotInput messages received.")
                return False
            if msg.gPOA > (self._safety + msg.gPRA):
                rospy.logerr("Finger A too far closed.")
                return False
            elif msg.gPOB > (self._safety + msg.gPRA):
                rospy.logerr("Finger B too far closed.")
                return False
            elif msg.gPOC > (self._safety + msg.gPRA):
                rospy.logerr("Finger C too far closed.")
                return False
            return True

    def control_gripper(self, command):
        robot_state.wait_to_continue()
        for i in range(5):
            try:
                _gripper_control(command=command)
                rospy.sleep(4)  # Wait for gripper to move
                if not self.check_gripper_values(command):
                    rospy.logerr("Gripper failed verification")
                    # TODO: Open Hand
                    return False
                return True
            except rospy.ServiceException as e:
                rospy.logwarn("Failure with control_gripper(%s): %s" % (command, str(e)))
        rospy.logerr("Failed to %s gripper" % command)
        return False

    def open(self):
        return self.control_gripper("open")

    def grab(self):
        return self.control_gripper("125")

    def vision(self):
        return self.control_gripper("close")

gripper = Gripper()


def generate_grasps(item, pose, shelf_pose, pointcloud, bin):
    for i in range(5):
        try:
            response = _grasp_generator(
                item=item,
                object_pose=pose,
                shelf_pose=shelf_pose,
                object_points=pointcloud,
                bin=bin,
            )
            return response.grasps.grasps, True
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with generate_grasps(%s, %s, <<pointcloud>>, %s): %s" % (item, pose, bin, str(e)))
    rospy.logerr("Failed to get online grasps for (%s, %s, <<pointcloud>>, %s)"
                 % (item, pose, bin))
    return None, False

def retreat_cheezit(self, grasp, group, response):
    # Special case for retreating for object cheezit
    # Get Bin bounds
    Tbase_shelf = PoseToMatrix(get_shelf_pose())
    bin_bounds = rospy.get_param("/shelf/bins/"+req.bin)
    _, _, bin_min_y, bin_max_y, _, _ = bin_bounds
    poses = [grasp.pregrasp]
    # Move up to allow rotation w/o collision. 1.5cm*sin(45deg)=1.06066cm
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.z += 0.015 
    # Move object to center of bin along y-axis and Rotate 90 deg about z
    Ttemp = PoseToMatrix(poses[-1])
    rotate_angle = numpy.pi/4.0
    Rotz = numpy.array([[numpy.cos(rotate_angle), -numpy.sin(rotate_angle), 0, 0],
                        [numpy.sin(rotate_angle), numpy.cos(rotate_angle), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    Trotate = numpy.dot(Ttemp, Rotz)
    poses.append(PoseFromMatrix(Trotate))
    bin_y_mid_base = numpy.dot(Tbase_shelf, numpy.array([ [(bin_max_y + bin_min_y)/2.0], [0], [0], [1] ]))
    poses[-1].position.y = bin_y_mid_base[1]
    # Move object out of bin
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.x = 0.4
    retreat_response, success = get_cartesian_path(
        group_name=group.get_name(),
        start_state=RobotState(
            joint_state=JointState(
                name=response.solution.joint_trajectory.joint_names,
                position=response.solution.joint_trajectory.points[-1].positions
            )
        ),  
        waypoints=poses,
        max_step=0.01,
        jump_threshold=0.0,
        avoid_collisions=True
    )
    return retreat_response, success

def retreat_normal(self, grasp, group, response):
    # Normal retreat for object
    poses = [grasp.pregrasp]
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.z += 0.032
    poses.append(deepcopy(poses[-1]))
    poses[-1].position.x = 0.4
    retreat_response, success = get_cartesian_path(
        group_name=group.get_name(),
        start_state=RobotState(
            joint_state=JointState(
                name=response.solution.joint_trajectory.joint_names,
                position=response.solution.joint_trajectory.points[-1].positions
            )
        ),  
        waypoints=poses,
        max_step=0.01,
        jump_threshold=0.0,
        avoid_collisions=False
    )
    return retreat_response, success

def execute_wallgrasp_left(group, bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z):
    try:
        poses = []
        x_depth = (bin_max_x - bin_min_x) / 2.0
        if x_depth > 0.10:
            x_depth = 0.10
        theta = numpy.pi/4.0
        x = self.fingerlength * numpy.cos(theta)
        y = self.fingerlength * numpy.sin(theta)

        # Bin values
        point_x = bin_min_x
        point_y = bin_max_y

        # pregrasp left
        y_msg = point_y - y
        x_msg = point_x - x + 0.07 # extra 7cm to move past the vertical beam

        # pregrasp message
        pregrasp = Pose()
        approach = Pose()

        unit_vector = PoseFromComponents([0, 0, 0], [0, 0, 0, 1])
        quat = QuaternionFromAxisAngle( [0, 0, 1], -numpy.pi/4.0 )
        Rotz = PoseFromComponents([0, 0, 0], quat)
        Rotz_msg = ComposePoses(unit_vector, Rotz)

        pregrasp.position.x = x_msg
        pregrasp.position.y = y_msg
        pregrasp.postiion.z = bin_min_z + 0.02
        pregrasp.rotation = Rotz_msg.rotation
        approach = deepcopy(pregrasp)
        approach.position.x -= 0.1 # set approach to be 10cm back from pregrasp

        self.plan

        # Cartesian move
        poses.append(approach)
        poses.append(pregrasp)
        poses.append(deepcopy(poses[-1])) # move 1cm into wall
        poses[-1].position.y += 0.01
        poses.append(deepcopy(poses[-1])) # move forward into bin
        poses[-1].position.x += x_depth

        if not follow_path(self.arm, poses, False):
            return False, "open"

        if not gripper.grab():
            rospy.loginfo("Executing cartesian retreat")
            follow_path(group, [grasp.approach])  # If grasp fails, move back to approach pose
            return False, "closed"

        # TODO: Retreat
        retreats.append(deepcopy(poses[-1]))
        retreats.append(deepcopy(retreats[-1]))
        retreats[-1].position.y -= 0.02
        retreats.append(deepcopy(retreats[-1]))
        retreats[-1].position.x -= x_depth - 0.05

        if not follow_path(self.arm, retreats, False):
            return False, "closed"
    except:
        print traceback.format_exc()
        rospy.logerr(str(traceback.format_exc()))

    return success

def execute_wallgrasp_right(group, bin_min_x, bin_min_y, bin_max_y, bin_min_z):
    try:
        poses = []
        retreats = []

        theta = numpy.pi/4.0
        x = self.fingerlength * numpy.cos(theta)
        y = self.fingerlength * numpy.sin(theta)

        # Bin values
        point_x = bin_min_x
        point_y = bin_max_y

        # pregrasp right
        y_msg_right = right_point_y + y
        x_msg_right = right_point_x - x + 0.07 # extra 7cm to move past the vertical beam

        # pregrasp message
        pregrasp = Pose()
        approach = Pose()

        unit_vector = PoseFromComponents([0, 0, 0], [0, 0, 0, 1])
        quat = QuaternionFromAxisAngle( [0, 0, 1], numpy.pi/4.0 )
        Rotz = PoseFromComponents([0, 0, 0], quat)
        Rotz_msg = ComposePoses(unit_vector, Rotz)

        pregrasp.position.x = x_msg
        pregrasp.position.y = y_msg
        pregrasp.postiion.z = bin_min_z + 0.02
        pregrasp.rotation = Rotz_msg.rotation
        approach = deepcopy(pregrasp)
        approach.position.x -= 0.1 # set approach to be 10cm back from pregrasp

        # Cartesian move
        poses.append(approach)
        poses.append(pregrasp)
        poses.append(deepcopy(poses[-1])) # move 1cm into wall
        poses[-1].position.y -= 0.01
        poses.append(deepcopy(poses[-1])) # move forward into bin
        poses[-1].position.x += x_depth

        if not follow_path(self.arm, poses, False):
            return False, "open"

        if not gripper.grab():
            rospy.loginfo("Executing cartesian retreat")
            follow_path(group, [grasp.approach])  # If grasp fails, move back to approach pose
            return False, "closed"

        # TODO: Retreat
        retreats.append(deepcopy(poses[-1]))
        retreats.append(deepcopy(retreats[-1]))
        retreats[-1].position.y += 0.02
        retreats.append(deepcopy(retreats[-1]))
        retreats[-1].position.x -= x_depth - 0.05

        if not follow_path(self.arm, retreats, False):
            return False, "closed"
    except:
        print traceback.format_exc()
        rospy.logerr(str(traceback.format_exc()))

    return success

def plan_grasps(group, grasps):
    cur_state = RobotState(
        joint_state=JointState(
            name=group.get_joints(),
            position=group.get_current_joint_values()
        )
    )
    for i, grasp in enumerate(grasps):
        rospy.loginfo("%s/%s" % (i+1, len(grasps)))
        approach_plan = None

        try:
            ik_solution = _compute_ik(
                ik_request = PositionIKRequest(
                    group_name = "arm_left_torso",
                    robot_state = cur_state,
                    pose_stamped = PoseStamped(
                        header=Header(frame_id="base_link"),
                        pose=grasp.approach
                    ),
                    avoid_collisions = True
                )
            )
        except:#, err:
            #print "compute_ik failed, "+str(err)
            print traceback.format_exc()
            rospy.logerr("Failed to call IK")
            continue

        if ik_solution.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logwarn("Failed to find valid IK solution "+str(ik_solution.error_code.val))
            continue

        # Plan Approach
        response, success = get_cartesian_path(
            group_name=group.get_name(),
            start_state=ik_solution.solution,
            waypoints=[grasp.pregrasp],
            max_step=0.01,
            jump_threshold=0.0,
            avoid_collisions=True
        )
        if not success:
            continue

        print "Fraction: ", response.fraction
        rospy.loginfo("Cartesian path had status code: "+str(response.error_code))

        if response.fraction < 1:
            continue
        if not check_collision(response.solution.joint_trajectory):
            continue

        # Plan Retreat
        retreat_response, success = self.retreat_normal(grasp, group, response)

        # if  ischeezit:
        #     retreat_response, success = self.retreat_cheezit(grasp, group, response)
        # else:
        #     retreat_response, success = self.retreat_normal(grasp, group, response)

        if not success:
            continue

        print "Fraction: ", retreat_response.fraction
        rospy.loginfo("Cartesian path had status code: "+str(retreat_response.error_code))

        if retreat_response.fraction < 1:
            continue
        if not check_collision(retreat_response.solution.joint_trajectory):
            continue

        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_workspace([-3, -3, -3, 3, 3, 3])
        for t in [1,3]:
            group.set_planning_time(t)
            plan = group.plan(ik_solution.solution.joint_state)
            if len(plan.joint_trajectory.points) > 0:
                approach_plan = plan
                break

        if not approach_plan:
            rospy.logwarn("Failed to find plan to approach pose")
            continue
        if not check_collision(approach_plan.joint_trajectory):
            continue

        rospy.loginfo("Found grasp")
        rospy.loginfo(ik_solution.solution.joint_state)
        yield grasp, approach_plan, response.solution, retreat_response.solution


def execute_grasp(group, grasp, plan_to_approach, plan_to_grasp, plan_to_retreat, shelf=FULL_SHELF):
    rospy.loginfo("Moving to approach pose")
    #start = list(plan.joint_trajectory.points[0].positions)
    #if not goto_pose(group, start, [1, 5, 30, 60], shelf=shelf):
    #    return False
    if not move(group, plan_to_approach.joint_trajectory):
        rospy.logerr("Failed to got to approach pose")
        return False, 'open'
    rospy.loginfo("Executing cartesian approach")
    if not move(group, plan_to_grasp.joint_trajectory):
        rospy.logerr("Failed to execute approach")
        return False, 'open'
    if not gripper.grab():
        rospy.loginfo("Executing cartesian retreat")
        follow_path(group, [grasp.approach])  # If grasp fails, move back to approach pose
        return False, 'closed'
    if not move(group, plan_to_retreat.joint_trajectory):
        rospy.logerr("Failed to execute retreat")
        return False, 'closed'
    return True, 'closed'
