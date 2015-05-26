
import rospy

from copy import deepcopy
from threading import Lock
from robotiq_s_model_articulated_msgs.msg import SModelRobotInput

from moveit import goto_pose, follow_path, move, robot_state
from shelf import FULL_SHELF
from services import _grasp_generator, _gripper_control, _compute_ik, get_cartesian_path
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
import traceback


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

        # Plan Retreat
        poses = [grasp.pregrasp]
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.z += 0.032
        poses.append(deepcopy(poses[-1]))
        poses[-1].position.x = 0.48
        retreat_response, success = get_cartesian_path(
            group_name=group.get_name(),
            start_state=JointState(
                name=response.solution.joint_trajectory.joint_names,
                position=response.solution.joint_trajectory.points[-1].positions
            ),
            waypoints=poses,
            max_step=0.01,
            jump_threshold=0.0,
            avoid_collisions=True
        )
        if not success:
            continue

        print "Fraction: ", retreat_response.fraction
        rospy.loginfo("Cartesian path had status code: "+str(retreat_response.error_code))

        if retreat_response.fraction < 1:
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

        # TODO: Plan retreat

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
        return False
    rospy.loginfo("Executing cartesian approach")
    if not move(group, plan_to_grasp.joint_trajectory):
        rospy.logerr("Failed to execute approach")
        return False
    if not gripper.grab():
        rospy.loginfo("Executing cartesian retreat")
        follow_path(group, [grasp.approach])  # If grasp fails, move back to approach pose
        return False
    if not move(group, plan_to_retreat.joint_trajectory):
        rospy.logerr("Failed to execute approach")
        return False
    return True
