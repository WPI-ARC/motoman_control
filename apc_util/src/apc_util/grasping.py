
import rospy

from copy import deepcopy
from threading import Lock
from robotiq_s_model_articulated_msgs.msg import SModelRobotInput

from moveit import goto_pose, follow_path, move
from shelf import FULL_SHELF
from services import _grasp_generator, _gripper_control


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
    if not move(group, plan.joint_trajectory):
        rospy.logerr("Failed to execute approach")
        return False

    if not gripper.grab():
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
