#!/usr/bin/python

import rospy
import subprocess
import moveit_commander
import yaml
import os

from geometry_msgs.msg import Pose, Point, Quaternion
from apc_util.moveit import goto_pose
from apc_util.shelf import PADDED_SHELF

depth = 0.87  # Depth of shelf
palm_offset = 0.165  # Offset from tool to palm

# Currently shelf + orientation are not calibrated
z = -0.155
orientation = {
    "x": 0.5,
    "y": 0.5,
    "z": 0.5,
    "w": 0.5,
}

default_orientation = Quaternion(
    x=0.557211015648,
    y=-0.438833281977,
    z=-0.561100598534,
    w=0.426740381256,
)

pose_left = Pose(
    position=Point(
        x=0.539323677836,
        y=0.483382331813,
        z=1.6160210999,
    ),
    orientation=default_orientation,
)

pose_right = Pose(
    position=Point(
        x=0.526801535149,
        y=-0.429182889409,
        z=0.88944822383,
    ),
    orientation=default_orientation,
)


if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_controller", shell=True)
    path = path.strip("\n")
    rospy.init_node("motoman_apc_controller")

    robot = moveit_commander.RobotCommander()
    robot.arm_left_torso.set_planning_time(20)

    rospy.loginfo("Starting up")
    skip = raw_input("Hit enter to move to initial pose or type skip to skip: ")
    if "s" not in skip:
        with PADDED_SHELF:
            rospy.loginfo("Left: %s" % goto_pose(robot.arm_left_torso, pose_left))
        # rospy.loginfo("Right: %s" % robot.arm_right.go(pose_right))
    rospy.loginfo("Please jog until the palms touch the shelf")
    raw_input("Hit enter to continue ")
    left = robot.arm_left.get_current_pose().pose
    # right = robot.arm_right.get_current_pose().pose

    skip = raw_input("Hit enter to move to initial pose or type skip to skip: ")
    if "s" not in skip:
        with PADDED_SHELF:
            rospy.loginfo("Right: %s" % goto_pose(robot.arm_left_torso, pose_right))
    rospy.loginfo("Please jog until the palms touch the shelf")
    raw_input("Hit enter to continue ")
    right = robot.arm_left.get_current_pose().pose

    x = depth/2 + palm_offset + (left.position.x + right.position.x)/2
    y = (left.position.y + right.position.y)/2
    rospy.loginfo("X = %s" % x)
    rospy.loginfo("Y = %s" % y)

    cfg = {
        "shelf": {
            "position": {
                "x": x,
                "y": y,
                "z": z
            },
            "orientation": orientation
        }
    }

    filename = os.path.join(os.path.dirname(__file__), "../../task_controller/cfg/shelf.yaml")
    with open(filename, "w") as file:
        yaml.dump(cfg, file, default_flow_style=False)
    rospy.loginfo("Wrote config to %s" % filename)
