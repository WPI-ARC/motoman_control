#!/usr/bin/python

import roslib; roslib.load_manifest('task_controller')
import rospy
import subprocess
import moveit_commander

depth = 0.87  # Depth of shelf
palm_offset = 0.165  # Offset from tool to palm

#   orientation:
#     x: 0.557211015648
#     y: -0.438833281977
#     z: -0.561100598534
#     w: 0.426740381256

# arm_left
# pose:
#   position:
#     x: 0.539323677836
#     y: 0.433382331813
#     z: 1.6160210999
# arm_right
# pose:
#   position:
#     x: 0.526801535149
#     y: -0.379182889409
#     z: 0.88944822383

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_controller", shell=True)
    path = path.strip("\n")
    rospy.init_node("motoman_apc_controller")

    robot = moveit_commander.RobotCommander()

    print "Starting up"
    raw_input("Hit enter to move to initial pose ")
    # TODO: Move to initial pose
    print "Please jog until the palms touch the shelf"
    raw_input("Hit enter to continue ")
    left = robot.arm_left.get_current_pose().pose
    right = robot.arm_right.get_current_pose().pose
    print "X =", depth/2 + palm_offset + (left.position.x + right.position.x)/2
    print "Y =", (left.position.y + right.position.y)/2
