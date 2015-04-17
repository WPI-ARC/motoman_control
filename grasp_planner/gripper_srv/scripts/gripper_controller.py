#!/usr/bin/env python


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a S-Model gripper.

This serves as an example for publishing messages on the 'SModelRobotOutput' topic using the 'SModel_robot_output' msg type for sending commands to a S-Model gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from time import sleep

from gripper_srv.srv import *


#--------------------------------------------------------------------
'''
Need to fix command = outputMsg.SModel_robot_output() locatation. When
called it will set all the values to 0. So probably if you set a speed, it will be overwritten everytime command is called since its inside the callback function.


'''

#---------------------------------------------------------------------

command = outputMsg.SModel_robot_output();

def genCommand(req):
    global command

    #Update the command according to request command

    if req.command == 'activate':
        command = outputMsg.SModel_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if req.command == 'reset':
        command = outputMsg.SModel_robot_output();
        command.rACT = 0

    if req.command == 'close':
        command.rPRA = 255

    if req.command == 'open':
        command.rPRA = 0

    if req.command == 'basic':
        command.rMOD = 0

    if req.command == 'pinch':
        command.rMOD = 1

    if req.command == 'wide':
        command.rMOD = 2

    if req.command == 'scissor':
        command.rMOD = 3

    #If the command entered is a int, assign this value to rPRA
    try:
        command.rPRA = int(req.command)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if req.command == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if req.command == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0


    if req.command == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if req.command == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0
    print "---------------------------------"
    print req.command
    print " "
    print command
    print "---------------------------------"

    pub.publish(command)

    return gripperResponse(True)


'''

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

'''

def publisher():
    global pub
    """Main loop which requests new commands and publish them on the SModelRobotOutput topic."""

    rospy.init_node('gripper_service')

    pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output,queue_size=5)

    s = rospy.Service('command_gripper', gripper, genCommand)

    rospy.spin()


if __name__ == '__main__':
    publisher()
