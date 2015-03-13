#!/usr/bin/python

import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros
import moveit_commander

import os
import sys
import subprocess
import threading
import ctypes

import MoveToBin
import ScanForItem
import PickItem
import FinishTask
import SafeMode
import ErrorHandler


class MotomanController:

    def __init__(self):
        rospy.loginfo("Starting APC task controller...")
        rospy.on_shutdown(self.cleanup)
        # Initialize the state machine
        self.running = False
        self.safed = False
        self.robot = moveit_commander.RobotCommander()
        
        self.sm = smach.StateMachine(
            outcomes=['DONE', 'FAILED', 'SAFE'],
            input_keys=['sm_input', 'bin', 'item'],
            output_keys=['sm_output']
        )
        
        # Populate the state machine from the modules
        with self.sm:
            self.safemode = SafeMode.SAFEMODE()

            smach.StateMachine.add(
                'MoveToBin', MoveToBin.MOVETOBIN(self.robot),
                transitions={'Success': 'ScanForItem', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_input', 'output': 'sm_data'}
            )

            smach.StateMachine.add(
                'ScanForItem', ScanForItem.SCANFORITEM(),
                transitions={'Success': 'PickItem', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_data', 'output': 'sm_data'}
            )

            smach.StateMachine.add(
                'PickItem', PickItem.PICKITEM(self.robot),
                transitions={'Success': 'FinishTask', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_data', 'output': 'sm_data'}
            )
            smach.StateMachine.add(
                'FinishTask', FinishTask.FINISHTASK(),
                transitions={'Success': 'DONE', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_data', 'output': 'sm_output'}
            )

            smach.StateMachine.add(
                'SafeMode', self.safemode,
                transitions={'Safed': 'SAFE'},
                remapping={'input': 'sm_data', 'output': 'sm_output'}
            )

            smach.StateMachine.add(
                'ErrorHandler', ErrorHandler.ERRORHANDLER(),
                transitions={'ReMove': 'MoveToBin', 'ReScan': 'ScanForItem', 'RePick': 'PickItem',
                             'ReFinish': 'FinishTask', 'Failed': 'FAILED', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_data', 'output': 'sm_output'}
            )
            
        # Set up the introspection server
        self.sis = smach_ros.IntrospectionServer('apc_smach_server', self.sm, '/SM_ROOT')
        self.sis.start()

    def Start(self):
        # Start the state machine
        self.running = True
        self.safed = False
        self.sm.userdata.sm_input = "Testing"
        self.sm.userdata.bin = "A"
        self.sm.userdata.item = "crayola"
        print "Starting..."
        print self.sm.userdata.sm_input
        print "...starting"
        final_outcome = self.sm.execute()
        self.running = False
        self.safemode.execute([])
        self.safed = True
        self.cleanup()

    def cleanup(self):
        # Complete any steps necessary for task controller shutdown
        if (self.running or not self.safed):
            rospy.logfatal("Forced shutdown of Motoman task controller...")
            self.sm.request_preempt()
            self.safemode.execute(None)
        rospy.loginfo("Shutting down Motoman task controller...")
        self.sis.stop()
        rospy.loginfo("...Motoman task controller shutdown complete")


if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_controller", shell=True)
    path = path.strip("\n")
    rospy.init_node("motoman_apc_controller")
    controller = MotomanController()
    controller.Start()
