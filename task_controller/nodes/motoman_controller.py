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

import PickAndPlaceItem
import ScoopAndPickItem
import PickScoop
import Scheduler
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
            # input_keys=['sm_input', 'bin', 'item'],
            input_keys=['sm_input'],
            output_keys=['sm_output']
        )

        # Populate the state machine from the modules
        with self.sm:
            self.safemode = SafeMode.SAFEMODE()

            # schedule = [("grab_empty", "B", "elmers_washable_no_run_school_glue")]
            # schedule = [("grab_empty", "A", "crayola_64_ct"),
            #             ("grab_empty", "B", "elmers_washable_no_run_school_glue")]
            # schedule = [("scoop", "C", "elmers_washable_no_run_school_glue")]
            schedule = [("grab_empty", "B", "feline_greenies_dental_treats")]
            # schedule = [("grab_empty", "B", "crayola_64_ct")]

            smach.StateMachine.add(
                'Scheduler', Scheduler.SIMPLESCHEDULER(schedule),
                transitions={'Pick': 'PickAndPlaceItem', 'Scoop': 'ScoopAndPickItem', 'ToolChange': 'PickScoop',
                             # 'Scoop': 'ScoopAndPlaceItem', 'ToolChange': 'ChangeScoop',
                             'Success': 'FinishTask', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'PickAndPlaceItem',
                PickAndPlaceItem.PICKANDPLACEITEM(self.robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_input', 'output': 'sm_data'}
            )

            smach.StateMachine.add(
                'ScoopAndPickItem',
                ScoopAndPickItem.SCOOPANDPICKITEM(self.robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_input', 'output': 'sm_data'}
            )

            smach.StateMachine.add(
                'PickScoop',
                PickScoop.PICKSCOOP(self.robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
                remapping={'input': 'sm_input', 'output': 'sm_data'}
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
                transitions={'ReMove': 'SafeMode', 'ReScan': 'SafeMode', 'RePick': 'SafeMode',
                             'ReFinish': 'SafeMode', 'Failed': 'FAILED', 'Fatal': 'SafeMode'},
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
        print "Starting..."
        print self.sm.userdata.sm_input
        print "...starting"
        final_outcome = self.sm.execute()
        print "Finished:", final_outcome
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
