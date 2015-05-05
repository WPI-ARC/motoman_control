#!/usr/bin/python

import roslib; roslib.load_manifest('task_controller')
import rospy
import smach
import smach_ros
import yaml
import os
import moveit_commander

from task_controller.PickAndPlaceItem import PickAndPlaceItem
from task_controller.ScoopAndPickItem import ScoopAndPickItem
from task_controller.PickScoop import PickScoop
from task_controller.Scheduler import SimpleScheduler
from task_controller.FinishTask import FinishTask
from task_controller.SafeMode import SafeMode
from task_controller.ErrorHandler import ErrorHandler


class MotomanController:

    def __init__(self, filename):
        rospy.loginfo("Starting APC task controller...")
        rospy.on_shutdown(self.cleanup)
        # Initialize the state machine
        self.running = False
        self.safed = False
        self.robot = moveit_commander.RobotCommander()

        self.sm = smach.StateMachine(outcomes=['DONE', 'FAILED', 'SAFE'])

        if not filename.startswith("/"):
            filename = os.path.join(os.path.dirname(__file__), "..", filename)
        print filename
        with open(filename) as file:
            schedule = yaml.load(file)
        print schedule

        # Populate the state machine from the modules
        with self.sm:
            self.safemode = SafeMode.SAFEMODE()

            smach.StateMachine.add(
                'Scheduler', SimpleScheduler(schedule),
                transitions={'Pick': 'PickAndPlaceItem', 'Scoop': 'ScoopAndPickItem', 'ToolChange': 'PickScoop',
                             'Success': 'FinishTask', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'PickAndPlaceItem',
                PickAndPlaceItem(self.robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'ScoopAndPickItem',
                ScoopAndPickItem.SCOOPANDPICKITEM(self.robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'PickScoop',
                PickScoop(self.robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'FinishTask', FinishTask(),
                transitions={'Success': 'DONE', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'SafeMode', self.safemode,
                transitions={'Safed': 'SAFE'},
            )

            smach.StateMachine.add(
                'ErrorHandler', ErrorHandler(),
                transitions={'ReMove': 'SafeMode', 'ReScan': 'SafeMode', 'RePick': 'SafeMode',
                             'ReFinish': 'SafeMode', 'Failed': 'FAILED', 'Fatal': 'SafeMode'},
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
    rospy.init_node("motoman_apc_controller")
    schedule_file = rospy.get_param("~schedule", "schedules/default.yaml")
    controller = MotomanController(schedule_file)
    controller.Start()
