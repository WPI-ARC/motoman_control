import rospy
import smach
import smach_ros

from task_controller.ScanAllBins import ScanAllBins
from task_controller.PickAndPlaceItem import PickAndPlaceItem
from task_controller.ScoopAndPickItem import ScoopAndPickItem
from task_controller.PickScoop import PickScoop
from task_controller.FinishTask import FinishTask
from task_controller.SafeMode import SafeMode
from task_controller.ErrorHandler import ErrorHandler
from apc_util.moveit import robot


class APCController:

    def __init__(self, scheduler):
        rospy.loginfo("Starting APC Controller...")
        rospy.on_shutdown(self.cleanup)
        # Initialize the state machine
        self.running = False
        self.safed = False

        self.sm = smach.StateMachine(outcomes=['DONE', 'FAILED', 'SAFE'])

        # Populate the state machine from the modules
        with self.sm:
            self.safemode = SafeMode()

            smach.StateMachine.add(
                'ScanAllBins', ScanAllBins,
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'Scheduler', scheduler,
                transitions={'Pick': 'PickAndPlaceItem', 'Scoop': 'ScoopAndPickItem', 'ToolChange': 'PickScoop',
                             'Success': 'FinishTask', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'PickAndPlaceItem',
                PickAndPlaceItem(robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'ScoopAndPickItem',
                ScoopAndPickItem(robot),
                transitions={'Success': 'Scheduler', 'Failure': 'ErrorHandler', 'Fatal': 'SafeMode'},
            )

            smach.StateMachine.add(
                'PickScoop',
                PickScoop(robot),
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
