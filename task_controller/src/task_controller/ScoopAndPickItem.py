import roslib; roslib.load_manifest('task_controller')
import smach

import Scoop

def SCOOPANDPICKITEM(robot):
    sm = smach.StateMachine(
        outcomes=['Success', 'Failure', 'Fatal'],
        input_keys=['input', 'bin', 'item'],
        output_keys=['output']
    )

    # Populate the state machine from the modules
    with sm:
        smach.StateMachine.add(
            'Scoop', Scoop.SCOOP(robot),
            transitions={'Success': 'Success', 'Failure': 'Failure', 'Fatal': 'Fatal'},
            # remapping={'output': 'sm_data'}
        )

    return sm
