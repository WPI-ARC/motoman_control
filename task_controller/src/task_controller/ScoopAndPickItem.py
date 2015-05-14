import smach

from PushWithScoop import PushWithScoop
from Scoop import Scoop


def ScoopAndPickItem(robot):
    """
    State machine to scoop out a bin and eventually pick the item.
    """

    sm = smach.StateMachine(
        outcomes=['Success', 'Failure', 'Fatal'],
        input_keys=['input', 'bin', 'item'],
        output_keys=['output']
    )

    # Populate the state machine from the modules
    with sm:
        # smach.StateMachine.add(
        #     'PushWithScoop', PushWithScoop(robot),
        #     transitions={'Success': 'Success', 'Failure': 'Failure', 'Fatal': 'Fatal'},
        # )
        smach.StateMachine.add(
            'Scoop', Scoop(robot),
            transitions={'Success': 'Success', 'Failure': 'Failure', 'Fatal': 'Fatal'},
            # remapping={'output': 'sm_data'}
        )

    return sm
