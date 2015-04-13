import roslib; roslib.load_manifest('task_controller')
import smach

import MoveToBin
import ScanForItem
import PickItem
import PlaceItem

def PICKANDPLACEITEM(robot):
    sm = smach.StateMachine(
        outcomes=['Success', 'Failure', 'Fatal'],
        input_keys=['input', 'bin', 'item'],
        output_keys=['output']
    )

    # Populate the state machine from the modules
    with sm:
        smach.StateMachine.add(
            'MoveToBin', MoveToBin.MOVETOBIN(robot),
            transitions={'Success': 'ScanForItem', 'Failure': 'Failure', 'Fatal': 'Fatal'},
            # remapping={'output': 'sm_data'}
        )

        smach.StateMachine.add(
            'ScanForItem', ScanForItem.SCANFORITEM(),
            transitions={'Success': 'PickItem', 'Failure': 'Failure', 'Fatal': 'Fatal'},
            remapping={'input': 'output'}
            # remapping={'input': 'sm_data', 'output': 'sm_data'}
        )

        smach.StateMachine.add(
            'PickItem', PickItem.PICKITEM(robot),
            transitions={'Success': 'PlaceItem', 'Failure': 'Failure', 'Fatal': 'Fatal'},
            remapping={'input': 'output'}
            # remapping={'input': 'sm_data', 'output': 'sm_data'}
        )

        smach.StateMachine.add(
            'PlaceItem', PlaceItem.PLACEITEM(robot),
            transitions={'Success': 'Success', 'Failure': 'Failure', 'Fatal': 'Fatal'},
            remapping={'input': 'output'}
            # remapping={'input': 'sm_data'}
        )

    return sm
