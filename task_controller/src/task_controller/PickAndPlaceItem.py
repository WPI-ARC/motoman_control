import roslib; roslib.load_manifest('task_controller')
import smach

from MoveToBin import MoveToBin
from ScanForItem import ScanForItem
from PickItem import PickItem
from PlaceItem import PlaceItem


def PickAndPlaceItem(robot):
    """
    State machine to pick and place a single item.
    """

    sm = smach.StateMachine(
        outcomes=['Success', 'Failure', 'Fatal'],
        input_keys=['bin', 'item', 'contents'],
    )

    # Populate the state machine from the modules
    with sm:
        smach.StateMachine.add(
            'MoveToBin', MoveToBin(robot),
            transitions={'Success': 'ScanForItem', 'Failure': 'Failure', 'Fatal': 'Fatal'},
        )

        smach.StateMachine.add(
            'ScanForItem', ScanForItem(robot),
            transitions={'Success': 'PickItem', 'Failure': 'Failure', 'Fatal': 'Fatal'},
        )

        smach.StateMachine.add(
            'PickItem', PickItem(robot),
            transitions={'Success': 'PlaceItem', 'Failure': 'Failure', 'Fatal': 'Fatal'},
        )

        smach.StateMachine.add(
            'PlaceItem', PlaceItem(robot),
            transitions={'Success': 'Success', 'Failure': 'Failure', 'Fatal': 'Fatal'},
        )

    return sm
