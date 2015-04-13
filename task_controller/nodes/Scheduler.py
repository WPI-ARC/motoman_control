import roslib; roslib.load_manifest('task_controller')
import rospy
import smach


class SIMPLESCHEDULER(smach.State):

    def __init__(self, schedule):
        smach.State.__init__(self, outcomes=['Pick', 'Scoop', 'ToolChange', 'Success', 'Failure', 'Fatal'],
                             input_keys=[], output_keys=['item', 'bin'])
        self.schedule = schedule
        self.location = 0

    def execute(self, data):
        rospy.loginfo("Scheduler running...")

        if self.location < len(self.schedule):
            action, bin, item = self.schedule[self.location]
            data.item = item
            data.bin = bin
            print action, bin, item
            self.location += 1
            if action.startswith("grab"):
                rospy.loginfo("Scheduling pick of %s from bin %s." % (item, bin))
                return 'Pick'
            elif action == 'scoop':
                rospy.loginfo("Scheduling scoop of %s from bin %s." % (item, bin))
                return 'Scoop'
            else:
                return 'Failure'

        return 'Success'
