import roslib; roslib.load_manifest('task_controller')
import rospy
import smach


class SimpleScheduler(smach.State):

    def __init__(self, schedule):
        smach.State.__init__(self, outcomes=['Pick', 'Scoop', 'ToolChange', 'Success', 'Failure', 'Fatal'],
                             input_keys=[], output_keys=['item', 'bin', 'contents'])
        self.schedule = schedule
        self.location = 0

    def execute(self, data):
        rospy.loginfo("Scheduler running...")

        if self.location < len(self.schedule):
            current = self.schedule[self.location]
            action = current["action"]
            self.location += 1

            if action.startswith("grab"):
                data.item = current["item"]
                data.bin = current["bin"]
                data.contents = current["others"]
                print action, current["bin"], current["item"]
                rospy.loginfo("Scheduling pick of %s from bin %s." % (current["item"], current["bin"]))
                return 'Pick'

            elif action == 'scoop':
                data.item = current["item"]
                data.bin = current["bin"]
                data.contents = current["others"]
                rospy.loginfo("Scheduling scoop of %s from bin %s." % (current["item"], current["bin"]))
                return 'Scoop'

            elif action == "pick_scoop":
                rospy.loginfo("Picking scoop up.")
                return 'ToolChange'

            else:
                return 'Failure'

        return 'Success'
