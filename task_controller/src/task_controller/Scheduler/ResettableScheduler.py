import rospy
import smach


from threading import Lock


class ResettableScheduler(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Pick', 'Scoop', 'Success', 'Failure', 'Fatal'],
                             input_keys=[], output_keys=['item', 'bin', 'contents'])
        self._m = Lock()
        self.schedule = []
        self.location = 0

    def execute(self, data):
        rospy.loginfo("Scheduler running...")

        schedule, location = [], 0
        while location >= len(schedule):
            rospy.sleep(1)
            # Get snapshot of current state
            self._m.acquire()
            schedule, location = self.schedule, self.location
            self._m.release()

        if location < len(schedule):
            current = schedule[location]
            self.location += 1

            if current.action.startswith("grab"):
                data.item = current.name
                data.bin = current.bin
                data.contents = current.contents
                print current.action, current.bin, current.name
                rospy.loginfo("Scheduling pick of %s from bin %s." % (current.name, current.bin))
                return 'Pick'

            elif current.action == 'scoop':
                data.item = current.name
                data.bin = current.bin
                data.contents = current.contents
                rospy.loginfo("Scheduling scoop of %s from bin %s." % (current.name, current.bin))
                return 'Scoop'

            else:
                return 'Failure'

        return 'Success'

    def set_schedule(self, schedule):
        self._m.acquire()
        self.schedule, self.location = schedule, 0
        self._m.release()
        rospy.loginfo("New schedule loaded")
