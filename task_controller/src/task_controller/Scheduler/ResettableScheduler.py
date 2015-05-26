import rospy
import smach


from threading import Lock


class ResettableScheduler(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['Pick', 'Scoop', 'Success', 'Failure', 'Fatal'],
                             input_keys=['new_information'], output_keys=['action', 'item', 'bin', 'contents', 'new_information'])
        self._m = Lock()
        self.schedule = []
        self.location = 0

    def execute(self, data):
        rospy.loginfo("Scheduler running...")

        if data.new_information is not None:
            no_scoop = rospy.get_param("/no_scoop")
            # Add suggested scoop if it makes sense
            if data.new_information.name not in no_scoop and len(data.new_information.contents) <= 2:
                self.schedule.append(data.new_information)
        data.new_information = None

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
                data.action = "grab"
                data.item = current.name
                data.bin = current.bin
                data.contents = current.contents
                print current.action, current.bin, current.name
                rospy.loginfo("Scheduling pick of %s from bin %s." % (current.name, current.bin))
                return 'Pick'

            elif current.action == 'scoop':
                data.action = "scoop"
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
