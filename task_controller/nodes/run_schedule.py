#!/usr/bin/python

import rospy
import yaml
import os

from task_controller.srv import SetSchedule
from apc_msgs.msg import ScheduleItem

if __name__ == '__main__':
    rospy.init_node("run_schedule")

    schedule_file = rospy.get_param("~schedule", "default.yaml")

    if not schedule_file.startswith("/"):
        schedule_file = os.path.join(os.path.dirname(__file__),
                                     "../schedules", schedule_file)
    print schedule_file
    with open(schedule_file) as file:
        raw_schedule = yaml.load(file)

    schedule = []
    for element in raw_schedule:
        schedule.items.append(ScheduleItem(
            action=element["actkion"],
            name=element["item"],
            bin=element["bin"][-1],
            contents=element["others"],
        ))
    print schedule

    set_schedule = rospy.ServiceProxy('set_schedule', SetSchedule)
    rospy.loginfo("Waiting for service `/set_schedule`")
    set_schedule.wait_for_service()
    rospy.loginfo("`/set_schedule` available")
    set_schedule(schedule)
