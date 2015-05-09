#!/usr/bin/python

import roslib; roslib.load_manifest('task_controller')
import rospy
import json
import yaml
import os

from task_controller.Scheduler import APCScheduler
from task_controller.msg import WorkOrder, APCItem

if __name__ == '__main__':
    rospy.init_node("convert_order_to_schedule")

    order_file = rospy.get_param("~order", "default.json")

    if not order_file.startswith("/"):
        order_file = os.path.join(os.path.dirname(__file__),
                                  "../orders", order_file)
    print order_file
    with open(order_file) as file:
        raw_order = json.load(file)

    order = WorkOrder()
    for element in raw_order["work_order"]:
        order.items.append(APCItem(
            key=element["bin"][-1]+"-"+element["item"],  # TODO: validate unique
            name=element["item"],
            bin=element["bin"][-1],
            contents=raw_order["bin_contents"][element["bin"]],
        ))

    print order

    scheduler = APCScheduler(order)

    schedule_file = rospy.get_param("~schedule", "default.yaml")

    if not schedule_file.startswith("/"):
        schedule_file = os.path.join(os.path.dirname(__file__),
                                     "../schedules", schedule_file)

    with open(schedule_file, "w") as file:
        raw_order = yaml.dump(scheduler.schedule, file)
