#!/usr/bin/python

import rospy
import json
import os

from task_controller.srv import SetSchedule
from apc_msgs.msg import APCItem, ScheduleItem


def schedule_from_order(order):
    single_items = []
    double_items = []
    multiple_items = []

    ranks = rospy.get_param("/item_priorities")
    for item in order:
        rank = ranks.index(item.name)
        if len(item.contents) == 1:
            single_items.append((rank, item))
        elif len(item.contents) == 2:
            double_items.append((rank, item))
        else:
            multiple_items.append((rank, item))

    single_items.sort()
    double_items.sort()
    multiple_items.sort()

    schedule = []
    for _, element in single_items:
        schedule.append(ScheduleItem(
            action="grab",
            name=element.name,
            bin=element.bin,
            contents=element.contents,
        ))
    for _, element in double_items:
        schedule.append(ScheduleItem(
            action="grab",
            name=element.name,
            bin=element.bin,
            contents=element.contents,
        ))
    for _, element in multiple_items:
        schedule.append(ScheduleItem(
            action="grab",
            name=element.name,
            bin=element.bin,
            contents=element.contents,
        ))
    return schedule

if __name__ == '__main__':
    rospy.init_node("handle_order")

    order_file = rospy.get_param("~order", "default.json")

    if not order_file.startswith("/"):
        order_file = os.path.join(os.path.dirname(__file__),
                                  "../orders", order_file)
    print order_file
    with open(order_file) as file:
        raw_order = json.load(file)

    order = []
    for element in raw_order["work_order"]:
        order.append(APCItem(
            key=element["bin"][-1]+"-"+element["item"],  # TODO: validate unique
            name=element["item"],
            bin=element["bin"][-1],
            contents=raw_order["bin_contents"][element["bin"]],
        ))

    print order

    schedule = schedule_from_order(order)

    print schedule

    set_schedule = rospy.ServiceProxy('set_schedule', SetSchedule)
    rospy.loginfo("Waiting for service `/set_schedule`")
    set_schedule.wait_for_service()
    rospy.loginfo("`/set_schedule` available")
    set_schedule(schedule)
