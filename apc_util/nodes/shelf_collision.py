#!/usr/bin/python

import rospy
import argparse

from apc_util.shelf import Shelf, add_shelf, remove_shelf

qualities = {
    "None": Shelf.NONE,
    "Simple": Shelf.SIMPLE,
    "Full": Shelf.FULL,
}


if __name__ == '__main__':
    rospy.init_node("shelf_collsion")

    parser = argparse.ArgumentParser(description="Publish shelf collision")
    parser.add_argument("--remove", help="Remove the shelf",
                        action="store_true")
    parser.add_argument("--quality", default="Full",
                        choices=["None", "Simple", "Full"],
                        help="Shelf quality (None, Simple, Full)")

    args = parser.parse_args()

    if args.remove:
        remove_shelf()
    else:
        add_shelf(qualities[args.quality])
