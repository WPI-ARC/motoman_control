#!/usr/bin/python

import rospy
import argparse

from apc_util.shelf import Shelf, add_shelf, remove_shelf

qualities = {
    "NONE": Shelf.NONE,
    "SIMPLE": Shelf.SIMPLE,
    "FULL": Shelf.FULL,
    "PADDED": Shelf.PADDED,
    "A": Shelf.BIN_A,
    "B": Shelf.BIN_B,
    "C": Shelf.BIN_C,
    "D": Shelf.BIN_D,
    "E": Shelf.BIN_E,
    "F": Shelf.BIN_F,
    "G": Shelf.BIN_G,
    "H": Shelf.BIN_H,
    "I": Shelf.BIN_I,
    "J": Shelf.BIN_J,
    "K": Shelf.BIN_K,
    "L": Shelf.BIN_L
}


if __name__ == '__main__':
    rospy.init_node("shelf_collsion")

    parser = argparse.ArgumentParser(description="Publish shelf collision")
    parser.add_argument("--remove", help="Remove the shelf",
                        action="store_true")
    parser.add_argument("--quality", default="Full",
                        help="Shelf quality (None, Simple, Full, Padded)")

    args = parser.parse_args()

    if args.remove:
        remove_shelf()
    else:
        add_shelf(qualities[args.quality.upper()])
