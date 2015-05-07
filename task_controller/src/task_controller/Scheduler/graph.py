
import random
import json
from collections import defaultdict


class Node:
    def __init__(self, bin, type, is_wide, item):
        self.bin = bin
        self.type = type
        self.is_wide = is_wide
        self.item = item
        self.alternatives = []
        # pick and place + vision + grasp
        values = {0: 10, 1: 15, 2: 20}
        self.value = random.randint(0, 5) + values[random.randint(0, 2)]
        self.execution_time = (60+60*random.random()) + 10*random.random() + (30+30*random.random())

    def current_scoop(self):
        if self.type == "grab-empty":
            return "None"
        elif self.type == "grab-wrong-scoop":
            if self.is_wide:
                return "Narrow"
            else:
                return "Wide"
        else:
            if self.is_wide:
                return "Wide"
            else:
                return "Narrow"

    def random_alternative(self):
        if len(self.alternatives) == 0:
            return self  # No other options
        return self.alternatives[random.randint(0, len(self.alternatives)-1)]

    def __repr__(self):
        return "%s<%s, %s>" % (self.item, self.bin, self.type)


def make_edge(i, j):
    if i.current_scoop() == j.current_scoop():
        cost = 0 + j.execution_time
    elif i.current_scoop() == "None" or j.current_scoop() == "None":
        cost = 40 + j.execution_time
    else:
        cost = 80 + j.execution_time

    return cost


def generate_graph(bins, types, wide, which_item=lambda bin: "None",
                   drop=lambda bin, type: False):
    nodes = []
    for bin in bins:
        for type in types:
            if drop(bin, type):
                continue
            item = which_item(bin)
            if not item:
                continue
            nodes.append(Node(bin, type, bin in wide, item))

    values = {}
    sets = defaultdict(lambda: [])
    for i in nodes:
        sets[i.bin].append(i)
        values[i.bin] = i.value

    edges = {}
    for i in nodes:
        for j in nodes:
            if i == j:
                continue
            if i.item == j.item:
                i.alternatives.append(j)
                continue
            edge = ((i.bin, sets[i.bin].index(i)), (j.bin, sets[j.bin].index(j)))
            print edge
            edges[edge] = make_edge(i, j)
            print edge, edges[edge]

    return (nodes, values, edges, sets)


def test_graph():
    return generate_graph(
        ["A", "B", "C", "D"],
        ["grab-empty", "grab-wrong-scoop", "grab-right-scoop", "scoop"],
        # types = ["grasp", "wrong", "right", "scoop"],
        ["B", "D"],
        drop=lambda bin, type: bin == "B" and type != "scoop"
    )


def full_graph(filename="code/example.json"):
    with open(filename) as json_file:
        data = json.load(json_file)

    def which_item(bin):
        for element in data["work_order"]:
            if bin == element["bin"][-1]:
                return element["item"]
        print "WARNING: No item for bin %s" % bin
        return None

    def drop(bin, type):
        # Currently, we must use scoop when multiple items are in the bin
        if len(data["bin_contents"]["bin_"+bin]) > 1 and type != "scoop":
            return True
        return False

    return generate_graph(
        "ABCDEFGHIJKL",
        ["grab-empty", "grab-wrong-scoop", "grab-right-scoop", "scoop"],
        "BEHK",
        which_item,
        drop,
    )
