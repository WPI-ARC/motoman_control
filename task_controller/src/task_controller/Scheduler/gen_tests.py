#!/usr/bin/env python

import yaml
import random

from graph import generate_graph

APC_BINS = "ABCDEFGHIJKL"


def apc_graph(scoop=""):
    def which_item(bin):
        return "Item-"+bin

    def drop(bin, type):
        if bin in scoop and type != "scoop":
            return True
        return False

    return generate_graph(
        "ABCDEFGHIJKL",
        ["grab-empty", "grab-wrong-scoop", "grab-right-scoop", "scoop"],
        "BEHK",
        which_item,
        drop,
    )


if __name__ == "__main__":
    simple_pattern = "datasets/simple-%d.yaml"
    testable_pattern = "datasets/testable-%d.yaml"
    simple_sets = 4
    apc_pattern = "datasets/apc-%d.yaml"

    for i in range(4, 9):
        sets = {}
        for bin in APC_BINS[:i]:
            sets[bin] = range(random.randint(simple_sets/2, simple_sets))

        edges = []
        for from_set in sets.keys():
            for from_perm in sets[from_set]:
                for to_set in sets.keys():
                    if from_set == to_set: continue
                    for to_perm in sets[to_set]:
                        edges.append({"from": {"set": from_set, "perm": from_perm},
                                      "to": {"set": to_set, "perm": to_perm},
                                      "cost": 50+50*random.random()})

        values = {}
        for bin in APC_BINS[:8]:
            values[bin] = random.randint(2, 6)

        data = {
            "sets": sets,
            "edges": edges,
            "values": values,
            "max_value": sum(values.values()),
            "max_cost": (i-1)*70,
        }
        with file(simple_pattern % i, "w") as f:
            yaml.dump(data, f)

    for i in range(10):
        sets = {}
        for bin in APC_BINS[:8]:
            sets[bin] = range(random.randint(simple_sets/2, simple_sets))

        edges = []
        for from_set in sets.keys():
            for from_perm in sets[from_set]:
                for to_set in sets.keys():
                    if from_set == to_set: continue
                    for to_perm in sets[to_set]:
                        edges.append({"from": {"set": from_set, "perm": from_perm},
                                      "to": {"set": to_set, "perm": to_perm},
                                      "cost": 50+50*random.random()})

        values = {}
        for bin in APC_BINS[:8]:
            values[bin] = random.randint(2, 6)

        data = {
            "sets": sets,
            "edges": edges,
            "values": values,
            "max_value": sum(values.values()),
            "max_cost": 300,
        }
        with file(testable_pattern % i, "w") as f:
            yaml.dump(data, f)

    for i in range(10):
        # Randomly assign roughly half of the bins to be scoop only
        scoop = ""
        for bin in APC_BINS:
            if random.random() < 0.5:
                scoop += bin

        # Generate the graph
        _, values, edges, sets = apc_graph(scoop)
        print i, scoop

        # Save the graph
        data = {
            "sets": {key: range(len(value)) for key, value in sets.items()},
            "edges": [{"from": {"set": from_set, "perm": from_perm},
                       "to": {"set": to_set, "perm": to_perm},
                       "cost": edge}
                      for ((from_set, from_perm), (to_set, to_perm)), edge in edges.items()],
            "values": values,
            "max_value": sum(values.values()),
            "max_cost": 1200,
        }
        with file(apc_pattern % i, "w") as f:
            yaml.dump(data, f)
