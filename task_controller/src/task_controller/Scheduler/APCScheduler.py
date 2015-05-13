import rospy
import smach
from collections import defaultdict
from trajlib.srv import GetTrajectory

from genetic_algorithm import genetic_algorithm, mutate, random_sample

trajlib = rospy.ServiceProxy("/trajlib", GetTrajectory)


class APCScheduler(smach.State):

    def __init__(self, order):
        smach.State.__init__(self, outcomes=['Pick', 'Scoop', 'ToolChange', 'Success', 'Failure', 'Fatal'],
                             input_keys=[], output_keys=['item', 'bin'])
        self.order = order
        self.items = {item.key: item for item in self.order.items}
        self.picked_items = []
        self.schedule = None
        self.location = 0

        if self.schedule is None:
            self.plan_schedule()

    def execute(self, data):
        rospy.loginfo("Scheduler running...")

        if self.location < len(self.schedule):
            current = self.schedule[self.location]
            action = current["action"]
            self.location += 1

            if action.startswith("grab"):
                data.item = current["item"]
                data.bin = current["bin"]
                print action, current["bin"], current["item"]
                rospy.loginfo("Scheduling pick of %s from bin %s." % (current["item"], current["bin"]))
                return 'Pick'

            elif action == 'scoop':
                data.item = current["item"]
                data.bin = current["bin"]
                rospy.loginfo("Scheduling scoop of %s from bin %s." % (current["item"], current["bin"]))
                return 'Scoop'

            elif action == "pick_scoop":
                rospy.loginfo("Picking scoop up.")
                return 'ToolChange'

            else:
                return 'Failure'

        return 'Success'

    def plan_schedule(self):
        values, sets, edges, max_cost = self.formulate_problem()
        mult = lambda x, y: x * y
        factorial = lambda n: reduce(mult, range(1, n+1))
        num_perms = factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
        rospy.loginfo("Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                      % (len(sets), len(edges), num_perms, max_cost))

        score, tour = genetic_algorithm(
            sets,
            cost(values, edges, max_cost),
            random_sample(sets.keys(), sets),
            mutate(sets),
            population=100,
            generations=500
        )

        self.schedule = []
        for key, method in tour:
            self.schedule.append({
                "action": method,
                "bin": self.items[key].bin,
                "item": self.items[key].name,
                "others": self.items[key].contents
            })

        print score, tour
        print self.schedule

    def formulate_problem(self):
        max_cost = 60*19  # Find a 19 minute plan

        # TODO: Filter out unsupported items

        values = {}
        for item in self.order.items:
            if not get_item_property(item.name, "supported", False):
                rospy.logwarn("Item %s is not currently supported." % item.name)
                continue
            values[item.key] = 0
            if len(item.contents) == 1:
                values[item.key] += 10
            elif len(item.contents) == 2:
                values[item.key] += 15
            else:
                values[item.key] += 20
            values[item.key] += get_item_property(item.name, "bonus", 0)

        sets = defaultdict(lambda: [])
        nodes = []
        for item in self.order.items:
            if not get_item_property(item.name, "supported", False):
                continue
            sets[item.key].append("grab")
            nodes.append((item.key, "grab"))
            # TODO: Add scooping back in
            # sets[item.key].append("scoop")
            # nodes.append((item.key, "scoop"))
            # TODO: Add support for right hand grab?

        edges = {}
        for i in nodes:
            edges[("Start", i)] = self.calculate_cost(i[0])
            for j in nodes:
                if i == j:
                    continue
                edges[(i, j)] = self.calculate_cost(j[0])

        from pprint import pprint
        # pprint(max_cost)
        # pprint(values)
        # pprint(sets)
        pprint(edges)

        return values, sets, edges, max_cost

    def calculate_cost(self, item):
        # TODO: Check status
        # traj_to_bin = trajlib(task="Forward", bin_num=self.items[item].bin)
        # traj_from_bin = trajlib(task="Drop", bin_num=self.items[item].bin)

        SCAN_TIME = 20  # TODO: measure
        # time_to_bin = traj_to_bin.plan.joint_trajectory.points[-1].time_from_start.to_sec()
        PICK_TIME = 25  # TODO: measure
        # time_to_drop = traj_from_bin.plan.joint_trajectory.points[-1].time_from_start.to_sec()
        DROP_TIME = 10  # TODO: measure
        # return SCAN_TIME + time_to_bin + PICK_TIME + time_to_drop + DROP_TIME
        return 1


def get_item_property(item, property, default):
    return rospy.get_param("/items/"+item+"/"+property, default)


# TODO: Load prob_catastrophic_failure
def cost(values, edges, max_cost, prob_catastrophic_failure=0.2):
    def real_cost(tour):
        score = values[tour[0][0]]
        prob_continue = 1

        cost = edges[("Start", tour[0])]
        prob_continue *= (1. - prob_catastrophic_failure)
        for edge in zip(tour, tour[1:]):
            # print cost, score
            if (cost + edges[edge]) <= max_cost:
                cost += edges[edge]
                score += prob_continue*values[edge[1][0]]
                prob_continue *= (1. - prob_catastrophic_failure)
            else:
                break
        # print cost, score
        bonus = max(min((max_cost-cost)/max_cost, 1), 0)
        return score + bonus
    return real_cost
