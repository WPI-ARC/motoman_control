#!/usr/bin/env python

from itertools import permutations

MAX_NUM = 99999999


def perms(bins, sets):
    perms = [0 for _ in bins]
    location = 0

    yield perms
    while True:
        perms[location] += 1
        while perms[location] >= len(sets[bins[location]]):
            perms[location] = 0
            location += 1
            if location >= len(bins):
                return
            perms[location] += 1
        location = 0
        yield perms


def tours(sets):
    total = 0
    for bins in permutations(sets.keys()):
        # print bins
        for perm in perms(bins, sets):
            # print perm
            total += 1
            yield zip(bins, perm)
    # print "Total permutations evaluated %s" % total


def brute_force(sets, cost):
    bestScore, best = -MAX_NUM, None
    for tour in tours(sets):
        score = cost(tour)
        # print "%0.2f: %s" % (score, tour)
        if score > bestScore:
            bestScore, best = score, tour
    return bestScore, best


if __name__ == "__main__":
    import sys
    from run_tests import load_dataset, cost

    values, sets, edges, max_cost = load_dataset(sys.argv[1])
    mult = lambda x, y: x * y
    factorial = lambda n: reduce(mult, range(1, n+1))
    num_perms = factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
    print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
        % (len(sets), len(edges), num_perms, max_cost)

    score, tour = brute_force(sets, cost(values, edges, max_cost))

    print
    print score, tour
