#!/usr/bin/env python

import os
import yaml

def load_dataset(filename):
    with file(filename) as f:
        data = yaml.safe_load(f)

    values = data["values"]
    sets = data["sets"]
    edges = {((edge["from"]["set"], edge["from"]["perm"]), (edge["to"]["set"], edge["to"]["perm"])):
             edge["cost"] for edge in data["edges"]}
    max_cost = data["max_cost"]

    return values, sets, edges, max_cost


def cost(values, edges, max_cost):
    def real_cost(tour):
        score = values[tour[0][0][0]]
        cost = 0.0
        for edge in zip(tour, tour[1:]):
            # print cost, score
            if (cost + edges[edge]) <= max_cost:
                cost += edges[edge]
                score += values[edge[1][0]]
            else:
                break
        # print cost, score
        bonus = max(min((max_cost-cost)/max_cost, 1), 0)
        return score + bonus
    return real_cost


def timeit(func, *args, **kwargs):
    start = time.time()
    score, tour = func(*args, **kwargs)
    stop = time.time()
    return score, tour, stop-start


if __name__ == "__main__":
    import time
    import sys
    from brute_force import brute_force
    from genetic_algorithm import genetic_algorithm, genetic_algorithm_plot, random_sample, mutate

    simple = "simple"
    testable = "testable"
    apc = "apc"
    datasets = "datasets/"

    if len(sys.argv) != 2:
        print "Error"
    elif sys.argv[1] == "bruteforce":  # Test simple against brute-force
        for filename in os.listdir(datasets):
            if simple not in filename: continue
            
            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, tour, runtime = timeit(brute_force, sets, cost(values, edges, max_cost))
            print "Brute Force:", score, runtime
            score, tour, runtime = timeit(genetic_algorithm, sets, cost(values, edges, max_cost),
                                          random_sample(sets.keys(), sets), mutate(sets))
            print "Genetic Algorithm:", score, runtime

    elif sys.argv[1] == "ga-pop":  # Compare GA population on apc 
        for filename in os.listdir(datasets):
            if apc not in filename: continue

            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, tour, runtime = timeit(genetic_algorithm, sets, cost(values, edges, max_cost),
                                          random_sample(sets.keys(), sets), mutate(sets), 10, 10000)
            print "Genetic Algorithm 10/10000:", score, runtime
            score, tour, runtime = timeit(genetic_algorithm, sets, cost(values, edges, max_cost),
                                          random_sample(sets.keys(), sets), mutate(sets), 100, 1000)
            print "Genetic Algorithm 100/1000:", score, runtime
            score, tour, runtime = timeit(genetic_algorithm, sets, cost(values, edges, max_cost),
                                          random_sample(sets.keys(), sets), mutate(sets), 1000, 100)
            print "Genetic Algorithm 1000/100:", score, runtime
            score, tour, runtime = timeit(genetic_algorithm, sets, cost(values, edges, max_cost),
                                          random_sample(sets.keys(), sets), mutate(sets), 10000, 10)
            print "Genetic Algorithm 10000/10:", score, runtime

    elif sys.argv[1] == "ga-mut":  # Compare GA population on apc 
        for filename in os.listdir(datasets):
            if apc not in filename: continue

            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm Normal:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0, reverse_segment=0)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm SWAP=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.6, swap_set=0, reverse_segment=0)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm SWAP=0.6:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0.2, reverse_segment=0)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm CHANGE=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0.6, reverse_segment=0)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm CHANGE=0.6:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0.2)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm INVERT=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0.6)
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm INVERT=0.6:", score, int(score), actual_cost, runtime

    elif sys.argv[1] == "ga-mut-opt":  # Compare GA population on apc 
        for filename in os.listdir(datasets):
            if apc not in filename: continue

            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0,
                       optimize=0.2, optimize_length=2, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm OPTIMIZE2=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0,
                       optimize=0.6, optimize_length=2, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm OPTIMIZE2=0.6:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0,
                       optimize=0.2, optimize_length=3, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm OPTIMIZE3=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0,
                       optimize=0.6, optimize_length=3, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm OPTIMIZE3=0.6:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0,
                       optimize=0.2, optimize_length=4, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm OPTIMIZE4=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0, swap_set=0, reverse_segment=0,
                       optimize=0.6, optimize_length=4, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm OPTIMIZE4=0.6:", score, int(score), actual_cost, runtime

    elif sys.argv[1] == "ga-mut-opt-extra":  # Compare GA population on apc 
        for filename in os.listdir(datasets):
            if apc not in filename: continue

            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0.2,
                       optimize=0.2, optimize_length=2, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm Default + OPTIMIZE2=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0.2,
                       optimize=0.2, optimize_length=3, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm Default + OPTIMIZE3=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0.2,
                       optimize=0.2, optimize_length=4, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm Default + OPTIMIZE4=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0,
                       optimize=0.2, optimize_length=2, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm SWAP=0.2, CHANGE=0.2, OPTIMIZE2=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0,
                       optimize=0.2, optimize_length=3, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm SWAP=0.2, CHANGE=0.2, OPTIMIZE3=0.2:", score, int(score), actual_cost, runtime

            score, tour, runtime = timeit(
                genetic_algorithm, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0,
                       optimize=0.2, optimize_length=4, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm SWAP=0.2, CHANGE=0.2, OPTIMIZE4=0.2:", score, int(score), actual_cost, runtime

    elif sys.argv[1] == "plot":  # Compare GA population on apc 
        for filename in os.listdir(datasets):
            if apc not in filename: continue

            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, best, runtime = timeit(
                genetic_algorithm_plot, sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets),
                mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0,
                       optimize=0.2, optimize_length=3, cost=cost(values, edges, max_cost))
            )
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm SWAP=0.2, CHANGE=0.2, OPTIMIZE3=0.2:", score, int(score), actual_cost, runtime
            print best
            print [int(score) for score in best]
            print [(1 - (score-int(score))) * max_cost for score in best]
            break

            
    elif sys.argv[1] == "brute2": # Test strenuous against brute-force
        for filename in os.listdir(datasets):
            if testable not in filename: continue
            
            print(filename)
            values, sets, edges, max_cost = load_dataset(datasets+filename)
            mult = lambda x, y: x * y
            factorial = lambda n: reduce(mult, range(1, n+1))
            num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
            print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
                % (len(sets), len(edges), num_perms, max_cost)

            score, tour, runtime = timeit(genetic_algorithm, sets, cost(values, edges, max_cost),
                                          random_sample(sets.keys(), sets), mutate(sets))
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Genetic Algorithm:", score, int(score), actual_cost, runtime
            
            score, tour, runtime = timeit(brute_force, sets, cost(values, edges, max_cost))
            actual_cost = (1 - (score-int(score))) * max_cost
            print "Brute Force:", score, int(score), actual_cost, runtime
