#!/usr/bin/env python

import random
from copy import deepcopy

from brute_force import brute_force


MAX_NUM = 99999999


def random_sample(bins, sets):
    def sample():
        random.shuffle(bins)
        return [(bin, random.choice(sets[bin])) for bin in bins]
    return sample


def choose(tours):
    current, total = 0, sum(fitness for fitness, _ in tours)
    target = random.random()*total
    for score, tour in tours:
        current += score
        if current > target:
            return deepcopy(tour)


def mutate(sets, swap_places=0.2, swap_set=0.2, reverse_segment=0.2, crossover=0.2,
           optimize=0, optimize_length=2, cost=None):
    def mutate(tours):
        tour = choose(tours)
        p = random.random()
        if p < swap_places:
            i, j = random.randint(0, len(tour)-1), random.randint(0, len(tour)-1)
            tour[i], tour[j] = tour[j], tour[i]
        elif p < (swap_places+swap_set):
            i = random.randint(0, len(tour)-1)
            tour[i] = (tour[i][0], random.choice(sets[tour[i][0]]))
        elif p < (swap_places+swap_set+reverse_segment):
            i, j = random.randint(0, len(tour)-1), random.randint(0, len(tour)-1)
            if i > j: i, j = j, i
            tour[i:j] = reversed(tour[i:j])
        elif p < (swap_places+swap_set+reverse_segment+optimize):
            i = random.randint(0, len(tour)-1-optimize_length)
            subtour = [key for key, _ in tour[i:i+optimize_length]]
            subset = {key: value for key, value in sets.items() if key in subtour}
            _, tour[i:i+optimize_length] = brute_force(subset, cost)
        elif p < (swap_places+swap_set+reverse_segment+optimize+crossover):
            other = dict(choose(tours))
            tour = [(bin, other[bin]) for (bin, _) in tour]
        return tour
    return mutate


def genetic_algorithm(sets, cost, sample, mutate, population=100, generations=1000):
    num_elites, num_new = 5, 5
    population = [sample() for i in range(population)]
    bestScore, best = -MAX_NUM, None

    # Run loop
    for generation in range(generations):
        scores = [cost(tour) for tour in population]
        # print "Generation %s, best score is %s" % (generation, max(scores))
        for tour, score in zip(population, scores):
            if score > bestScore:
                bestScore, best = score, tour

        # Choose elites
        elites = list(reversed(sorted(zip(scores, population))))
        new_population = [i[1] for i in elites[:num_elites]]
        # Breed new
        for i in range(len(population)-num_elites-num_new):
            new_population.append(mutate(elites))
        # Sample new
        for i in range(num_new):
            new_population.append(sample())
        # print new_population
        population = new_population

    return bestScore, best

def genetic_algorithm_plot(sets, cost, sample, mutate, population=100, generations=1000):
    num_elites, num_new = 5, 5
    population = [sample() for i in range(population)]
    bestScore, best = -MAX_NUM, []

    # Run loop
    for generation in range(generations):
        scores = [cost(tour) for tour in population]
        # print "Generation %s, best score is %s" % (generation, max(scores))
        for tour, score in zip(population, scores):
            if score > bestScore:
                bestScore = score
        best.append(bestScore)

        # Choose elites
        elites = list(reversed(sorted(zip(scores, population))))
        new_population = [i[1] for i in elites[:num_elites]]
        # Breed new
        for i in range(len(population)-num_elites-num_new):
            new_population.append(mutate(elites))
        # Sample new
        for i in range(num_new):
            new_population.append(sample())
        # print new_population
        population = new_population

    return bestScore, best

if __name__ == "__main__":
    import sys
    from run_tests import load_dataset, cost

    values, sets, edges, max_cost = load_dataset(sys.argv[1])
    mult = lambda x, y: x * y
    factorial = lambda n: reduce(mult, range(1, n+1))
    num_perms =  factorial(len(sets)) * reduce(mult, [len(set) for set in sets.values()])
    print "Solving for %s sets connected by %s edges totalling %s permutations with a max cost of %s." \
        % (len(sets), len(edges), num_perms, max_cost)

    score, tour = genetic_algorithm(sets, cost(values, edges, max_cost), random_sample(sets.keys(), sets), mutate(sets))

    print
    print score, tour
