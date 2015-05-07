
from copy import deepcopy
from random import shuffle
from graph import full_graph as graph

nodes, edges = graph()

def pairs(nodes):
    r1, r2 = range(nodes), range(nodes)
    shuffle(r1)
    shuffle(r2)
    for i in r1:
        for j in r2:
            if i != j:
                # yield (nodes[i], nodes[j])
                yield i, j

def swapped_nodes(nodes):
    for i, j in pairs(len(nodes)):
        if i < j:
            copy = nodes[:]
            copy[i], copy[j] = nodes[j], nodes[i]
            yield copy
            copy[i], copy[j] = copy[i].random_alternative(), copy[j]
            yield copy
            copy[i], copy[j] = copy[i], copy[j].random_alternative()
            yield copy
            copy[i], copy[j] = copy[i].random_alternative(), copy[j].random_alternative()
            yield copy

def score(nodes):
    score = 0
    for i in range(len(nodes)-1):
        score += edges[(nodes[i], nodes[i+1])]
    return score

def hill_climb(tour, move_operator, score, max_evaluations=1000):
    best = tour
    best_score = score(best)

    for num_evaluations in range(1, max_evaluations+1):
        move_made=False
        for next in move_operator(best):
            if num_evaluations >= max_evaluations:
                break
            
            # See if this is better
            next_score = score(next)
            num_evaluations += 1
            if next_score < best_score:
                best, best_score = next, next_score
                move_made = True
                break # Search depth first
            
        if not move_made:
            break # At local optimum
    
    return (num_evaluations, best_score, best)
            
possible = nodes[:]
shuffle(possible)
tour = []
for i in possible:
    unique = True
    for j in tour:
        if i.bin == j.bin:
            unique = False
            break
    if unique:
        tour.append(i)

iters, score, tour = hill_climb(tour, swapped_nodes, score)
print "Got a tour with score=%s in %s iterations:\n" % (score, iters)
for i, tour in enumerate(tour):
    print "%2d. %s %s from bin %s" % (i+1, tour.type, tour.item, tour.bin)
print
