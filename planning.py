import math
import random

import numpy as np

from config import DEFAULT_ABC_CONFIG, DEFAULT_MSCGA_CONFIG


def point_distance(p1, p2):
    return math.sqrt(
        (p1.X() - p2.X()) ** 2 +
        (p1.Y() - p2.Y()) ** 2 +
        (p1.Z() - p2.Z()) ** 2
    )


def build_distance_matrix(points):
    n = len(points)
    dist_matrix = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            distance = point_distance(points[i], points[j])
            dist_matrix[i, j] = distance
            dist_matrix[j, i] = distance
    return dist_matrix


def calculate_path_length(path, dist_matrix):
    total_length = 0.0
    for i in range(len(path) - 1):
        total_length += dist_matrix[path[i]][path[i + 1]]
    return total_length


def solve_sequential_open_path(points):
    return list(range(len(points))), build_distance_matrix(points)


def solve_greedy_open_path(points):
    n = len(points)
    if n == 0:
        return [], np.zeros((0, 0))

    dist_matrix = build_distance_matrix(points)
    visited = [False] * n
    path = [0]
    visited[0] = True
    current = 0

    for _ in range(n - 1):
        next_point = -1
        min_dist = float("inf")
        for i in range(n):
            if not visited[i] and dist_matrix[current][i] < min_dist:
                min_dist = dist_matrix[current][i]
                next_point = i
        if next_point == -1:
            break
        path.append(next_point)
        visited[next_point] = True
        current = next_point

    return path, dist_matrix


def run_abc_solver(points, config=None):
    import ABC

    cfg = config or DEFAULT_ABC_CONFIG
    conf = ABC.TSPConfig(
        points=points,
        FOOD_NUMBER=cfg.food_number,
        LIMIT=cfg.limit,
        MAXIMUM_EVALUATION=cfg.maximum_evaluation,
        SHOW_PROGRESS=cfg.show_progress,
        RANDOM_SEED=cfg.random_seed,
        SEED=cfg.seed,
        CLOSED_TOUR=cfg.closed_tour,
        angle_threshold_deg=cfg.angle_threshold_deg,
        weights=cfg.weights,
        VISUALIZE=cfg.visualize,
        VIS_SAVE_DIR=cfg.vis_save_dir,
        VIS_INTERVAL_EVALS=cfg.vis_interval_evals,
        PLOT_HISTORY=cfg.plot_history,
    )

    abc = ABC.ABC_TSP3D(conf)
    return abc.run()


# ---------------------------------------------------------------------------
# MSCGA — Multi-Strategy Combined Genetic Algorithm
# ---------------------------------------------------------------------------

def _mscga_create_distance_matrix(coords):
    """Euclidean distance matrix from a numpy (N, 3) array."""
    n = len(coords)
    diff = coords[:, np.newaxis, :] - coords[np.newaxis, :, :]
    return np.sqrt(np.sum(diff ** 2, axis=2))


def _mscga_path_total(path, dist_matrix, closed=False):
    total = sum(dist_matrix[path[i], path[i + 1]] for i in range(len(path) - 1))
    if closed:
        total += dist_matrix[path[-1], path[0]]
    return total


def _mscga_fitness(path, dist_matrix, closed=False):
    d = _mscga_path_total(path, dist_matrix, closed)
    return 1.0 / d if d > 0 else 0.0


def _mscga_generate_population(pop_size, num_cities):
    base = list(range(num_cities))
    return [random.sample(base, num_cities) for _ in range(pop_size)]


# -- Crossover operators ----------------------------------------------------

def _mscga_fill_child(child, parent, start, end):
    size = len(child)
    current_pos = (end + 1) % size
    fill_pos = (end + 1) % size
    for _ in range(size):
        city = parent[current_pos]
        if city not in child:
            child[fill_pos] = city
            fill_pos = (fill_pos + 1) % size
        current_pos = (current_pos + 1) % size


def _mscga_ordered_crossover(p1, p2):
    size = len(p1)
    c1, c2 = [-1] * size, [-1] * size
    s, e = sorted(random.sample(range(size), 2))
    c1[s:e + 1] = p1[s:e + 1]
    c2[s:e + 1] = p2[s:e + 1]
    _mscga_fill_child(c1, p2, s, e)
    _mscga_fill_child(c2, p1, s, e)
    return c1, c2


def _mscga_pmx_crossover(p1, p2):
    size = len(p1)
    c1, c2 = [-1] * size, [-1] * size
    s, e = sorted(random.sample(range(size), 2))
    c1[s:e + 1] = p1[s:e + 1]
    c2[s:e + 1] = p2[s:e + 1]
    m1 = {p1[i]: p2[i] for i in range(s, e + 1)}
    m2 = {p2[i]: p1[i] for i in range(s, e + 1)}
    for i in range(size):
        if c1[i] == -1:
            g = p2[i]
            while g in c1:
                g = m1.get(g, g)
            c1[i] = g
        if c2[i] == -1:
            g = p1[i]
            while g in c2:
                g = m2.get(g, g)
            c2[i] = g
    return c1, c2


def _mscga_cycle_crossover(p1, p2):
    size = len(p1)
    c1, c2 = [-1] * size, [-1] * size
    visited = [False] * size
    cycles = []
    for i in range(size):
        if not visited[i]:
            cycle = []
            cur = i
            while not visited[cur]:
                visited[cur] = True
                cycle.append(cur)
                cur = p2.index(p1[cur])
            cycles.append(cycle)
    for idx, cycle in enumerate(cycles):
        if idx % 2 == 0:
            for j in cycle:
                c1[j] = p1[j]
                c2[j] = p2[j]
        else:
            for j in cycle:
                c1[j] = p2[j]
                c2[j] = p1[j]
    return c1, c2


# -- Mutation operators -----------------------------------------------------

def _mscga_scmutation(ind, rate):
    if random.random() < rate:
        s, e = sorted(random.sample(range(len(ind)), 2))
        seg = ind[s:e + 1]
        random.shuffle(seg)
        ind[s:e + 1] = seg
    return ind


def _mscga_swap_mutation(ind, rate):
    if random.random() < rate:
        i, j = random.sample(range(len(ind)), 2)
        ind[i], ind[j] = ind[j], ind[i]
    return ind


def _mscga_inversion_mutation(ind, rate):
    if random.random() < rate:
        s, e = sorted(random.sample(range(len(ind)), 2))
        ind[s:e + 1] = ind[s:e + 1][::-1]
    return ind


def _mscga_insertion_mutation(ind, rate):
    if random.random() < rate and len(ind) > 1:
        pos = random.randint(0, len(ind) - 1)
        city = ind.pop(pos)
        new_pos = random.randint(0, len(ind) - 1)
        ind.insert(new_pos, city)
    return ind


def _mscga_adaptive_mutation(ind, rate, gen, max_gen):
    adaptive_rate = rate * (1 - gen / max_gen)
    return _mscga_swap_mutation(ind, adaptive_rate)


# -- 2-opt local search ----------------------------------------------------

def _mscga_two_opt(path, dist_matrix):
    n = len(path)
    best = path[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 2, n):
                old_d = dist_matrix[best[i - 1], best[i]] + dist_matrix[best[j - 1], best[j]]
                new_d = dist_matrix[best[i - 1], best[j - 1]] + dist_matrix[best[i], best[j]]
                if new_d < old_d:
                    best[i:j] = best[i:j][::-1]
                    improved = True
                    break
            if improved:
                break
    return best


# -- Main entry point -------------------------------------------------------

def run_mscga_solver(points, config=None, closed_tour=False):
    """Run the MSCGA solver on a list of gp_Pnt points.

    Parameters
    ----------
    points : list of gp_Pnt
    config : MSCGAConfig or None
    closed_tour : bool
        If True, optimises a closed loop (TSP); otherwise an open path.

    Returns
    -------
    dict with keys ``best_tour`` (list of int), ``best_length`` (float),
    and ``history`` (dict of per-generation stats).
    """
    cfg = config or DEFAULT_MSCGA_CONFIG

    coords = np.array([[p.X(), p.Y(), p.Z()] for p in points])
    dist_matrix = _mscga_create_distance_matrix(coords)
    n = len(coords)

    pop_size = cfg.pop_size
    generations = cfg.generations
    crossover_rate = cfg.crossover_rate
    mutation_rate = cfg.mutation_rate

    population = _mscga_generate_population(pop_size, n)

    best_path = None
    best_distance = float("inf")
    no_improve = 0
    max_no_improve = 25
    restart_count = 0
    max_restarts = 5

    gen_history = []
    cur_dist_history = []
    best_dist_history = []

    for gen in range(generations):
        fitness_list = [_mscga_fitness(p, dist_matrix, closed_tour) for p in population]
        distances = [_mscga_path_total(p, dist_matrix, closed_tour) for p in population]

        cur_best_idx = distances.index(min(distances))
        cur_distance = distances[cur_best_idx]
        if cur_distance < best_distance:
            best_distance = cur_distance
            best_path = population[cur_best_idx][:]
            no_improve = 0
            if cfg.show_progress:
                print("MSCGA gen {}: best = {:.2f}".format(gen, best_distance))
        else:
            no_improve += 1

        new_pop = [best_path[:]]

        for _ in range(pop_size - 1):
            # Tournament selection
            t_idx = random.sample(range(len(population)), 3)
            t_fit = [fitness_list[i] for i in t_idx]
            parent = population[t_idx[t_fit.index(max(t_fit))]]

            if random.random() < crossover_rate:
                other = population[random.randint(0, pop_size - 1)]
                phase = gen / generations
                if phase < 1.0 / 3:
                    child, _ = _mscga_pmx_crossover(parent, other)
                elif phase < 2.0 / 3:
                    child, _ = _mscga_ordered_crossover(parent, other)
                else:
                    child, _ = _mscga_cycle_crossover(parent, other)
            else:
                child = parent[:]

            # Mutation + local search (phase-dependent)
            phase = gen / generations
            if phase < 1.0 / 3:
                child = _mscga_scmutation(child, mutation_rate * 2.0)
                child = _mscga_inversion_mutation(child, mutation_rate * 1.8)
                if random.random() < cfg.two_opt_early:
                    child = _mscga_two_opt(child, dist_matrix)
            elif phase < 2.0 / 3:
                child = _mscga_inversion_mutation(child, mutation_rate)
                child = _mscga_swap_mutation(child, mutation_rate)
                child = _mscga_insertion_mutation(child, mutation_rate * 0.8)
                if random.random() < cfg.two_opt_mid:
                    child = _mscga_two_opt(child, dist_matrix)
            else:
                child = _mscga_adaptive_mutation(child, mutation_rate * 0.5, gen, generations)
                child = _mscga_insertion_mutation(child, mutation_rate * 0.3)
                if random.random() < cfg.two_opt_late:
                    child = _mscga_two_opt(child, dist_matrix)

            new_pop.append(child)

        population = new_pop

        # Population restart when stuck
        if no_improve > max_no_improve and restart_count < max_restarts:
            restart_count += 1
            elite_size = max(pop_size // 2, 10)
            elite = sorted(population, key=lambda x: _mscga_path_total(x, dist_matrix, closed_tour))[:elite_size]
            population = elite + _mscga_generate_population(pop_size - elite_size, n)
            no_improve = 0
            mutation_rate *= 1.2
            if cfg.show_progress:
                print("MSCGA restart #{}, mutation_rate={:.3f}".format(restart_count, mutation_rate))

        gen_history.append(gen)
        cur_dist_history.append(cur_distance)
        best_dist_history.append(best_distance)

        if no_improve > max_no_improve * 2:
            if cfg.show_progress:
                print("MSCGA early stop at gen {}".format(gen))
            break

    # Final 2-opt polish
    best_path = _mscga_two_opt(best_path, dist_matrix)
    best_distance = _mscga_path_total(best_path, dist_matrix, closed_tour)

    return {
        "best_tour": np.array(best_path),
        "best_length": best_distance,
        "history": {
            "generations": gen_history,
            "current_distance": cur_dist_history,
            "best_distance": best_dist_history,
        },
    }
