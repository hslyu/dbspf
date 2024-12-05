import copy
import math
import random
import sys
import time
from os import path

import numpy as np
import pygad

sys.path.append(path.dirname(path.abspath(path.dirname(__file__))))

import drone_basestation as db  # noqa


def isAvailable(position, map_width, min_altitude, max_altitude):
    isAvailable = False
    # If the position is in the map, return true.
    isAvailable = (
        0 <= position[0] <= map_width
        and 0 <= position[1] <= map_width
        and min_altitude <= position[2] <= max_altitude
    )
    return isAvailable


def fitness_func_factory(
    init_path: list[db.TrajectoryNode],
    map_width: float,
    min_altitude: float,
    max_altitude: float,
    grid_size: float,
):
    """
    path: stores initial ua, ra, pc for every node.
    """

    def fitness_func(solution, solution_idx):
        nonlocal init_path

        gene_node = init_path[0]
        fitness = gene_node.reward
        for i, action in enumerate(solution):
            position = gene_node.position
            next_position = position.copy()
            if action == 0:
                pass
            elif action == 1:
                next_position[0] += grid_size
            elif action == 2:
                next_position[0] -= grid_size
            elif action == 3:
                next_position[1] += grid_size
            elif action == 4:
                next_position[1] -= grid_size
            elif action == 5:
                next_position[2] += grid_size
            elif action == 6:
                next_position[2] -= grid_size
            position = (
                next_position
                if isAvailable(next_position, map_width, min_altitude, max_altitude)
                else position
            )

            ra_list = [user.ra for user in init_path[i].user_list]
            psd_list = [user.psd for user in init_path[i].user_list]
            gene_node = db.TrajectoryNode(
                position, parent=gene_node, ra_list=ra_list, psd_list=psd_list
            )

            fitness += gene_node.reward

        return fitness

    return fitness_func


def genetic_tp(
    init_path,
    grid_size,
    map_width,
    min_altitude,
    max_altitude,
    verbose=False,
):
    fitness_func = fitness_func_factory(
        init_path, map_width, min_altitude, max_altitude, grid_size
    )

    sol_per_pop = 50
    num_generations = 10000
    num_parents_mating = 10
    num_genes = 20

    gene_space = {"low": 0, "high": 6}

    global last_fitness
    last_fitness = 0

    callback_func = callback_generation if verbose else None
    ga_instance = pygad.GA(
        num_generations=num_generations,
        num_parents_mating=num_parents_mating,
        fitness_func=fitness_func,
        sol_per_pop=sol_per_pop,
        num_genes=num_genes,
        on_generation=callback_func,
        gene_type=int,
        gene_space=gene_space,
        stop_criteria="saturate_50",
        mutation_probability=0.05,
    )

    ga_instance.run()
    solution, solution_fitness, solution_idx = ga_instance.best_solution()

    return solution, solution_fitness, solution_idx


def callback_generation(ga_instance):
    Generation = ga_instance.generations_completed
    if Generation % 100 == 0:
        global last_fitness
        Fitness = ga_instance.best_solution()[1]
        Change = ga_instance.best_solution()[1] - last_fitness
        print(f"{Generation = :02d}, {Fitness = :.3f}, {Change = :.5f}")
        last_fitness = ga_instance.best_solution()[1]
    else:
        pass


def get_path(
    root,
    grid_size,
    map_width,
    min_altitude,
    max_altitude,
    max_timeslot,
    verbose=False,
    num_iter=10,
):
    node = root
    path = [root]
    init_path = []
    for _ in range(max_timeslot):
        next_position = node.position.copy()
        action = random.randint(0, 6)
        if action == 0:
            pass
        elif action == 1:
            next_position[0] += grid_size
        elif action == 2:
            next_position[0] -= grid_size
        elif action == 3:
            next_position[1] += grid_size
        elif action == 4:
            next_position[1] -= grid_size
        elif action == 5:
            next_position[2] += grid_size
        elif action == 6:
            next_position[2] -= grid_size
        position = (
            next_position
            if isAvailable(next_position, map_width, min_altitude, max_altitude)
            else node.position.copy()
        )
        position = node.position.copy()
        node = db.TrajectoryNode(position, parent=node)
        path.append(node)

    prev_fitness = -2
    fitness = -1
    for i in range(num_iter):
        if fitness < prev_fitness:
            break
        prev_fitness = fitness

        solution, fitness, _ = genetic_tp(
            path,
            grid_size,
            map_width,
            min_altitude,
            max_altitude,
            verbose,
        )

        node = path[0]
        for j, action in enumerate(solution):
            position = node.position
            next_position = position.copy()
            if action == 0:
                pass
            elif action == 1:
                next_position[0] += grid_size
            elif action == 2:
                next_position[0] -= grid_size
            elif action == 3:
                next_position[1] += grid_size
            elif action == 4:
                next_position[1] -= grid_size
            elif action == 5:
                next_position[2] += grid_size
            elif action == 6:
                next_position[2] -= grid_size
            position = (
                next_position
                if isAvailable(next_position, map_width, min_altitude, max_altitude)
                else position
            )

            node = db.TrajectoryNode(position, num_iter=0, parent=node)
            path[j + 1] = node
        if i == 0:
            init_path = copy.deepcopy(path)

    return path, init_path


def main():
    vehicle_velocity = 15
    map_width = 3000
    grid_size = map_width // 10
    time_step = grid_size // vehicle_velocity
    min_altitude = 50
    max_altitude = 200
    max_timeslot = 20
    num_ue = 20
    initial_data = 10
    time_window_size = [4, 8]
    time_period_size = [20, 20]
    datarate_window = [10, 10]

    num_exp = 20
    avg_pf = 0
    avg_pf_dfs = 0
    for i in range(num_exp):
        random.seed(i)

        position = [
            random.randint(0, map_width) // 10 * 10,
            random.randint(0, map_width) // 10 * 10,
            random.randint(min_altitude, max_altitude) // 10 * 10,
        ]

        user_list = []
        for i in range(num_ue):
            tw_size = random.randint(time_window_size[0], time_window_size[1])
            time_period = random.randint(time_period_size[0], time_period_size[1])
            datarate = random.randint(datarate_window[0], datarate_window[1])
            user = db.User(
                i,  # id
                [
                    random.randint(0, map_width) // 10 * 10,
                    random.randint(0, map_width) // 10 * 10,
                ],  # position
                random.randint(0, max_timeslot - tw_size),  # time_start
                tw_size,
                time_period,  # time window
                datarate,
                initial_data,
                max_data=99999,
            )  # data
            user_list.append(user)

        node = db.TrajectoryNode(position)
        node.user_list = copy.deepcopy(user_list)
        tree = db.TrajectoryTree(
            node,
            vehicle_velocity,
            time_step,
            grid_size,
            map_width,
            min_altitude,
            max_altitude,
            tree_depth=3,
            num_node_iter=0,
            max_timeslot=max_timeslot,
        )
        dfs_path = tree.pathfinder()
        dfs_pf_list = [
            math.log(user.total_data - initial_data)
            for user in dfs_path[-1].user_list
            if user.total_data > initial_data
        ]
        pf_dfs = sum(dfs_pf_list)

        node = db.TrajectoryNode(position)
        node.user_list = copy.deepcopy(user_list)
        node.get_reward()

        start = time.time()
        path, init_path = get_path(
            node,
            grid_size,
            map_width,
            min_altitude,
            max_altitude,
            max_timeslot,
            True,
            num_iter=10,
        )
        user_list = path[-1].user_list
        pf = sum(
            math.log(user.total_data - 10) for user in user_list if user.total_data > 10
        )

        avg_pf += pf
        avg_pf_dfs += pf_dfs
        # for a, b in zip(path, init_path):
        #     print(f"{a.position = }, {b.position = }")
        print(f"GA PF: {pf}, DFS PF: {pf_dfs}")
        # for ga_node, dfs_node in zip(path, dfs_path):
        #     print(f"{ga_node.position = }, {dfs_node.position = }")
        # print(f"Elapsed time: {time.time() - start:.3f} s")
    print(f"Averaged GA PF: {avg_pf / num_exp}, DFS PF: {avg_pf_dfs / num_exp}")


if __name__ == "__main__":
    main()
