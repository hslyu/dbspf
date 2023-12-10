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
    node: db.TrajectoryNode,
    map_width: float,
    min_altitude: float,
    max_altitude: float,
    grid_size: float,
):
    def fitness_func(solution, solution_idx):
        nonlocal node
        gene_node = db.TrajectoryNode(node.position, num_iter=0, parent=node)
        gene_node.current_time = 0
        gene_node.reward = node.reward
        fitness = node.reward
        for action in solution:
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

            gene_node = db.TrajectoryNode(position, num_iter=0, parent=gene_node)
            # user_list = gene_node.user_list
            # fitness = sum(
            #     math.log(user.total_data - 10) for user in user_list if user.total_data > 10
            # )

            fitness += gene_node.reward

        return fitness

    return fitness_func


def genetic_tp(
    root,
    vehicle_velocity,
    time_step,
    grid_size,
    map_width,
    min_altitude,
    max_altitude,
    max_timeslot,
    verbose=False,
):
    tree = db.TrajectoryTree(
        root,
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

    initial_gene = []
    for i in range(max_timeslot - 1):
        if dfs_path[i].position == dfs_path[i + 1].position:
            initial_gene.append(0)
        elif dfs_path[i].position[0] < dfs_path[i + 1].position[0]:
            initial_gene.append(1)
        elif dfs_path[i].position[0] > dfs_path[i + 1].position[0]:
            initial_gene.append(2)
        elif dfs_path[i].position[1] < dfs_path[i + 1].position[1]:
            initial_gene.append(3)
        elif dfs_path[i].position[1] > dfs_path[i + 1].position[1]:
            initial_gene.append(4)
        elif dfs_path[i].position[2] < dfs_path[i + 1].position[2]:
            initial_gene.append(5)
        elif dfs_path[i].position[2] > dfs_path[i + 1].position[2]:
            initial_gene.append(6)

    sol_per_pop = 200
    initial_population = np.random.randint(0, 6, size=(sol_per_pop, 19))
    initial_population[0] = initial_gene

    fitness_func = fitness_func_factory(
        root, map_width, min_altitude, max_altitude, grid_size
    )
    num_generations = 10000
    num_parents_mating = 10

    num_genes = 19

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
        initial_population=initial_population,
        stop_criteria="saturate_100",
        mutation_probability=0.2,
    )

    ga_instance.run()
    solution, solution_fitness, solution_idx = ga_instance.best_solution()

    return solution, solution_fitness, solution_idx


def callback_generation(ga_instance):
    Generation = ga_instance.generations_completed
    if Generation % 1 == 0:
        global last_fitness
        Fitness = ga_instance.best_solution()[1]
        Change = ga_instance.best_solution()[1] - last_fitness
        print(f"{Generation = :02d}, {Fitness = :.3f}, {Change = :.3f}")
        last_fitness = ga_instance.best_solution()[1]
    else:
        pass


def get_path(
    root,
    vehicle_velocity,
    time_step,
    grid_size,
    map_width,
    min_altitude,
    max_altitude,
    max_timeslot,
    bandwidth,
    verbose=None,
):
    db.BANDWIDTH_ORIG = bandwidth
    solution, _, _ = genetic_tp(
        root,
        vehicle_velocity,
        time_step,
        grid_size,
        map_width,
        min_altitude,
        max_altitude,
        max_timeslot,
        verbose,
    )

    path = [root]
    node = root
    for action in solution:
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

        next_node = db.TrajectoryNode(position, num_iter=0, parent=node)
        path.append(next_node)
        node = next_node
    return path


def main():
    db.BANDWIDTH_ORIG = 5
    vehicle_velocity = 15
    time_step = 3
    grid_size = 45
    map_width = 600
    min_altitude = 50
    max_altitude = 200
    max_timeslot = 20
    num_ue = 20
    initial_data = 10
    time_window_size = [4, 8]
    time_period_size = [20, 20]
    datarate_window = [0, 0]

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
    node.user_list = user_list

    start = time.time()
    # solution, solution_fitness, solution_idx = genetic_tp(
    #     node,
    #     vehicle_velocity,
    #     time_step,
    #     grid_size,
    #     map_width,
    #     min_altitude,
    #     max_altitude,
    #     max_timeslot,
    #     verbose=True,
    # )
    path = get_path(
        node,
        vehicle_velocity,
        time_step,
        grid_size,
        map_width,
        min_altitude,
        max_altitude,
        max_timeslot,
        10,
        True,
    )
    user_list = path[-1].user_list
    pf = sum(
        math.log(user.total_data - 10) for user in user_list if user.total_data > 10
    )
    print(f"PF: {pf}")
    print(f"Elapsed time: {time.time() - start:.3f} s")
    # print("Parameters of the best solution : {solution}".format(solution=solution))
    # print(
    #     "Fitness value of the best solution = {solution_fitness}".format(
    #         solution_fitness=solution_fitness
    #     )
    # )
    # print(
    #     "Index of the best solution : {solution_idx}".format(solution_idx=solution_idx)
    # )


if __name__ == "__main__":
    main()
