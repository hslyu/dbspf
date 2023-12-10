import math
import random
import sys
from os import path

import numpy as np
import pygad

sys.path.append(path.dirname(path.abspath(path.dirname(__file__))))

import drone_basestation as db  # noqa

# Constant for wirless communication
FREQUENCY = 2.0 * 1e9  # Hz
LIGHTSPEED = 3 * 1e8  # m/s
BANDWIDTH_ORIG = 2  # MHz
POWER_ORIG = 200  # mW
BANDWIDTH = 1.0  # <BANDWIDTH_ORIG> MHz per unit
POWER = 1.0  # 200 mW per unit
NOISE_DENSITY = -173.8  # noise spectral density(Johnson-Nyquist_noise)
LOS_EXCESSIVE = 1  # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 40  # dB, excessive pathloss of nlos link
# LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
# NLOS_EXCESSIVE = 20 # dB, excessive pathloss of nlos link
SURROUNDING_A = 9.64  # Envrionmental parameter for probablistic LOS link
SURROUNDING_B = 0.06  # Envrionmental parameter for probablistic LOS link
# SURROUNDING_A = 9.64 # Envrionmental parameter for probablistic LOS link
# SURROUNDING_B = 0.35 # Envrionmental parameter for probablistic LOS link
# Optimization hyperparameter
EPSILON = 1e-9
STEP_SIZE = 1e-3
THRESHOLD = 1e-8
# etc
INF = 1e8 - 1


def psd2snr(psd, pathloss):
    """
    Because unit of psd is 200mW/2MHz = 1e-4 mw/Hz, we should convert it to mw/Hz
    """

    if psd == 0:
        return 0
    else:
        return (
            10 * math.log10(psd * POWER_ORIG / (BANDWIDTH_ORIG * 1e6))
            - pathloss
            - NOISE_DENSITY
        )


def snr2se(snr):
    """
    Because unit of resource is <BANDWIDTH_ORIG> MHz,
    we should convert the unit of se from bps/Hz to Mbps/<BANDWIDTH_ORIG> MHz
    """
    return BANDWIDTH_ORIG * math.log2(1 + 10 ** (snr / 10))


def proportion_array(arr: np.ndarray):
    return arr / sum(arr)


def fitness_func_factory(node: db.TrajectoryNode):
    def fitness_func(solution, solution_idx):
        nonlocal node
        user_list = node.get_valid_user()
        ua_list = solution[: len(user_list)]
        ra_gene = solution[len(user_list) : len(user_list) * 2]
        ra_list = proportion_array(np.array(ra_gene))
        psd_list = np.array(solution[len(user_list) * 2 :])

        # Sum of power should be less than P.
        ra_pc_mul_list = [ra * pc for ra, pc in zip(ra_list, psd_list)]
        if sum(ra_pc_mul_list) > 1:
            fitness = -INF
            return fitness

        for i, ua in enumerate(ua_list):
            if ua == 0:
                continue
            user = user_list[i]
            user.ra = ra_list[i]
            user.snr = psd2snr(psd_list[i], user.pathloss)
            user.se = snr2se(user.snr)
            if user.ra * user.se < user.datarate:
                fitness = -INF
                return fitness

        fitness = 0
        for ua, user in zip(ua_list, user_list):
            fitness += ua * math.log(1 + user.ra * user.se / user.total_data)

        return fitness

    return fitness_func


def callback_generation(ga_instance):
    Generation = ga_instance.generations_completed
    if Generation % 100 == 0:
        global last_fitness
        Fitness = ga_instance.best_solution()[1]
        Change = ga_instance.best_solution()[1] - last_fitness
        print(f"{Generation = :02d}, {Fitness = :.3f}, {Change = :.3f}")
        last_fitness = ga_instance.best_solution()[1]
    else:
        pass


def main():
    map_width = 600
    min_altitude = 50
    max_altitude = 200
    max_timeslot = 20
    num_ue = 40
    initial_data = 10
    time_window_size = [8, 8]
    time_period_size = [20, 20]
    datarate_window = [1, 1]

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
    node.copy_user(user_list)
    node.current_time = 0

    user_list = node.get_valid_user()

    num_generations = 10000
    num_parents_mating = 10
    sol_per_pop = 200
    num_genes = len(user_list) * 3

    ua_bound = [0, 1]
    ra_bound = {"low": 0, "high": 2}
    pc_bound = {"low": 0.5, "high": 1.5}
    gene_space = []
    gene_type = []
    for _ in range(len(user_list)):
        gene_space.append(ua_bound)
        gene_type.append(int)
    for _ in range(len(user_list)):
        gene_space.append(ra_bound)
        gene_type.append(float)
    for _ in range(len(user_list)):
        gene_space.append(pc_bound)
        gene_type.append(float)

    global last_fitness
    last_fitness = 0

    ga_instance = pygad.GA(
        num_generations=num_generations,
        num_parents_mating=num_parents_mating,
        fitness_func=fitness_func_factory(node),
        sol_per_pop=sol_per_pop,
        num_genes=num_genes,
        gene_space=gene_space,
        gene_type=gene_type,
        mutation_probability=0.1,
        callback_generation=callback_generation,
        stop_criteria="saturate_500",
    )

    ga_instance.run()
    solution, solution_fitness, solution_idx = ga_instance.best_solution()
    print("Parameters of the best solution : {solution}".format(solution=solution))
    print(
        "Fitness value of the best solution = {solution_fitness}".format(
            solution_fitness=solution_fitness
        )
    )
    print(
        "Index of the best solution : {solution_idx}".format(solution_idx=solution_idx)
    )
    ua_list = solution[: len(user_list)]
    ra_list = solution[len(user_list) : len(user_list) * 2]
    ra_list = proportion_array(np.array(ra_list))
    psd_list = solution[len(user_list) * 2 :]
    print("----")
    print([user for user, ua in zip(user_list, ua_list) if ua == 1])
    print([ra for ra, ua in zip(ra_list, ua_list) if ua == 1])
    print([psd for psd, ua in zip(psd_list, ua_list) if ua == 1])
    print("----")
    print(node.get_reward())
    return


if __name__ == "__main__":
    main()
