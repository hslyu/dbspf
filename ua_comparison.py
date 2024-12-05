#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import argparse
import copy
import random
import time

from drone_basestation import TrajectoryNode, User
from genetic_algorithm import rrm_ga_optimizer as ga_rrm

# Constant for UAV
VEHICLE_VELOCITY = 15.0  # m/s
TIME_STEP = 3  # s
MAX_TIMESLOT = 20  # unit of (TIME_STEP) s
## Constant for map
MAP_WIDTH = 600  # meter, Both X and Y axis width
MIN_ALTITUDE = 50  # meter
MAX_ALTITUDE = 200  # meter
GRID_SIZE = 45  # meter
# Constant for user
NUM_UE = 20
TIME_WINDOW_SIZE = [4, 8]
TIME_PERIOD_SIZE = [MAX_TIMESLOT, MAX_TIMESLOT]
DATARATE_WINDOW = [5, 5]  # Requiring datarate Mb/s
INITIAL_DATA = 10  # Mb
TREE_DEPTH = 1
MAX_DATA = 99999999


def get_parser():
    parser = argparse.ArgumentParser(
        description="Convergence test",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--index_start", default=None, type=int, help="Iteration start index"
    )
    parser.add_argument(
        "--index_end", default=None, type=int, help="Iteration end index"
    )
    return parser


parser = get_parser()
args = parser.parse_args()

position = [
    random.randint(0, MAP_WIDTH) // 10 * 10,
    random.randint(0, MAP_WIDTH) // 10 * 10,
    random.randint(MIN_ALTITUDE, MAX_ALTITUDE) // 10 * 10,
]

step = 1
reward_ga = 0
reward = 0
reward_max_SINR = 0
avg_time = 0

if args.index_start is None:
    num_env = 10
    iter_range = range(num_env)
else:
    num_env = args.index_end - args.index_start
    iter_range = range(args.index_start, args.index_end)

for env_idx in iter_range:
    random.seed(env_idx)
    user_list = []
    while True:
        for i in range(NUM_UE):
            tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
            time_period = random.randint(TIME_PERIOD_SIZE[0], TIME_PERIOD_SIZE[1])
            datarate = random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1])
            user = User(
                i,  # id
                [
                    random.randint(0, MAP_WIDTH),
                    random.randint(0, MAP_WIDTH),
                ],  # position
                random.randint(0, time_period - tw_size),
                tw_size,
                time_period,  # time window
                datarate,
                INITIAL_DATA + random.randint(0, 20),
                MAX_DATA,
            )  # data
            user_list.append(user)

        root = TrajectoryNode(position)
        root.current_time = 10

        root.user_list = copy.deepcopy(user_list)
        if len(root.get_valid_user()) == 0:
            user_list = []
        else:
            break

    # eps_reward_ga = ga_rrm.get_rrm_reward(root)
    # reward_ga += eps_reward_ga

    start = time.time()
    root.user_list = copy.deepcopy(user_list)
    # eps_reward_ga = ga_rrm.get_rrm_reward(root)
    # reward_ga += eps_reward_ga
    eps_reward = root.get_reward()
    reward += eps_reward
    eps_time = time.time() - start
    avg_time += eps_time
    print(f"Elapsed time: {eps_time:.5f}")

    # root.user_list = copy.deepcopy(user_list)
    # eps_reward_max_SINR = root.get_reward(init_ua_ra_mode="max_SINR")
    # reward_max_SINR += eps_reward_max_SINR

    # print(
    #     f"env_idx: {env_idx}, reward_ga: {eps_reward_ga:.5f}, reward: {eps_reward:.5f}, reward_max_SINR: {eps_reward_max_SINR:.5f}"
    # )

print(f"Average time: {avg_time / num_env}")
# print(f"Average reward_ga: {reward_ga / num_env}")
# print(f"Average reward: {reward / num_env}")
# print(f"Average reward: {reward_max_SINR / num_env}")
