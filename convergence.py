#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import argparse
import random
import math
import time
import numpy as np
from drone_basestation import User, TrajectoryNode
import copy

# Constant for UAV
VEHICLE_VELOCITY = 15. # m/s
TIME_STEP = 3 # s
MAX_TIMESLOT = 20 # unit of (TIME_STEP) s
## Constant for map
MAP_WIDTH = 600 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 200 # meter
GRID_SIZE = 45 # meter
# Constant for user
NUM_UE = 40
NUM_NODE_ITER = 0
TIME_WINDOW_SIZE = [1, 10]
TIME_PERIOD_SIZE = [MAX_TIMESLOT, MAX_TIMESLOT]
DATARATE_WINDOW = [10, 10] # Requiring datarate Mb/s
INITIAL_DATA = 10 # Mb
TREE_DEPTH = 1
MAX_DATA = 99999999

def get_parser():
    parser = argparse.ArgumentParser(description='Convergence test',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-n', '--num_node_iter', type=int, default=0, help='Number of node iteration.')
    return parser

parser = get_parser()
args = parser.parse_args()

position = [random.randint(0, MAP_WIDTH)//10*10,
            random.randint(0, MAP_WIDTH)//10*10,
            random.randint(MIN_ALTITUDE, MAX_ALTITUDE)//10*10]

end=30
step = 1
reward_list = np.zeros(end//step)
num_env = 1000
for env_idx in range(num_env):
    user_list = []
    for i in range(NUM_UE):
        tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
        time_period = random.randint(TIME_PERIOD_SIZE[0], TIME_PERIOD_SIZE[1])
        datarate = random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1])
        user = User(i, # id
                [random.randint(0, MAP_WIDTH), random.randint(0, MAP_WIDTH)], # position
                random.randint(0, time_period-tw_size), tw_size, time_period, # time window
                datarate, INITIAL_DATA*random.random()*10, MAX_DATA) # data
        user_list.append(user)

    root = TrajectoryNode(position, NUM_NODE_ITER, None, None)

    idx = 0
#    tmp = []
    for niter in range(0,end,step):
        root.user_list = copy.deepcopy(user_list)
        root.get_reward(niter)
        reward_list[idx] += root.get_reward(niter)
#        tmp.append(reward_list[idx])
        idx+=1
#    print(tmp)

    print(f'Current step: {env_idx} with reward {(reward_list/(env_idx+1))/max(reward_list/(env_idx+1))}')
