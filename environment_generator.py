#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import random
import json
import os
from utils import create_dir
import argparse

# Tree constant example
DIRECTORY_PATH = '/home/hslyu/dbspf/data'
# Number of iteration
NUM_ITERATION=300
# Constant for UAV
VEHICLE_VELOCITY = 15. # m/s
TIME_STEP = 3 # s
MAX_TIMESLOT = 20 # unit of (TIME_STEP) s
## Constant for map
MAP_WIDTH = 600 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 200 # meter
GRID_SIZE = 40 # meter
# Constant for user
NUM_UE = 200
TIME_WINDOW_SIZE = [8,8] 
TIME_PERIOD_SIZE = [MAX_TIMESLOT, MAX_TIMESLOT]
#DATARATE_WINDOW = [0,0] # Requiring datarate Mb/s
INITIAL_DATA = 10 # Mb

def get_parser():
    parser = argparse.ArgumentParser(description='Generate consistent random position',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output_dir', default=DIRECTORY_PATH, type=str, help='Directory to save the environment')
    parser.add_argument('--num_iteration', default=NUM_ITERATION, type=int, help='Total number of iteration')
    parser.add_argument('--vehicle_velocity', default=VEHICLE_VELOCITY, type=float, help='Drone maximum velocity')
    parser.add_argument('--time_step', default=TIME_STEP, type=int, help='Time unit for trajectory planning')
    parser.add_argument('--max_timeslot', default=MAX_TIMESLOT, type=int, help='Total time of trajectory planning')
    parser.add_argument('--map_width', default=MAP_WIDTH, type=int, help='Map width')
    parser.add_argument('--min_altitude', default=MIN_ALTITUDE, type=int, help='Minimum altitude')
    parser.add_argument('--max_altitude', default=MAX_ALTITUDE, type=int, help='Maximum altitude')
    parser.add_argument('--grid_size', default=GRID_SIZE, type=float, help='Unit length of descritized map')
    parser.add_argument('--num_ue', default=NUM_UE, type=int, help='Number of user')
    parser.add_argument('--time_window_size', default=TIME_WINDOW_SIZE, type=int, nargs='+', help='Time window size')
    parser.add_argument('--time_period_size', default=TIME_PERIOD_SIZE, type=int, nargs='+', help='Time period size')
#    parser.add_argument('--datarate_window', default=DATARATE_WINDOW, type=int, nargs='+', help='Datarate window')
    parser.add_argument('--initial_data', default=INITIAL_DATA, type=float, help='Initial data')
    parser.add_argument('--args_filename', default='args.json', type=str, help='Name of argument json file')
    parser.add_argument('--generate_args_only', default=False, type=bool, help='Generate just arguments if true. Otherwise, the scripts generates a number of environments')

    return parser

def environment_generator(parser):
    args = parser.parse_args()
    create_dir(os.path.join(args.output_dir, 'env'))
    with open(os.path.join(args.output_dir, args.args_filename), 'w') as f:
        json.dump(args.__dict__, f, ensure_ascii=False, indent=4)
    if args.generate_args_only:
        return

    for i in range(args.num_iteration):
        env_dict = {}
        env_dict['num_iteration']=i
        random.seed(i)
        # Initial position of UAV
        env_dict['root_position'] = [random.randint(0, args.map_width)//args.grid_size*args.grid_size,
                             random.randint(0, args.map_width)//args.grid_size*args.grid_size,
                             200]
#                             random.randint(args.min_altitude, args.max_altitude)//args.grid_size*args.grid_size]
        # Make user list
        user_list = []
        for j in range(args.num_ue):
            user_data = {}
            user_data['id'] = j
            tw_size = random.randint(args.time_window_size[0], args.time_window_size[1])
            time_period = random.randint(args.time_period_size[0], args.time_period_size[1])
            user_data['position'] = [random.randint(0, args.map_width), random.randint(0, args.map_width)]
            user_data['time_start'] = random.randint(0, args.max_timeslot-tw_size)
            user_data['tw_size'] = tw_size
            user_data['time_period'] = time_period
#            user_data['datarate'] = random.randint(args.datarate_window[0], args.datarate_window[1])
            user_data['datarate'] = 0
            user_data['total_data'] = args.initial_data
#            user_data['max_data'] = random.randint(3,5)*user_data['datarate']
            user_data['max_data'] = 999999
            user_list.append(user_data)
        env_dict['user_list'] = user_list

        with open(os.path.join(args.output_dir, f'env/env_{i:04d}.json'), 'w') as f:
            json.dump(env_dict, f, ensure_ascii=False, indent=4)

if __name__ =='__main__':

    # Generate environment
    parser = get_parser()
    environment_generator(parser)
