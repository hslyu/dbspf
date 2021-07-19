#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import random
import json
import os
from drone_basestation import User, TrajectoryNode

def create_dir(directory):
    try:#{{{
        if not os.path.exists(directory):
            os.makedirs(directory)
#        else:
#            print('Directory already exists: ' + directory)
    except OSError:
        print('Fail to create diretory: ' +  directory)#}}}

def environment_generator(parser):
    args = parser.parse_args()#{{{
    create_dir(args.directory)

    with open(os.path.join(args.directory,'args.json'), 'w') as f:
        json.dump(args.__dict__, f, ensure_ascii=False, indent=4)

    for i in range(args.num_iteration):
        env_dict = {}
        env_dict['num_iteration']=i
        random.seed(i)
        # Initial position of UAV
        env_dict['root_position'] = [random.randint(0, args.map_width)//args.grid_size*args.grid_size,
                             random.randint(0, args.map_width)//args.grid_size*args.grid_size,
                             random.randint(args.min_altitude, args.max_altitude)//args.grid_size*args.grid_size]
        # Make user list
        user_list = []
        for j in range(args.num_ue):
            user_data = {}
            user_data['id'] = j
            tw_size = random.randint(args.time_window_size[0], args.time_window_size[1])
            user_data['position'] = [random.randint(0, args.map_width), random.randint(0, args.map_width)]
            user_data['time_start'] = random.randint(0, args.max_time-tw_size)
            user_data['tw_size'] = tw_size
            user_data['datarate'] = random.randint(args.datarate_window[0], args.datarate_window[1])
            user_data['total_data'] = args.initial_data
            user_data['max_data'] = 999999
            user_list.append(user_data)
        env_dict['user_list'] = user_list

        create_dir(os.path.join(args.directory, 'env'))
        with open(os.path.join(args.directory,'env/env_{}.json'.format(i)), 'w') as f:
            json.dump(env_dict, f, ensure_ascii=False, indent=4)#}}}

def load_args(path):
    with open(os.path.join(path, 'args.json')) as f:#{{{
        args = json.load(f)
        args = type('Arguments', (object,), args)
    return args#}}}

def load_root(path, args, env_index):
    with open(os.path.join(path,'env/env_{}.json'.format(env_index))) as f:#{{{
        env = json.load(f)
        if env['num_iteration'] != env_index:
            print("FATAL ERROR (load_root) : iteration index is not matched")
            exit()
#        elif len(user_dict_list) != args.num_ue:
#            print("FATAL ERROR (load_root) : number of user is not matched")
#            exit()

        root = TrajectoryNode(env['root_position'])
        
        user_list = []
        user_dict_list = env['user_list']
        for user_dict in user_dict_list:
            user = User(*user_dict.values())
            user_list.append(user)
        root.user_list = user_list

    return root#}}}

if __name__ =='__main__':
    import argparse
    # Tree constant example
    DIRECTORY_PATH = os.path.join(os.getcwd(),'data')
    # Number of iteration
    NUM_ITERATION=1000
    # Constant for UAV
    VEHICLE_VELOCITY = 10. # m/s
    TIME_STEP = 1 # s
    MAX_TIME = 200 # unit of (TIME_STEP) s
    ## Constant for map
    MAP_WIDTH = 200 # meter, Both X and Y axis width
    MIN_ALTITUDE = 50 # meter
    MAX_ALTITUDE = 100 # meter
    GRID_SIZE = 10 # meter
    # Constant for user
    NUM_UE = 80
    TIME_WINDOW_SIZE = [3,7]
    DATARATE_WINDOW = [35, 60] # Requiring datarate Mb/s
    INITIAL_DATA = 10 # Mb

    def get_parser():
        parser = argparse.ArgumentParser(description='Generate consistent random position',
                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        parser.add_argument('--directory', default=DIRECTORY_PATH, type=str, help='Directory to save the environment')
        parser.add_argument('--num_iteration', default=NUM_ITERATION, type=int, help='Total number of iteration')
        parser.add_argument('--vehicle_velocity', default=VEHICLE_VELOCITY, type=float, help='Drone maximum velocity')
        parser.add_argument('--time_step', default=TIME_STEP, type=int, help='Time unit for trajectory planning')
        parser.add_argument('--max_time', default=MAX_TIME, type=int, help='Total time of trajectory planning')
        parser.add_argument('--map_width', default=MAP_WIDTH, type=int, help='Map width')
        parser.add_argument('--min_altitude', default=MIN_ALTITUDE, type=int, help='Minimum altitude')
        parser.add_argument('--max_altitude', default=MAX_ALTITUDE, type=int, help='Maximum altitude')
        parser.add_argument('--grid_size', default=GRID_SIZE, type=float, help='Unit length of descritized map')
        parser.add_argument('--num_ue', default=NUM_UE, type=int, help='Number of user')
        parser.add_argument('--time_window_size', default=TIME_WINDOW_SIZE, nargs='+', help='Time window size')
        parser.add_argument('--datarate_window', default=DATARATE_WINDOW, nargs='+', help='Datarate window')
        parser.add_argument('--initial_data', default=INITIAL_DATA, type=float, help='Initial data')

        return parser

    # Generate environment
    parser = get_parser()
    environment_generator(parser)

#    # Load environment
#    args = load_args(DIRECTORY_PATH)
#    for i in range(args.num_iteration):
#        root = load_root(DIRECTORY_PATH, args, i)
