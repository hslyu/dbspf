#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import argparse
from drone_basestation import *

# Tree constant example{{{
# Number of iteration
NUM_EPOCH=1000
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
# Tree depth
TREE_DEPTH = 1#}}}

def get_parser():#{{{
    parser = argparse.ArgumentParser(description="Drone base station trajectory planner with user data request time under proportional fairness scheme",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--vehicle-velocity", default=VEHICLE_VELOCITY, type=float, help="Drone maximum velocity")
    parser.add_argument("--time-step", default=TIME_STEP, type=int, help="Time unit for trajectory planning")
    parser.add_argument("--max-time", default=MAX_TIME, type=int, help="Total time of trajectory planning")
    parser.add_argument("--map_width", default=MAP_WIDTH, type=int, help="Map width")
    parser.add_argument("--min-altitude", default=MIN_ALTITUDE, type=int, help="Minimum altitude")
    parser.add_argument("--max-altitude", default=MAX_ALTITUDE, type=int, help="Maximum altitude")
    parser.add_argument("--grid-size", default=GRID_SIZE, type=float, help="Unit length of descritized map")
    parser.add_argument("--num-ue", default=NUM_UE, type=int, help="Number of user")
    parser.add_argument("--time-window-size", default=TIME_WINDOW_SIZE, nargs='+', help="Time window size")
    parser.add_argument("--datarate-window", default=DATARATE_WINDOW, nargs='+', help="Datarate window")
    parser.add_argument("--initial-data", default=INITIAL_DATA, type=float, help="Initial data")
    parser.add_argument("--tree-depth", default=TREE_DEPTH, type=int, help="Tree depth")
    parser.add_argument("--num-epoch", default=NUM_EPOCH, type=int, help="Tree depth")
    return parser#}}}

if __name__ =="__main__":
    parser = get_parser()
    args = parser.parse_args()

    position = np.array([random.randint(0, args.map_width)//args.grid_size*args.grid_size,
                         random.randint(0, args.map_width)//args.grid_size*args.grid_size,
                         random.randint(args.min_altitude, args.max_altitude)//args.grid_size*args.grid_size])
    # Initial grid position of uav
    root = TrajectoryNode(position)
    # Make user list
    user_list = []
    for i in range(args.num_ue):
        tw_size = random.randint(args.time_window_size[0], args.time_window_size[1])
        user = User(i, # id
                random.randint(0, args.map_width), random.randint(0, args.map_width), # map
                random.randint(0, args.max_time-tw_size), tw_size, # time window
                random.randint(args.datarate_window[0], args.datarate_window[1]), # data
                args.initial_data, 999999) # data
        user_list.append(user)
    root.user_list = user_list

    tree = TrajectoryTree(root, args.vehicle_velocity,\
                            args.time_step, args.grid_size,\
                            args.map_width, args.min_altitude, args.max_altitude,\
                            args.tree_depth, args.max_time)

    PATH = tree.pathfinder()
    reward = 0
    time = 0
    for node in PATH:
       reward += node.reward
       time += node.elapsed_time
    print(PATH)
    print(reward)
    print(time)
