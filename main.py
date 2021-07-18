#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import argparse
import os
from drone_basestation import *
from environment_manager import *

def get_parser():
    parser = argparse.ArgumentParser(description='Simulate drone base station with specific depth',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--tree-depth', type=int, help='Tree depth')
    parser.add_argument('--env-path', default='/root/mnt/dbspf/env', type=str, help='Path of the environment directory')
    return parser

#def save_result(PATH):

if __name__ =="__main__":
    parser = get_parser()
    args = parser.parse_args().tree_depth
    if not bool(args.tree_depth):
        parser.error("Tree depth must be specified. Usage: {} --tree-depth 3".format(__file__))

    # Load environment
    env_args = load_args(args.env_path)
    for env_index in range(args.num_iteration):
        root = load_root(args.env_path, env_args, env_index)
        tree = TrajectoryTree(root, env_args.vehicle_velocity,\
                                env_args.time_step, env_args.grid_size,\
                                env_args.map_width, env_args.min_altitude, env_args.max_altitude,\
                                args.tree_depth, env_args.max_time)
        PATH = tree.pathfinder()
        reward = 0
        time = 0
        for node in PATH:
           reward += node.reward
           time += node.elapsed_time
        print(PATH)
        print(reward)
        print(time)
