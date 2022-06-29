#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station iterative simulation code.
# If you want to simulate just one environment, execute drone_basestation.py.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import argparse
import os
import json
import drone_basestation as dbs
import math
from drone_basestation import User, TrajectoryNode
from utils import create_dir, open_json

def get_parser():
    parser = argparse.ArgumentParser(description='Simulate drone base station with specific depth',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-t', '--tree_depth', type=int, help='Tree depth')
    parser.add_argument('-n', '--num_node_iter', type=int, default=0, help='Number of node iteration.')
    parser.add_argument('--env_path', default=os.path.join(os.getcwd(), 'data'), type=str, help='Path of the environment directory')
    parser.add_argument('--env_args_filename', default='args.json', type=str, help='Filename of the environment argument json file')
    parser.add_argument('--result_path', default=os.path.join(os.getcwd(), 'result'), type=str, help='Path of the result directory')
    parser.add_argument('--num_user', type=int, help='Path of the environment directory')
    parser.add_argument('--index_start', default=0, type=int, help='Iteration start index')
    parser.add_argument('--index_end', type=int, help='Iteration end index')
    parser.add_argument('--datarate', type=int, default=False, help='Option for datarate effect simulation')
    return parser

def load_root(path, num_user, env_index):
    with open(os.path.join(path,f'env/env_{env_index:04d}.json')) as f:
        env = json.load(f)
        if env['num_iteration'] != env_index:
            print("FATAL ERROR (load_root) : iteration index is not matched")
            exit()

        root = TrajectoryNode(env['root_position'])
        
        user_list = []
        user_dict_list = env['user_list']
        for user_dict in user_dict_list[0:num_user]:
            user = User(*user_dict.values())
            user_list.append(user)
        root.user_list = user_list

    return root

def save_result(filename, result_dir, env_args, main_args, env_index, total_reward, total_time, trajectory):
    result = {}
    result['environment_name'] = os.path.join(f'../data/env/env_{env_index:04d}.json')
    result['env_args'] = env_args
    result['tree_depth'] = main_args.tree_depth
    result['num_user'] = main_args.num_user
    result['total_reward'] = total_reward
    result['total_time'] = total_time

    node_list = []
    for node in trajectory:
        node_dict = node.__dict__.copy()
        # Delete recursive unserializable obejct "TrajectoryNode"
        del node_dict['leafs']
        if node.parent is not None:
            node_dict['parent'] = node.parent.position
        node_dict['user_list'] = []
        for user in node.user_list:
            node_dict['user_list'].append(user.__dict__)
        node_list.append(node_dict)

    result['trajectory'] = node_list
    with open(os.path.join(result_dir, filename), 'w') as f:
        json.dump(result, f, ensure_ascii=False, indent=4)

if __name__ =="__main__":
    parser = get_parser()
    main_args = parser.parse_args()

    if not bool(main_args.tree_depth):
        parser.error("Tree depth must be specified. Usage: {} --tree_depth 3".format(__file__))
    if not bool(main_args.index_end):
        parser.error("End iteration index should be specified.")
    if not bool(main_args.num_user):
        parser.error("Number of user should be specified.")

    # Load environment
    env_args_dict = open_json(os.path.join(main_args.env_path, main_args.env_args_filename))

    env_args = type('Arguments', (object,), env_args_dict)
    # Create directory to store the result
    create_dir(main_args.result_path)
    
    avg_obj = 0
    avg_reward = 0
    # Load root node and start trajectory plannnig
    for env_index in range(main_args.index_start, main_args.index_end):
        root = load_root(main_args.env_path, main_args.num_user, env_index)
        for user in root.user_list:
            user.datarate = main_args.datarate
        tree = dbs.TrajectoryTree(root, env_args.vehicle_velocity,
                                env_args.time_step, env_args.grid_size,
                                env_args.map_width, env_args.min_altitude, env_args.max_altitude,
                                main_args.tree_depth, main_args.num_node_iter, env_args.max_timeslot)
        dbs_trajectory = tree.pathfinder()
        total_reward = 0
        total_time = 0
        user_list = dbs_trajectory[-1].user_list
        obj = sum([math.log2(user.total_data-env_args.initial_data) for user in user_list if user.total_data != env_args.initial_data])
        avg_obj += obj
        for node in dbs_trajectory:
            total_reward += node.reward
            total_time += node.elapsed_time
        save_result(f'env_{env_index:04d}-depth_{main_args.tree_depth}-ue_{main_args.num_user}.json', main_args.result_path, env_args_dict, main_args, env_index, total_reward, total_time, dbs_trajectory)
#        print(f'[{env_index-main_args.index_start+1}/{main_args.index_end-main_args.index_start}] Total reward: {total_reward:.2f}, Total time: {total_time}')
        avg_reward += total_reward
    num_exp = main_args.index_end-main_args.index_start
    print(f'Depth: {main_args.tree_depth}, #users: {main_args.num_user}, datarate: {main_args.datarate}, avg_reward: {avg_reward/num_exp}, avg_obj: {avg_obj/num_exp}')
