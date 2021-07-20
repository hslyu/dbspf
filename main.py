#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import argparse
import os
import json
import environment_manager as em
import drone_basestation as dbs

def get_parser():
    parser = argparse.ArgumentParser(description='Simulate drone base station with specific depth',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-t', '--tree_depth', type=int, help='Tree depth')
    parser.add_argument('--data_path', default=os.path.join(os.getcwd(), 'data'), type=str, help='Path of the environment directory')
    parser.add_argument('--index_start', default=0, type=int, help='Iteration start index')
    parser.add_argument('--index_end', type=int, help='Iteration end index')
    return parser

def save_result(data_path, env_index, total_reward, total_time, trajectory):
    result = {}
    result['configuration_path'] = os.path.join(data_path, 'args.json')
    result['environment_path'] = os.path.join(data_path, f'env/env_{env_index:04d}.json')
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
    with open(os.path.join(data_path, f'result/result_{env_index:04d}.json'), 'w') as f:
        json.dump(result, f, ensure_ascii=False, indent=4)

if __name__ =="__main__":
    parser = get_parser()
    args = parser.parse_args()

    if not bool(args.tree_depth):
        parser.error("Tree depth must be specified. Usage: {} --tree-depth 3".format(__file__))
    if not bool(args.index_end):
        parser.error("End iteration index should be specified.".format(__file__))

    # Load environment
    env_args = em.load_args(args.data_path)
    # Create directory to store the result
    em.create_dir(os.path.join(args.data_path, 'result'))
    
    # Load root node and start trajectory plannnig
    for env_index in range(args.index_start, args.index_end):
        root = em.load_root(args.data_path, env_args, env_index)
        tree = dbs.TrajectoryTree(root, env_args.vehicle_velocity,
                                env_args.time_step, env_args.grid_size,
                                env_args.map_width, env_args.min_altitude, env_args.max_altitude,
                                args.tree_depth, env_args.max_time)
        dbs_trajectory = tree.pathfinder()
        total_reward = 0
        total_time = 0
        for node in dbs_trajectory:
            total_reward += node.reward
            total_time += node.elapsed_time
        save_result(args.data_path, env_index, total_reward, total_time, dbs_trajectory)
        print(f'[{env_index-args.index_start+1}/{args.index_end-args.index_start}] Total reward: {total_reward:.2f}, Total time: {total_time}')

