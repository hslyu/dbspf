#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import json
import os
import pickle
import matplotlib.pyplot as plt

# Import parent directory
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
from utils import open_json, create_dir

def plot_per_ue(dir_path, num_user_list, depth):#{{{
    with open(dir_path+'time_per_depth.pkl', 'rb') as f:
        time_per_depth = pickle.load(f)
    with open(dir_path+'reward_per_depth.pkl', 'rb') as f:
        reward_per_depth = pickle.load(f)
    with open(dir_path+'avg_data_per_depth.pkl', 'rb') as f:
        avg_data_per_depth = pickle.load(f)
    with open(dir_path+'var_data_per_depth.pkl', 'rb') as f:
        var_data_per_depth = pickle.load(f)

    plot1 = plt.figure(1)
    for idx in range(depth):
        plt.plot(num_user_list, time_per_depth[idx], label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Time")
    plt.legend()
    
    plot2 = plt.figure(2)
    for idx in range(depth):
        plt.plot(num_user_list, reward_per_depth[idx],label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Reward per time step")
    plt.legend()

    plot3 = plt.figure(3)
    for idx in range(depth):
        plt.plot(num_user_list, 
                [reward /time for reward, time in zip(reward_per_depth[idx], time_per_depth[idx])],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Avg. reward / computation time")
    plt.legend()

    plot4 = plt.figure(4)
    for idx in range(depth):
        plt.plot(num_user_list, 
                avg_data_per_depth[idx],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Average of the user received data")
    plt.legend()

    plot5 = plt.figure(5)
    for idx in range(depth):
        plt.plot(num_user_list, 
                var_data_per_depth[idx],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Variance of the user received data")
    plt.legend()
    plt.show()
    #}}}

if __name__=="__main__":
    # Load data
    num_env = 20
    max_time = 200
    max_depth = 4
    num_user_list = list(range(10,56,5))

    result_dir = os.path.join(os.getcwd(),'../result')
    pkl_dir_path = 'some-value_per_depth/'

    '''
    time_per_depth = []#{{{
    reward_per_depth = []
    avg_data_per_depth = []
    var_data_per_depth = []
    for depth in range(1, max_depth+1):
        avg_time_per_ue = []
        avg_reward_per_ue = []
        avg_data_per_ue = []
        var_data_per_ue = []
        for num_ue in num_user_list:
            dirname = os.path.join(result_dir,f'result-depth_{depth}-user_{num_ue}')
            avg_time_per_step = 0
            avg_reward_per_step = 0
            avg_data = 0
            var_data = 0
            for env_index in range(num_env):
                # Open file
                filename = f'env_{env_index:04d}-depth_{depth}-ue_{num_ue}.json'
                result = open_json(os.path.join(dirname,filename))
                # Save time, reward result
                avg_time_per_step += result['total_time']/max_time
                avg_reward_per_step += result['total_reward']/max_time
                # Save avg., var.
                user_list = result['trajectory'][-1]['user_list']
                total_data = sum([user['total_data'] for user in user_list])
                total_data_square = sum([user['total_data']**2 for user in user_list])
                avg_data += total_data/len(user_list)
                var_data += total_data_square/len(user_list) - (total_data/len(user_list))**2
            # Normalize
            avg_time_per_step /= num_env
            avg_reward_per_step /= num_env
            avg_data /= num_env
            var_data /= num_env
            
            avg_time_per_ue.append(avg_time_per_step)
            avg_reward_per_ue.append(avg_reward_per_step)
            avg_data_per_ue.append(avg_data)
            var_data_per_ue.append(var_data)
            print(f'Step : [{depth}, {num_ue}],  Time : {avg_time_per_step:.6f},  Reward : {avg_reward_per_step:.2f}', 
                    end='\r', flush=True)
        print('')
        time_per_depth.append(avg_time_per_ue)
        reward_per_depth.append(avg_reward_per_ue)
        avg_data_per_depth.append(avg_data_per_ue)
        var_data_per_depth.append(var_data_per_ue)

    create_dir(pkl_dir_path)
    # Save time, reward
    with open(pkl_dir_path+'time_per_depth.pkl', 'wb') as f:
        pickle.dump(time_per_depth, f)
    with open(pkl_dir_path+'reward_per_depth.pkl', 'wb') as f:
        pickle.dump(reward_per_depth, f)
    with open(pkl_dir_path+'avg_data_per_depth.pkl', 'wb') as f:
        pickle.dump(avg_data_per_depth, f)
    with open(pkl_dir_path+'var_data_per_depth.pkl', 'wb') as f:
        pickle.dump(var_data_per_depth, f)#}}}
    '''
    plot_per_ue(pkl_dir_path, num_user_list, max_depth)
