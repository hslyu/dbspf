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
from utils import create_dir

def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        time = result['total_time']
        reward = result['total_reward']
        user_list = result['trajectory'][-1]['user_list']
        total_data = sum([user['total_data'] for user in user_list])
        total_data_square = sum([user['total_data']**2 for user in user_list])
        
        avg_data = total_data/len(user_list)
        var_data = total_data_square/len(user_list)
        return time, reward, avg_data, var_data

def parse_result(num_env, max_time, depth, num_ue, datarate_list, result_dir, pkl_dir_path):#{{{
    avg_time_per_step_per_datarate = []
    avg_reward_per_step_per_datarate = []
    avg_data_per_datarate = []
    var_data_per_datarate = []
    for datarate in datarate_list:
        dirname = os.path.join(result_dir,f'datarate_{datarate}')

        avg_time_per_step = 0
        avg_reward_per_step = 0
        avg_data = 0
        var_data = 0
        for env_idx in range(num_env):
            # Open file
            filename = f'env_{env_idx:04d}-depth_{depth}-ue_{num_ue}.json'
            time, reward, env_avg_data, env_var_data = open_json(os.path.join(dirname,filename))

            # Get average
            avg_time_per_step += time
            avg_reward_per_step += reward
            avg_data += env_avg_data
            var_data += env_var_data
            print(f'Environment: [{env_idx+1}/{num_env}] for datarate {datarate}', end='\r', flush=True)
        print('')
        # Normalize
        avg_time_per_step /= num_env*max_time
        avg_reward_per_step /= num_env*max_time
        avg_data /= num_env
        var_data /= num_env
        # Save
        avg_time_per_step_per_datarate.append(avg_time_per_step)
        avg_reward_per_step_per_datarate.append(avg_reward_per_step)
        avg_data_per_datarate.append(avg_data)
        var_data_per_datarate.append(var_data)

    create_dir(pkl_dir_path)
    # Save time, reward
    with open(pkl_dir_path+'avg_time_per_step_per_datarate.pkl', 'wb') as f:
        pickle.dump(avg_time_per_step_per_datarate, f)
    with open(pkl_dir_path+'avg_reward_per_step_per_datarate.pkl', 'wb') as f:
        pickle.dump(avg_reward_per_step_per_datarate, f)
    with open(pkl_dir_path+'avg_data_per_datarate.pkl', 'wb') as f:
        pickle.dump(avg_data_per_datarate, f)
    with open(pkl_dir_path+'var_data_per_datarate.pkl', 'wb') as f:
        pickle.dump(var_data_per_datarate, f)#}}}

def plot_per_datarate(pkl_dir_path, datarate_list):#{{{
    with open(pkl_dir_path+'avg_time_per_step_per_datarate.pkl', 'rb') as f:
        avg_time_per_step_per_datarate = pickle.load(f)
    with open(pkl_dir_path+'avg_reward_per_step_per_datarate.pkl', 'rb') as f:
        avg_reward_per_step_per_datarate = pickle.load(f)
    with open(pkl_dir_path+'avg_data_per_datarate.pkl', 'rb') as f:
        avg_data_per_datarate = pickle.load(f)
    with open(pkl_dir_path+'var_data_per_datarate.pkl', 'rb') as f:
        var_data_per_datarate = pickle.load(f)

    plot1 = plt.figure(1)
    plt.plot(datarate_list, avg_time_per_step_per_datarate)
    plt.xlabel("Datarate")
    plt.ylabel("Average time per step")
    
    plot2 = plt.figure(2)
    plt.plot(datarate_list, avg_reward_per_step_per_datarate)
    plt.xlabel("Datarate")
    plt.ylabel("Reward per step")

    plot3 = plt.figure(3)
    plt.plot(datarate_list, 
            [reward /time for reward, time in zip(avg_reward_per_step_per_datarate, avg_time_per_step_per_datarate)])
    plt.xlabel("Datarate")
    plt.ylabel("Avg. reward/Avg. time")

    plot4 = plt.figure(4)
    plt.plot(datarate_list, avg_data_per_datarate)
    plt.xlabel("Datarate")
    plt.ylabel("Average of user received data")

    plot5 = plt.figure(5)
    plt.plot(datarate_list, var_data_per_datarate)
    plt.xlabel("Datarate")
    plt.ylabel("Variance of user received data")
    plt.show()#}}}
    
if __name__=="__main__":
    # Load data
    num_env = 100
    max_time = 200
    depth = 2
    num_ue = 40
    datarate_list = list(range(5,101,5))
    # Input and output directory path
    result_dir = os.path.join(os.getcwd(),'../result_datarate')
    pkl_dir_path = 'some-value_per_datarate/'

    # Executions
#    parse_result(num_env, max_time, depth, num_ue, datarate_list, result_dir, pkl_dir_path)
    plot_per_datarate(pkl_dir_path, datarate_list)
