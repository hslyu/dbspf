#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import json
import os
import pickle
import matplotlib.pyplot as plt
from open_json import open_json

dir_path = 'analysis/some-value_per_depth/'

def plot_time_reward_per_ue(depth,dir_path):#{{{
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
    plt.ylabel("Time per user")
    plt.legend()
    
    plot2 = plt.figure(2)
    for idx in range(depth):
        plt.plot(num_user_list, 
                [reward /time for reward, time in zip(reward_per_depth[idx], time_per_depth[idx])],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Total reward per unit time")
    plt.legend()

    plot3 = plt.figure(3)
    for idx in range(depth):
        plt.plot(num_user_list, 
                avg_data_per_depth[idx],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Avg data per user")
    plt.legend()

    plot4 = plt.figure(4)
    for idx in range(depth):
        plt.plot(num_user_list, 
                var_data_per_depth[idx],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("var data per user")
    plt.legend()

    plot5 = plt.figure(5)
    for idx in range(depth):
        plt.plot(num_user_list, 
                [reward/200 for reward in reward_per_depth[idx]],
                label = f'depth{idx+1}')
    plt.xlabel("Number of user")
    plt.ylabel("Reward per depth")
    plt.legend()
    plt.show()#}}}

if __name__=="__main__":
    # Load data
    num_env = 20
    max_time = 200
    depth = 5
    master_dir = os.path.join(os.getcwd(),'result')
    num_user_list = list(range(10,56,5))


    '''
    time_per_depth = []#{{{
    reward_per_depth = []
    avg_data_per_depth = []
    var_data_per_depth = []
    for depth in range(1,6):
        avg_time = []
        avg_reward = []
        avg_data = []
        var_data = []
        for num_ue in num_user_list:
            dirname = os.path.join(master_dir,f'result-depth_{depth}-user_{num_ue}')
            total_time = 0
            total_reward = 0
            total_data = 0
            total_data_square = 0
            for env_index in range(num_env):
                filename = f'env_{env_index:04d}-depth_{depth}-ue_{num_ue}.json'
                result = open_json(os.path.join(dirname,filename))
                total_time += result['total_time']
                total_reward += result['total_reward']
                user_list = result['trajectory'][-1]['user_list']
                for user in user_list:
                    total_data += user['total_data']
                    total_data_square += user['total_data']**2
            avg_data.append(total_data/num_ue/num_env)
            var_data.append(total_data_square/num_ue/num_env - (total_data/num_ue/num_env)**2)

            avg_time_per_step = total_time/num_env
            avg_reward_per_step = total_reward/num_env
            avg_time.append(avg_time_per_step)
            avg_reward.append(avg_reward_per_step)
            print(f'Step : [{depth}, {num_ue}],  Time : {avg_time_per_step:.6f},  Reward : {avg_reward_per_step:.2f}')
        time_per_depth.append(avg_time)
        reward_per_depth.append(avg_reward)
        avg_data_per_depth.append(avg_data)
        var_data_per_depth.append(var_data)
    # Save time, reward
    with open('analysis/time_per_depth.pkl', 'wb') as f:
        pickle.dump(time_per_depth, f)
    with open('analysis/reward_per_depth.pkl', 'wb') as f:
        pickle.dump(reward_per_depth, f)
    with open('analysis/avg_data_per_depth.pkl', 'wb') as f:
        pickle.dump(avg_data_per_depth, f)
    with open('analysis/var_data_per_depth.pkl', 'wb') as f:
        pickle.dump(var_data_per_depth, f)#}}}
    '''
    plot_time_reward_per_ue(depth, dir_path)
