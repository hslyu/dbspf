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

import system_model as sm
import numpy as np

num_env=150
def parse(root: str="/home/hslyu/storage/result_twc21_07_05_prev/tw20_user20"):
    list_avg_rate = []

    for i in range(0,11):
        path = os.path.join(root,f"datarate_{i}")
        path = os.path.join(root,f"datarate_10")
        avg_rate = 0
        for j in range(num_env):
            env_name = f"env_{j:04d}"

            env_rate = 0
            prev_list_data = [10]*20
            list_data = []
            count = 0
            k = 1
            while k <= 19: 
                pkl_name = f"UBS_{k}.pkl" 
                file = os.path.join(path, env_name, pkl_name)
                if os.path.exists(file):
                    with open(file, 'rb') as f:
                        UBS = pickle.load(f)

                    list_data = [int(ue.serviced_data) for ue in UBS.list_ue]
                    list_data_diff = [data - prev_data for data, prev_data in zip(list_data, prev_list_data)]
                    print(list_data_diff)
                    env_rate += sum(list_data_diff)
                    count += np.count_nonzero(list_data_diff)

                    prev_list_data = list_data
                k += 1

            env_rate = env_rate/count if count != 0 else env_rate
            avg_rate += env_rate
            
        list_avg_rate.append(avg_rate/num_env)

    for avg_rate in list_avg_rate:
        print(avg_rate)
    return list_avg_rate

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("time")
    plt.ylabel("datarate")
    plt.plot(parse(), label=f"TWC21")
        
    plt.legend(loc='best')

    plt.show()
