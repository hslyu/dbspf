#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import json
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
import math

# Import parent directory
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

#base="/home/hslyu/storage/ours-rate-ue20-tw8"
#comparison_mode=False
comparison_mode=True
#base="/home/hslyu/storage/circular-rate-ue20-tw8"
base="/home/hslyu/storage/fixed-rate-ue20-tw8"

num_env=150
def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        time = result['total_time']
        reward = result['total_reward']
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth):

    pf = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_20.json"
        
        user_list = open_json(os.path.join(root, filename))
        pf += sum([math.log(user['total_data']-10) for user in user_list if user['total_data'] != 10])

    pf /= num_env

    return pf

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("datarate")
    plt.ylabel("pf")

    for depth in range(1,6,2):
        pf_list = []
        for d in range(0,11):
            if comparison_mode:
                root = os.path.join(base,f"datarate_{d}")
            else:
                root = os.path.join(base,f"datarate_{d}/user_20/depth_{depth}")
            pf = parse(root, depth)
            pf_list.append(pf)
            print(pf)
        print(f"-----------depth: {depth}-------------")
        plt.plot(pf_list, label=f"depth_{depth}")
        plt.legend(loc='best')

    plt.show()

