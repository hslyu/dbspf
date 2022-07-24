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
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth):

    coverage = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_20.json"
        
        user_list = open_json(os.path.join(root, filename))
        list_data = [0 for user in user_list if user['total_data'] != 10]

        coverage += len(list_data)/len(user_list)

    coverage /= num_env

    return coverage

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("datarate")
    plt.ylabel("coverage")

    if comparison_mode:
        depth = 1
        coverage_list = []
        for d in range(0,11):
            root = os.path.join(base,f"datarate_{d}")
            coverage = parse(root, depth)
            coverage_list.append(coverage)
            print(coverage)
        plt.plot(coverage_list, label=name)
        plt.legend(loc='best')

    else:
        for depth in range(1,6,2):
            coverage_list = []
            print(f"-----------depth: {depth}-------------")
            for d in range(0,11):
                root = os.path.join(base,f"datarate_{d}/user_20/depth_{depth}")
                coverage = parse(root, depth)
                coverage_list.append(coverage)
                print(coverage)
            print(f"-----------depth: {depth}-------------")
            plt.plot(coverage_list, label=f"depth_{depth}")
            plt.legend(loc='best')

    plt.show()

