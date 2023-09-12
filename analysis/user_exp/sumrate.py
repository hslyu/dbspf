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

#base="/home/hslyu/storage/bw20/ours-user"
#comparison_mode=False
comparison_mode=True
#base="/home/hslyu/storage/bw20/circular-user/datarate_5/"
base="/home/hslyu/storage/bw20/fixed-user/datarate_5/"

num_env=150
def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth, user):

    sumrate = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_{user}.json"
        
        user_list = open_json(os.path.join(root, filename))
        data = sum([user['total_data']-10 for user in user_list])

        sumrate += data/20

    sumrate /= num_env

    return sumrate

if __name__=="__main__":
    if comparison_mode:
        depth = 1
        coverage_list = []
        for user in range(10,81,10):
            root = base
            coverage = parse(root, depth, user)
            coverage_list.append(coverage)
            print(coverage)
    else:
        for depth in range(1,6,2):
            sumrate_list = []
            print(f"-----------depth: {depth}-------------")
            for user in range(10,81,10):
                root = os.path.join(base,f"user_{user}/depth_{depth}")
                sumrate = parse(root, depth, user)
                sumrate_list.append(sumrate)
                print(sumrate)
            print(f"-----------depth: {depth}-------------")
