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

base="/home/hslyu/storage/ours-user-rate5-tw8"
comparison_mode=False
#comparison_mode=True
#base="/home/hslyu/storage/circular-user-rate5-tw8/datarate_5/"
#base="/home/hslyu/storage/fixed-user-rate5-tw8/datarate_5/"
num_env=150
def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth, user):

    jfi = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_{user}.json"
        
        user_list = open_json(os.path.join(root, filename))
        list_data = [user['total_data']-10 for user in user_list if user['total_data'] != 10]

        if sum(list_data) != 0:
            jfi +=  sum(list_data)**2/(len(list_data)*sum([data**2 for data in list_data]))

    jfi /= num_env

    return jfi

if __name__=="__main__":
    if comparison_mode:
        depth = 1
        jfi_list = []
        for user in range(10,81,10):
            root = base
            jfi = parse(root, depth, user)
            jfi_list.append(jfi)
            print(jfi)
    else:
        for depth in range(1,6,2):
            jfi_list = []
            for user in range(10,81,10):
                root = os.path.join(base,f"user_{user}/depth_{depth}")
                jfi = parse(root, depth, user)
                jfi_list.append(jfi)
                print(jfi)
            print(f"-----------depth: {depth}-------------")
