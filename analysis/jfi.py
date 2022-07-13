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

comparison_mode=True
base="/home/hslyu/storage/result_fixed_07_14/"
#base="/home/hslyu/storage/result_ours_07_12/tw20_user20/"
name="fixed"
num_env=150
def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth):

    jfi = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_20.json"
        
        user_list = open_json(os.path.join(root, filename))
        list_data = [user['total_data']-10 for user in user_list if user['total_data'] != 10]

        if sum(list_data) != 0:
            jfi +=  sum(list_data)**2/(len(list_data)*sum([data**2 for data in list_data]))

    jfi /= num_env

    return jfi

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("datarate")
    plt.ylabel("jfi")

    if comparison_mode:
        depth = 1
        jfi_list = []
        for d in range(0,11):
            root = base+f"datarate_{d}"
            jfi = parse(root, depth)
            jfi_list.append(jfi)
            print(jfi)
        print(f"-----------depth: {depth}-------------")
        plt.plot(jfi_list, label=f"depth_{depth}")
        plt.legend(loc='best')

    else:
        for depth in range(1,8):
            jfi_list = []
            for d in range(0,11):
                root = base+f"datarate_{d}/user_20/depth_{depth}"
                jfi = parse(root, depth)
                jfi_list.append(jfi)
                print(jfi)
            print(f"-----------depth: {depth}-------------")
            plt.plot(jfi_list, label=f"depth_{depth}")
            plt.legend(loc='best')

        plt.show()

