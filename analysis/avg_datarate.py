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


num_env=150
def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth):

    avg_rate = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_20.json"
        
        user_list = open_json(os.path.join(root, filename))
        time = sum([user['serviced_time'] for user in user_list])
        data = sum([user['total_data']-10 for user in user_list if user['total_data'] != 10])

        avg_rate = avg_rate + data/time if time != 0 else avg_rate

    avg_rate /= num_env

    return avg_rate

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("datarate")
    plt.ylabel("coverage")

    for depth in range(1,8):
        coverage_list = []
        print(f"-----------depth: {depth}-------------")
        for d in range(0,11):
            root = f"~/storage/result_ours_07_10/tw20_user20/datarate_{d}/user_20/depth_{depth}"
            coverage = parse(root, depth)
            coverage_list.append(coverage)
            print(coverage)
        print(f"-----------depth: {depth}-------------")
        plt.plot(coverage_list, label=f"depth_{depth}")
        plt.legend(loc='best')

    plt.show()

