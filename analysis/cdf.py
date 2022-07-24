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

comparison_mode=False
#base="/home/hslyu/storage/result_circular_07_14/"
#name="circular"
#base="/home/hslyu/storage/result_fixed_07_14/"
#name="fixed"
base="/home/hslyu/storage/result_ours_07_14/"

d=10
num_env=150
def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
        user_list = result['trajectory'][-1]['user_list']
        
        return user_list

def parse(root: str, depth):

    list_cdf = [0]*101
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_20.json"
        
        idx=0
        user_list = open_json(os.path.join(root, filename))
        print([(user['total_data']-10)/user['serviced_time'] if user['serviced_time'] != 0 else 0 for user in user_list])
        count = 0
        for data in range(0,101,1):
            for user in user_list:
                if user['serviced_time'] == 0 :
                    list_cdf[idx] += 1/num_env/20
                elif (user['total_data']-10)/user['serviced_time'] <= data:
                    list_cdf[idx] += 1/num_env/20
            idx += 1
    return list_cdf

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("datarate")
    plt.ylabel("cdf")

    if comparison_mode:
        depth = 1
        cdf_list = []
        root = base+f"datarate_{d}"
        cdf = parse(root, depth)
        cdf_list.append(cdf)
        for prob in cdf:
            print(prob)
        plt.plot(cdf_list, label=name)
        plt.legend(loc='best')
    else:
#        for depth in range(1,7):
#            cdf_list = []
#            print(f"-----------depth: {depth}-------------")
#            root = base+f"datarate_{d}/user_20/depth_{depth}"
#            cdf = parse(root, depth)
#            cdf_list.append(cdf)
#            for prob in cdf:
#                print(prob)
#            print(f"-----------depth: {depth}-------------")
#            plt.plot(cdf_list, label=f"depth_{depth}")
#            plt.legend(loc='best')

        depth = 5
        cdf_list = []
        print(f"-----------depth: {depth}-------------")
        root = base+f"datarate_{d}/user_20/depth_{depth}"
        cdf = parse(root, depth)
        cdf_list.append(cdf)
#        for prob in cdf:
#            print(prob)
        print(f"-----------depth: {depth}-------------")
        plt.plot(cdf_list, label=f"depth_{depth}")
        plt.legend(loc='best')
    plt.show()

