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

num_env=150
def parse(root: str="/home/hslyu/storage/result_twc21_07_07/tw20_user20"):
    list_jfi = []

    for i in range(0,11):
        path = os.path.join(root,f"datarate_{i}")
        jfi = 0
        for j in range(num_env):
            env_name = f"env_{j:04d}"

            k = 19
            while k >= 0: 
                pkl_name = f"UBS_{k}.pkl" 
                file = os.path.join(path, env_name, pkl_name)
                if os.path.exists(file):
                    with open(file, 'rb') as f:
                        UBS = pickle.load(f)
                    break
                else:
                    k -= 1

            jfi += UBS.calc_JFI()

        list_jfi.append(jfi/num_env)

    for jfi in list_jfi:
        print(jfi)
    return list_jfi

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("time")
    plt.ylabel("jfi")
    plt.plot(parse(), label=f"TWC21")
        
    plt.legend(loc='best')

    plt.show()
