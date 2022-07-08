#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import json
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

# Import parent directory
import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import system_model as sm

num_env=150
def parse(root: str="/home/hslyu/storage/result_twc21_07_07/tw20_user20/datarate_0"):
    list_pf = np.zeros(20)

    for j in range(num_env):
        env_name = f"env_{j:04d}"
        pf = 0

        t = 0
        while t < 20: 
            pkl_name = f"UBS_{t}.pkl" 
            file = os.path.join(root, env_name, pkl_name)
            if os.path.exists(file):
                with open(file, 'rb') as f:
                    UBS = pickle.load(f)
                    pf = UBS.calc_PF()
            list_pf[t] += pf
            t += 1

    list_pf /= num_env

    return list_pf

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("time")
    plt.ylabel("pf")
    for d in range(0,11):
        root = f"/home/hslyu/storage/result_twc21_07_07/tw20_user20/datarate_{d}"
        plt.plot(parse(root), label=f"datarate_{d}")
        
    plt.legend(loc='best')

    plt.show()

