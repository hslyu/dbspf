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
sys.path.append('/home/hslyu/dbspf/comparison_scheme')

import system_model as sm
import numpy as np

num_env=120
def parse(root: str="/home/hslyu/storage/bw20/twc-user"):
    list_sumrate = []

    for user in range(10,81,10):
        path = os.path.join(root,f"tw20_user{user}/datarate_{5}")
        sumrate = 0
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

            list_ue = UBS.list_ue
            list_rate = [(ue.serviced_data-10) for ue in list_ue]
            sumrate += sum(list_rate)/num_env

        list_sumrate.append(sumrate/20)

    for sumrate in list_sumrate:
        print(sumrate)
    return list_sumrate

if __name__=="__main__":
    plot = plt.figure(1)
    plt.xlabel("time")
    plt.ylabel("datarate")
    plt.plot(parse(), label=f"TWC21")
        
    plt.legend(loc='best')

    plt.show()
