#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr

import inspect
import json
import math

# Import parent directory
import os
import pickle
import sys

import matplotlib.pyplot as plt
import numpy as np

current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

# base="/home/hslyu/storage/bw20/ours-user/"
# comparison_mode=False
comparison_mode = True
# base="/home/hslyu/storage/bw20/circular-user/datarate_5/"
# base="/home/hslyu/storage/bw20/fixed-user/datarate_5/"
# base = "/home/hslyu/storage/dbspf/bw10/genetic-user/datarate_5/"
base = "/home/hslyu/storage/dbspf/bw2/dqn-user/datarate_5/"

num_env = 150


def open_json(file_path):
    with open(file_path, encoding="utf-8") as f:
        result = json.load(f)
        time = result["total_time"]
        reward = result["total_reward"]
        user_list = result["trajectory"][-1]["user_list"]

        return user_list


def parse(root: str, depth, user):
    pf = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_{user}.json"

        user_list = open_json(os.path.join(root, filename))
        pf += sum(
            [
                math.log(user["total_data"] - 10)
                for user in user_list
                if user["total_data"] != 10
            ]
        )

    pf /= num_env

    return pf


if __name__ == "__main__":
    if comparison_mode:
        depth = 1
        pf_list = []
        for user in range(10, 81, 10):
            root = base
            pf = parse(root, depth, user)
            pf_list.append(pf)
            print(pf)
    else:
        for depth in range(1, 6, 2):
            print(f"-----------depth: {depth}-------------")
            pf_list = []
            for user in range(10, 81, 10):
                root = os.path.join(base, f"user_{user}/depth_{depth}")
                pf = parse(root, depth, user)
                pf_list.append(pf)
                print(pf)
            print(f"-----------depth: {depth}-------------")
