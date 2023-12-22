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

# base="/home/hslyu/storage/bw20/ours-rate"
# comparison_mode=False
comparison_mode = True
# base="/home/hslyu/storage/bw20/circular-rate"
# base="/home/hslyu/storage/bw20/fixed-rate"
# base = "/home/hslyu/storage/dbspf/bw2/genetic-rate"
base = "/home/hslyu/storage/dbspf/bw2/dqn-rate"

num_env = 150


def open_json(file_path):
    with open(file_path, encoding="utf-8") as f:
        result = json.load(f)
        user_list = result["trajectory"][-1]["user_list"]

        return user_list


def parse(root: str, depth):
    sumrate = 0
    for j in range(num_env):
        filename = f"env_{j:04d}-depth_{depth}-ue_20.json"

        user_list = open_json(os.path.join(root, filename))
        data = sum([user["total_data"] - 10 for user in user_list])

        sumrate += data / 20

    sumrate /= num_env

    return sumrate


if __name__ == "__main__":
    plot = plt.figure(1)
    plt.xlabel("datarate")
    plt.ylabel("sumrate")

    if comparison_mode:
        depth = 1
        coverage_list = []
        for d in range(0, 11):
            root = os.path.join(base, f"datarate_{d}")
            coverage = parse(root, depth)
            coverage_list.append(coverage)
            print(coverage)
        plt.plot(coverage_list, label=name)
        plt.legend(loc="best")
    else:
        for depth in range(1, 6, 2):
            sumrate_list = []
            print(f"-----------depth: {depth}-------------")
            for d in range(0, 11):
                root = os.path.join(base, f"datarate_{d}/user_20/depth_{depth}")
                sumrate = parse(root, depth)
                sumrate_list.append(sumrate)
                print(sumrate)
            print(f"-----------depth: {depth}-------------")
            plt.plot(sumrate_list, label=f"depth_{depth}")
            plt.legend(loc="best")

    plt.show()
