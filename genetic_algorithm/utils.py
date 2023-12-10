#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station iterative simulation code.
# If you want to simulate just one environment, execute drone_basestation.py.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import argparse
import json
import math
import os

import drone_basestation as dbs
from drone_basestation import TrajectoryNode, User


def load_root(path, num_user, env_index):
    with open(os.path.join(path, f"env/env_{env_index:04d}.json")) as f:
        env = json.load(f)
        if env["num_iteration"] != env_index:
            print("FATAL ERROR (load_root) : iteration index is not matched")
            exit()

        root = TrajectoryNode(env["root_position"])

        user_list = []
        user_dict_list = env["user_list"]
        for user_dict in user_dict_list[0:num_user]:
            user = User(*user_dict.values())
            user_list.append(user)
        root.user_list = user_list

    return root, user_list
