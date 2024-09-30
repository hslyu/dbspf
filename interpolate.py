#!/usr/bin/env python

import copy
import inspect
import json
import math

import matplotlib
import numpy as np
from scipy.interpolate import CubicSpline

import drone_basestation as db

matplotlib.use("module://matplotlib-backend-wezterm")
import matplotlib.pyplot as plt


def create_user_from_dict(data_dict):
    init_params = inspect.signature(db.User.__init__).parameters
    filtered_data = {
        key: value for key, value in data_dict.items() if key in init_params
    }
    filtered_data["tw_size"] = data_dict["time_end"] - data_dict["time_start"] + 1
    user = db.User(**filtered_data)
    user.ra = data_dict["ra"]
    user.psd = data_dict["psd"]
    user.total_data = data_dict["total_data"]
    return user


def load_json(path):
    with open(path, encoding="utf-8") as f:
        result = json.load(f)
        trajectory_info = result["trajectory"]
        path = []
        for node_info in trajectory_info:
            node = db.TrajectoryNode(node_info["position"])
            node.current_time = node_info["current_time"]
            user_list = []
            for user_info in node_info["user_list"]:
                user = create_user_from_dict(user_info)
                user_list.append(user)
            node.user_list = user_list
            path.append(node)
    return path


def compute_node_with_interpolation(
    path, num_step, interpolation="linear", compute_rrm=False
):
    if interpolation == "cubic":
        position_list = [node.position for node in path]

        x_vals = [p[0] for p in position_list]
        y_vals = [p[1] for p in position_list]
        z_vals = [p[2] for p in position_list]

        t = np.arange(len(position_list))
        cs_x = CubicSpline(t, x_vals)
        cs_y = CubicSpline(t, y_vals)
        cs_z = CubicSpline(t, z_vals)

        t_fine = np.linspace(t[0], t[-1], len(t) * num_step)
        x_fine = cs_x(t_fine)
        y_fine = cs_y(t_fine)
        z_fine = cs_z(t_fine)

        cubic_position_list = list(zip(x_fine, y_fine, z_fine))

    current_node = path[0]
    total_data_list = [10 for _ in current_node.user_list]

    node_list = []
    for next_node in path[1:]:
        for i in range(num_step):
            if interpolation == "linear":
                ratio = (num_step - i) / num_step
                node_position = np.array(current_node.position) * ratio + np.array(
                    next_node.position
                ) * (1 - ratio)
            elif interpolation == "cubic":
                node_position = cubic_position_list[
                    current_node.current_time * num_step + i
                ]
            node = db.TrajectoryNode(np.asarray(node_position))
            node.current_time = current_node.current_time + i / num_step

            user_list = []
            for user in current_node.user_list:
                user_position = [
                    user.position[0] + user.velocity[0] * i / num_step,
                    user.position[1] + user.velocity[1] * i / num_step,
                ]
                u = copy.deepcopy(user)
                u.total_data = total_data_list[u.id]
                u.received_data = 0
                u.position = user_position
                u.pathloss = node.get_pathloss(node.position, u)
                if not compute_rrm:
                    u.snr = node.psd2snr(u.psd, u.pathloss)
                    u.se = node.snr2se(u.snr)
                    u.received_data = u.ra * u.se / num_step
                    u.total_data += u.received_data
                    total_data_list[u.id] = u.total_data

                user_list.append(u)
            node.user_list = user_list
            node.get_reward(1)
            if compute_rrm:
                for u in node.user_list:
                    u.received_data = u.ra * u.se / num_step
                    total_data_list[u.id] += u.received_data
                    u.total_data = total_data_list[u.id]
            node_list.append(node)

        current_node = next_node

    return node_list


def compute_pf(node):
    return sum(
        [
            math.log(user.total_data - 10)
            for user in node.user_list
            if user.total_data > 11
        ]
    )


if __name__ == "__main__":
    path = load_json(
        "result/ours/datarate_10/user_20/depth_1/env_0002-depth_5-ue_20.json"
    )
    num_step = 100
    node_list = compute_node_with_interpolation(path, num_step, "cubic")

    discrete_pf = []
    for node in path:
        discrete_pf.append(compute_pf(node))

    node_list = compute_node_with_interpolation(path, num_step, "cubic")
    continuous_pf_without_rrm = []
    for node in node_list:
        continuous_pf_without_rrm.append(compute_pf(node))

    node_list = compute_node_with_interpolation(path, num_step, "cubic", True)
    continuous_pf_with_rrm = []
    for node in node_list:
        continuous_pf_with_rrm.append(compute_pf(node))

    print(f"{discrete_pf = };")
    print(f"{continuous_pf_without_rrm = };")
    print(f"{continuous_pf_with_rrm = };")
