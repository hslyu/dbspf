#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import math
import random
import time
from dataclasses import dataclass

import numpy as np

# Constant for wirless communication
FREQUENCY = 2.0 * 1e9  # Hz
LIGHTSPEED = 3 * 1e8  # m/s
BANDWIDTH_ORIG = 2  # MHz
NUM_SUBCARRIER = BANDWIDTH_ORIG * 5
SUBCARRIER_BANDWIDTH = 0.2  # MHz, 200 kHz
POWER_ORIG = 200  # mW
BANDWIDTH = 1.0  # <BANDWIDTH_ORIG> MHz per unit
POWER = 1.0  # 200 mW per unit
RICIAN_K = 12
NOISE_DENSITY = -173.8  # noise spectral density(Johnson-Nyquist_noise)
LOS_EXCESSIVE = 1  # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 40  # dB, excessive pathloss of nlos link
# LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
# NLOS_EXCESSIVE = 20 # dB, excessive pathloss of nlos link
SURROUNDING_A = 9.64  # Envrionmental parameter for probablistic LOS link
SURROUNDING_B = 0.06  # Envrionmental parameter for probablistic LOS link
# SURROUNDING_A = 9.64 # Envrionmental parameter for probablistic LOS link
# SURROUNDING_B = 0.35 # Envrionmental parameter for probablistic LOS link
# Optimization hyperparameter
EPSILON = 1e-9
STEP_SIZE = 1e-3
THRESHOLD = 1e-8
# etc
INF = 1e8 - 1


class User:
    def __init__(
        self,
        uid=0,
        position=[0, 0],
        time_start=0,
        tw_size=0,
        time_period=0,
        datarate=0,
        initial_data=0,
        max_data=0,
    ):
        # ------------      Below attributes are constant       ------------ #
        self.id = uid
        self.position = position
        self.time_start = time_start
        self.time_end = self.time_start + tw_size - 1
        self.time_period = time_period
        self.serviced_time = 0
        # Requiring datarate Mb/s
        self.datarate = datarate
        self.max_data = max_data
        self.pathloss = 0.0
        # ------------    Properties with ground base station   ------------ #
        self.pathloss_gbs = 0
        self.snr_gbs = 0
        self.se_gbs = 0
        # ------------  Below attributes are control variables  ------------ #
        self.ra = 0.0
        self.psd = 0.0
        # ------ Below attributes are variables determined by ra, psd ------ #
        self.se = 0.0
        self.snr = 0.0
        # Total received throughput
        # Initial is not zero for our formulation
        self.total_data = initial_data
        self.received_data = 0.0

    def __str__(self):
        return "id: {}, (x,y) : ({:.2f}, {:.2f}), tw : [{}, {}], tw_size : {}".format(
            self.id,
            self.position[0],
            self.position[1],
            self.time_start,
            self.time_end,
            self.time_end - self.time_start,
        )

    def __repr__(self):
        return "{}".format(self.id)


@dataclass
class GroundBaseStation:
    def __init__(self):
        self.position = [MAP_WIDTH / 2, MAP_WIDTH / 2, 30]
        self.pathloss_alpha = 4

    def user_channel(self, user: User):
        return 10 * math.log10(self.distance(user)) * self.pathloss_alpha

    def distance(self, user: User):
        return (
            (user.position[0] - self.position[0]) ** 2
            + (user.position[1] - self.position[1]) ** 2
            + 30**2
        ) ** 0.5


class TrajectoryNode:
    def __init__(self, position, num_iter=0, parent=None, gbs=None):
        # value
        self.position = position
        self.elapsed_time = 0.0
        # link
        self.leafs = []
        self.parent = parent
        self.GBS = gbs
        # for IITP computation
        self.serviced_ua_list = []

        # If this node is root
        if self.parent is None:
            self.current_time = 0
            self.reward = 0
        else:
            # Copy parent information
            self.current_time = parent.current_time + 1
            self.user_list = [User() for i in range(len(parent.user_list))]
            self.copy_user(parent.user_list)

            # Ground base station
            #            gbs = GroundBaseStation()
            if self.GBS is not None:
                for user in self.user_list:
                    user.pathloss_gbs = self.GBS.user_channel(user)

            # Calculate reward
            self.reward = self.get_reward(num_iter)

    #            self.reward = self.get_random_reward()

    def __repr__(self):
        return "{}".format(self.position)

    def copy_user(self, user_list):
        for idx, user in enumerate(user_list):
            self.user_list[idx].id = user.id
            self.user_list[idx].position = user.position
            self.user_list[idx].time_start = user.time_start
            self.user_list[idx].time_end = user.time_end
            self.user_list[idx].time_period = user.time_period
            self.user_list[idx].serviced_time = user.serviced_time
            self.user_list[idx].datarate = user.datarate
            self.user_list[idx].max_data = user.max_data
            self.user_list[idx].total_data = user.total_data
            self.user_list[idx].pathloss = self.get_pathloss(self.position, user)
            if self.current_time % user.time_period == 0:
                self.user_list[idx].received_data = 0
            else:
                self.user_list[idx].received_data = user.received_data

    def get_info(self):
        print(
            f"================================Current step: {self.current_time}====================================="
        )
        print("User throughput list (Mbps)")
        print(
            [
                0
                if user.serviced_time == 0
                else ((user.total_data - 10) / (user.serviced_time)) * 100 // 1 / 100
                for user in self.user_list
            ]
        )
        print("User total data (Mb)")
        print([(user.total_data - 10) // 1 for user in self.user_list])
        print(
            "====================================================================================="
        )

    def __repr__(self):
        return "{}".format(self.position)

    def get_random_reward(self):
        return random.randint(0, 10)

    def distance_from_leaf(self, position, user):
        """
        Distance between the UAV and i-th User
        """
        return math.sqrt(
            (position[0] - user.position[0]) ** 2
            + (position[1] - user.position[1]) ** 2
            + position[2] ** 2
        )

    def get_pathloss(self, position, user):
        """
        Caculate pathloss --> snr --> spectral efficiency
        """
        distance = self.distance_from_leaf(position, user)
        angle = math.asin(position[2] / distance)
        los_prob = 1 / (
            1
            + SURROUNDING_A
            * math.exp(-SURROUNDING_B * (180 / math.pi * angle - SURROUNDING_A))
        )
        pathloss = (
            20 * math.log10(4 * math.pi * FREQUENCY * distance / LIGHTSPEED)
            + los_prob * LOS_EXCESSIVE
            + (1 - los_prob) * NLOS_EXCESSIVE
        )
        return pathloss

    def psd2snr(self, psd, pathloss):
        """
        Because unit of psd is 200mW/2MHz = 1e-4 mw/Hz, we should convert it to mw/Hz
        """

        if psd == 0:
            return 0
        else:
            return (
                10 * math.log10(psd * POWER_ORIG / (BANDWIDTH_ORIG * 1e6))
                - pathloss
                - NOISE_DENSITY
            )

    def snr2se(self, snr):
        """
        Because unit of resource is <BANDWIDTH_ORIG> MHz,
        we should convert the unit of se from bps/Hz to Mbps/<BANDWIDTH_ORIG> MHz
        """
        return BANDWIDTH_ORIG * math.log2(1 + 10 ** (snr / 10))

    def get_valid_user(self) -> list[User]:
        valid_users = []
        # Find valid user set
        for user in self.user_list:
            #            if user.time_start <= self.current_time%user.time_period <= user.time_end and \
            #                    user.max_data > user.received_data:
            if user.time_start <= self.current_time <= user.time_end:
                valid_users.append(user)
        # LIST OF VALID USERS
        return valid_users

    def one_user(self, user):
        user.ra = 1
        user.psd = 1  # POWER/BANDWIDTH
        user.snr = self.psd2snr(user.psd, user.pathloss)
        user.se = self.snr2se(user.snr)
        rate = user.ra * user.se
        if rate < user.datarate:
            return 0
        reward = self.objective_function([user.psd], [user])
        user.received_data += rate
        user.total_data += rate
        user.serviced_time += 1
        self.serviced_ua_list = [user]
        return reward

    def get_reward(self, num_iter=0, init_ua_ra_mode="local"):
        valid_user_list = self.get_valid_user()

        if valid_user_list == []:
            return 0
        elif len(valid_user_list) == 1:
            return self.one_user(valid_user_list[0])

        # Initialize user psd, snr, and se.
        for user in self.user_list:
            # initial psd of users
            user.psd = 1  # POWER/BANDWIDTH
            # SNR in dBm
            user.snr = self.psd2snr(user.psd, user.pathloss)
            user.se = self.snr2se(user.snr)
            if self.GBS != None:
                user.snr_gbs = self.psd2snr(user.psd, user.pathloss_gbs)
                user.se_gbs = self.snr2se(user.snr_gbs)

        if init_ua_ra_mode == "local":
            init_ua_ra = self.init_ua_ra_local
        elif init_ua_ra_mode == "max_SINR":
            init_ua_ra = self.init_ua_ra_max_SINR
        elif init_ua_ra_mode == "linear":
            init_ua_ra = self.init_ua_ra_linear
        else:  # init_ua_ra_mode == "opt":
            init_ua_ra = self.init_ua_ra_opt
        ua_list, ra_list = init_ua_ra()
        # Get rid of zero ra
        ua_list = [user for user in ua_list if user.ra != 0]
        psd_list = self.kkt_psd(ua_list)
        reward = self.objective_function([user.psd for user in ua_list], ua_list)
        prev_reward = reward
        count = 1

        if len(ua_list) != 1:
            while count < num_iter:
                count += 1
                ra_list = self.kkt_ra(ua_list)  # return ra
                psd_list = self.kkt_psd(ua_list)  # return psd
                reward = self.objective_function(psd_list, ua_list)
                if prev_reward - reward < 2e-3:
                    break
                prev_reward = reward

        # for user, ra, psd in zip(ua_list, ra_list, psd_list):
        #     user.ra = ra
        #     user.psd = psd
        #     user.snr = self.psd2snr(user.psd, user.pathloss)
        #     user.se = self.snr2se(user.snr)
        #     rate = ra * user.se
        #     self.user_list[user.id].received_data += rate
        #     self.user_list[user.id].total_data += rate
        #     user.serviced_time += 1

        self.discretized_rrm(ua_list, ra_list, psd_list)
        self.serviced_ua_list = ua_list

        reward_gbs = 0
        if self.GBS is not None:
            ua_list_not_served = [
                user for user in self.get_valid_user() if user not in ua_list
            ]
            ua_list, ra_list = self.init_ua_ra(ua_list_not_served, True)
            #            print(ua_list_not_served, ua_list, ra_list)
            psd_list = self.kkt_psd(ua_list)
            reward_gbs = self.objective_function(
                [user.psd for user in ua_list], ua_list
            )

        return reward + reward_gbs

    def discretized_rrm(self, ua_list, ra_list, psd_list):
        def _rician_fading(user):
            los_real = random.random()
            los_im = (1 - los_real**2) ** 0.5
            los_real *= (RICIAN_K / (RICIAN_K + 1)) ** 0.5
            los_im *= (RICIAN_K / (RICIAN_K + 1)) ** 0.5
            # los_real = random.random() * 0.9588299321097967
            # los_im = (0.9588299321097967 - los_real**2) ** 0.5

            nlos_real = random.random()
            nlos_im = (1 - los_real**2) ** 0.5
            nlos_real *= (1 / (RICIAN_K + 1)) ** 0.5
            nlos_im *= (1 / (RICIAN_K + 1)) ** 0.5
            # nlos_real = random.gauss(0, 0.5) * 0.2839809171235324
            # nlos_im = random.gauss(0, 0.5) * 0.2839809171235324

            rician_fading = (
                (los_real + nlos_real) ** 2 + (los_im + nlos_im) ** 2
            ) ** 0.5
            pathloss_with_fading = user.pathloss - 10 * math.log10(rician_fading)
            return pathloss_with_fading

        discrete_ra_list = [int(NUM_SUBCARRIER * ra) for ra in ra_list]
        remainder = NUM_SUBCARRIER - sum(discrete_ra_list)
        diff = [ra - int(ra) for ra in ra_list]
        # Get indices of the top N elements
        indices = sorted(range(len(diff)), key=lambda i: diff[i])[-remainder:]
        # choose top <remainder> users from diff
        for idx in indices:
            discrete_ra_list[idx] += 1

        reward = 0
        for user, ra, psd in zip(ua_list, discrete_ra_list, psd_list):
            user.ra = ra
            user.psd = psd
            user.serviced_time += 1
            throughput = 0
            for _ in range(ra):
                user.snr = self.psd2snr(user.psd, _rician_fading(user))
                user.se = (
                    self.snr2se(user.snr) / BANDWIDTH_ORIG
                )  # normalize the spectral efficiency
                throughput += SUBCARRIER_BANDWIDTH * user.se
            reward += math.log(1 + throughput / user.total_data)
            user.received_data += throughput
            user.total_data += throughput
        return reward

    def init_ua_ra_linear(self, user_pool=None, isGBS=False):
        def ra_objective(sorted_valid_user_list, ra_list):
            if isGBS:
                return sum(
                    [
                        math.log(
                            1 + BANDWIDTH_ORIG * ra * user.se_gbs / user.total_data
                        )
                        if user.time_start <= self.current_time <= user.time_end
                        else 0
                        for user, ra in zip(sorted_valid_user_list, ra_list)
                    ]
                )
            else:
                return sum(
                    [
                        math.log(1 + BANDWIDTH_ORIG * ra * user.se / user.total_data)
                        if user.time_start <= self.current_time <= user.time_end
                        else 0
                        for user, ra in zip(sorted_valid_user_list, ra_list)
                    ]
                )

        def sort_key(user):
            # TODO: user.datarate is weird: user.datarate/user.se?
            if isGBS:
                return 1.0 / (user.total_data + user.datarate) / user.se
            else:
                return 1.0 / (user.total_data + user.datarate) / user.se

        if isGBS:
            if user_pool is None:
                print("Invalid user pool")
                exit()
            user_pool = user_pool
            sorted_valid_user_list = sorted(user_pool, key=sort_key, reverse=True)
            sorted_valid_user_list = [
                user
                for user in sorted_valid_user_list
                if user.datarate / user.se_gbs < BANDWIDTH
            ]
            ra_filler = [
                user.total_data / user.se_gbs for user in sorted_valid_user_list
            ]
            ra_min = [user.datarate / user.se_gbs for user in sorted_valid_user_list]
        else:
            user_pool = self.get_valid_user()
            sorted_valid_user_list = sorted(user_pool, key=sort_key, reverse=True)
            sorted_valid_user_list = [
                user
                for user in sorted_valid_user_list
                if user.datarate / user.se < BANDWIDTH
            ]
            ra_filler = [user.total_data / user.se for user in sorted_valid_user_list]
            ra_min = [user.datarate / user.se for user in sorted_valid_user_list]

        max_ra_list = []
        max_objective = -999999
        # Break flag for double for-loop
        is_infeasible = False
        for i in range(len(sorted_valid_user_list)):
            #            ra_list = [( BANDWIDTH + sum(ra_filler[:i+1]) ) / (i+1) - ra_filler[idx] for idx in range(i+1)] + [0] * ( len(self.user_list) - i - 1 )
            ra_list = [
                (BANDWIDTH + sum(ra_filler[: i + 1])) / (i + 1) - ra_filler[idx]
                for idx in range(i + 1)
            ]

            # Check feasibility of the requested datarate
            for idx in range(i + 1):
                if ra_min[idx] > ra_list[idx]:
                    is_infeasible = True
            if is_infeasible:
                break

            if max_objective < ra_objective(sorted_valid_user_list, ra_list):
                max_objective = ra_objective(sorted_valid_user_list, ra_list)
                max_ra_list = ra_list

        candidate_user_list = []
        for user, ra in zip(sorted_valid_user_list, max_ra_list):
            user.ra = ra
            if ra > 0:
                candidate_user_list.append(user)

        return candidate_user_list, max_ra_list

    def init_ua_ra_max_SINR(self, user_pool=None, isGBS=False):
        valid_user_list = self.get_valid_user()
        max_user = None
        max_pathloss = 9999
        for user in valid_user_list:
            if user.pathloss < max_pathloss:
                max_user = user
                max_pathloss = user.pathloss

        max_user.ra = 1
        return [max_user], [1]

    def init_ua_ra_opt(self, user_pool=None, isGBS=False):
        def ra_objective(sorted_valid_user_list, ra_list):
            if isGBS:
                return sum(
                    [
                        math.log(1 + ra * user.se_gbs / user.total_data)
                        if user.time_start <= self.current_time <= user.time_end
                        else 0
                        for user, ra in zip(sorted_valid_user_list, ra_list)
                    ]
                )
            else:
                return sum(
                    [
                        math.log(1 + ra * user.se / user.total_data)
                        if user.time_start <= self.current_time <= user.time_end
                        else 0
                        for user, ra in zip(sorted_valid_user_list, ra_list)
                    ]
                )

        def max_waterfilling(v, user):
            if isGBS:
                return max(
                    user.datarate / user.se_gbs, v - user.total_data / user.se_gbs
                )
            else:
                return max(user.datarate / user.se, v - user.total_data / user.se)

        def sum_bandwidth(v, user_list):
            return sum([max_waterfilling(v, user) for user in user_list]) - BANDWIDTH

        max_ra_list = []
        max_ua_list = []
        max_objective = -1e6

        valid_user_list = user_pool if isGBS else self.get_valid_user()
        for i in range(2 ** len(valid_user_list)):
            candidate_user_list = []
            for j, user in enumerate(valid_user_list):
                if i & 1 << j:
                    candidate_user_list.append(user)

            if (
                sum(
                    [
                        user.datarate / user.se_gbs
                        if isGBS
                        else user.datarate / user.se
                        for user in candidate_user_list
                    ]
                )
                > BANDWIDTH
            ):
                continue

            v_min = 0
            v_max = 10000
            while (
                abs(
                    sum_bandwidth(v_min, candidate_user_list)
                    - sum_bandwidth(v_max, candidate_user_list)
                )
                > 1e-4
            ):
                v_new = (v_min + v_max) / 2
                if sum_bandwidth(v_new, candidate_user_list) < 0:
                    v_min = v_new
                else:
                    v_max = v_new

            candidate_ra_list = [
                max_waterfilling(v_min, user) for user in candidate_user_list
            ]
            if max_objective < ra_objective(candidate_user_list, candidate_ra_list):
                max_objective = ra_objective(candidate_user_list, candidate_ra_list)
                max_ra_list = candidate_ra_list
                max_ua_list = candidate_user_list

        for user, ra in zip(max_ua_list, max_ra_list):
            user.ra = ra

        return max_ua_list, max_ra_list

    def init_ua_ra_local(self, user_pool=None, isGBS=False):
        def ra_objective(user_list, ra_list):
            if isGBS:
                return sum(
                    [
                        math.log(1 + ra * user.se_gbs / user.total_data)
                        if user.time_start <= self.current_time <= user.time_end
                        else 0
                        for user, ra in zip(user_list, ra_list)
                    ]
                )
            else:
                return sum(
                    [
                        math.log(1 + ra * user.se / user.total_data)
                        if user.time_start <= self.current_time <= user.time_end
                        else 0
                        for user, ra in zip(user_list, ra_list)
                    ]
                )

        def max_waterfilling(v, user):
            if isGBS:
                return max(
                    user.datarate / user.se_gbs, v - user.total_data / user.se_gbs
                )
            else:
                return max(user.datarate / user.se, v - user.total_data / user.se)

        def sum_bandwidth(v, user_list):
            return sum([max_waterfilling(v, user) for user in user_list]) - BANDWIDTH

        def ra_list(user_list):
            if user_list == []:
                return []
            v_min = 0
            v_max = 10000
            while (
                abs(sum_bandwidth(v_min, user_list) - sum_bandwidth(v_max, user_list))
                > 1e-4
            ):
                v_new = (v_min + v_max) / 2
                if sum_bandwidth(v_new, user_list) < 0:
                    v_min = v_new
                else:
                    v_max = v_new
            ra_list = [max_waterfilling(v_min, user) for user in user_list]
            return ra_list

        def recursive_ua(associated_user_list, candidate_user_list):
            max_user_list = associated_user_list
            max_ra_list = ra_list(associated_user_list)
            max_objective = ra_objective(max_user_list, max_ra_list)
            max_idx = None
            no_change = True
            for idx, user in enumerate(candidate_user_list):
                tmp_user_list = associated_user_list + [user]

                if (
                    sum(
                        [
                            user.datarate / user.se_gbs
                            if isGBS
                            else user.datarate / user.se
                            for user in tmp_user_list
                        ]
                    )
                    > BANDWIDTH
                ):
                    continue

                tmp_ra_list = ra_list(tmp_user_list)

                tmp_obj = ra_objective(tmp_user_list, tmp_ra_list)
                if max_objective < tmp_obj:
                    max_objective = tmp_obj
                    max_user_list = tmp_user_list
                    max_ra_list = tmp_ra_list
                    max_idx = idx

            if max_idx != None:
                no_change = False
                del candidate_user_list[max_idx]

            return max_user_list, max_ra_list, candidate_user_list, no_change

        associated_user_list = []
        candidate_user_list = user_pool if isGBS else self.get_valid_user()
        while True:
            (
                associated_user_list,
                associated_ra_list,
                candidate_user_list,
                no_change,
            ) = recursive_ua(associated_user_list, candidate_user_list)
            if no_change:
                break

        for user, ra in zip(associated_user_list, associated_ra_list):
            user.ra = ra

        return associated_user_list, associated_ra_list

    def kkt_ra(self, user_list):
        """
        Use projected adagrad for KKT dual problem
        """

        def regularized_gradient(lambda_1, lambda_2, mu_list, user_list):
            """
            return 2+len(user_list) length gradient
            """
            grad_list = []
            grad_lambda_1 = 1.0
            grad_lambda_2 = 1.0
            for i, user in enumerate(user_list):
                grad_lambda_1 += (
                    -1.0 / (lambda_1 + user.psd * lambda_2 - mu_list[i])
                    + user.total_data / user.se
                )
                grad_lambda_2 += (
                    -user.psd / (lambda_1 + user.psd * lambda_2 - mu_list[i])
                    + user.psd * user.total_data / user.se
                )
            grad_list.append(grad_lambda_1)
            grad_list.append(grad_lambda_2)

            for i, user in enumerate(user_list):
                grad_mu_i = (
                    1.0 / (lambda_1 + user.psd * lambda_2 - mu_list[i])
                    - user.datarate / user.se
                    - user.total_data / user.se
                )
                grad_list.append(grad_mu_i)
            return grad_list

        def regularized_dual_function(lambda_1, lambda_2, mu_list, user_list):
            value = 0.0
            for i, user in enumerate(user_list):
                if lambda_1 + user.psd * lambda_2 - mu_list[i] < 0:
                    print(user.id)
                    print(lambda_1 + user.psd * lambda_2 - mu_list[i])
                    print(lambda_1)
                    print(user.psd * lambda_2)
                    print(mu_list[i])
                value -= math.log(lambda_1 + user.psd * lambda_2 - mu_list[i])
                value -= mu_list[i] * (
                    user.datarate / user.se + user.total_data / user.se
                )
                value += lambda_1 * user.total_data / user.se
                value += lambda_2 * user.psd * user.total_data / user.se
            value += lambda_1
            value += lambda_2
            return value

        # Initialize arguments of the dual function
        lambda_1 = 0.5
        lambda_2 = 0.8
        mu_list = [0.5] * len(user_list)
        weight_list = [0] * (2 + len(mu_list))

        #        start = time.time()
        count = 0
        while True:
            count += 1
            # Save before update
            prev_lambda_1 = lambda_1
            prev_lambda_2 = lambda_2
            prev_mu_list = mu_list

            # RMSPROP update rule
            grad_list = regularized_gradient(lambda_1, lambda_2, mu_list, user_list)
            #            weight_list = [weight + grad_list[i]**2 for i, weight in enumerate(weight_list)]
            weight_list = [
                max(0.2, grad_list[i] ** 2) for i, weight in enumerate(weight_list)
            ]
            lambda_1 = max(
                EPSILON,
                lambda_1 - STEP_SIZE / (weight_list[0] + EPSILON) ** 0.5 * grad_list[0],
            )
            # Clipping: 1e-3
            lambda_2 = max(
                1e-5,
                lambda_2 - STEP_SIZE / (weight_list[1] + EPSILON) ** 0.5 * grad_list[1],
            )
            #            if len(user_list) != 0:
            #                print(f'{lambda_1+user_list[0].psd*lambda_2 - .1=}')
            mu_list = [
                min(
                    lambda_1 + user_list[idx].psd * lambda_2 - 1e-8,
                    max(
                        EPSILON,
                        mu
                        - 1.3
                        * STEP_SIZE
                        / (weight_list[2 + idx] + EPSILON) ** 0.5
                        * grad_list[2 + idx],
                    ),
                )
                for idx, mu in enumerate(mu_list)
            ]

            # Print option
            #            if count%2000==0 :
            #                print("Gradient:", grad_list)
            #                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
            #                print("Input:", [lambda_1, lambda_2]+mu_list )
            #                print("Count:",count)
            #                print("-------------------------")

            # Stop condition
            gap = regularized_dual_function(
                prev_lambda_1, prev_lambda_2, prev_mu_list, user_list
            ) - regularized_dual_function(lambda_1, lambda_2, mu_list, user_list)
            if gap < THRESHOLD or count > 1000:
                #                print("Gradient:", grad_list)
                #                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
                #                print("Input:", [lambda_1, lambda_2]+mu_list )
                #                print("Count:",count)
                #                print("-------------------------")
                break

        #        resource_list = [max(user.datarate/user.se, BANDWIDTH/(lambda_1+user.psd*lambda_2-mu_list[idx])-user.total_data/user.se) for idx, user in enumerate(user_list)]
        resource_list = [
            BANDWIDTH / (lambda_1 + user.psd * lambda_2 - mu_list[idx])
            - user.total_data / user.se
            for idx, user in enumerate(user_list)
        ]

        # Normalize the resource
        diff_list = [
            resource - user.datarate / user.se
            for resource, user in zip(resource_list, user_list)
        ]
        sum_diff_list = sum(diff_list)
        if sum_diff_list > EPSILON:
            available_bandwidth = BANDWIDTH - sum(
                [user.datarate / user.se for user in user_list]
            )
            resource_list = [
                user.datarate / user.se + available_bandwidth * diff / sum_diff_list
                for diff, user in zip(diff_list, user_list)
            ]
        else:
            sum_resource = sum(resource_list)
            resource_list = [
                BANDWIDTH * resource / sum_resource for resource in resource_list
            ]
        #        print("Normalized resource list, sum:", resource_list, sum(resource_list))
        #        print("self, self.parent:", self, self.parent)
        #        print("-------------------------")
        for user, resource in zip(user_list, resource_list):
            user.ra = resource

        return resource_list

    #
    #    def kkt_psd(self, user_list):
    #
    #        def max_waterfilling(v, user):
    #            return max((user.datarate/(user.ra*BANDWIDTH_ORIG)-1)**2/pl_over_noise

    def kkt_psd(self, user_list):
        lambda_list = []
        # 10^(-\xi/10)/n_0
        pl_over_noise_list = []
        c_rho_list = []
        for user in user_list:
            pl_over_noise = 10 ** ((-user.pathloss - NOISE_DENSITY) / 10.0)
            pl_over_noise_list.append(pl_over_noise)
            c_rho_list.append(
                (user.datarate / (user.ra * BANDWIDTH_ORIG) ** 2 - 1) / pl_over_noise
            )
            lambda_list.append(
                pl_over_noise
                / pow(2, user.datarate / (user.ra * BANDWIDTH_ORIG))
                / user.total_data
            )
        # return the sorted index list of the user lambda_list
        sorted_index = sorted(range(len(user_list)), key=lambda k: lambda_list[k])
        #        print(c_rho_list)

        max_objective_value = -99999
        max_psd_list = []
        max_sorted_index = []
        for i in range(len(user_list)):
            #        for i in range(1):
            term_1 = 0
            term_2 = 0
            term_3 = 0
            # Indexes of users such that lambda_psd < lambda_list[idx]
            # If lambda_psd < lambda_list[idx], mu_i = 0
            for idx in sorted_index[i:]:
                term_1 += user_list[idx].ra / user_list[idx].total_data
                term_2 += user_list[idx].ra / pl_over_noise_list[idx]
            # Indexes of users such that lambda_psd > lambda_list[idx]
            # If lambda_psd < lambda_list[idx], mu_i != 0
            for idx in sorted_index[:i]:
                term_3 += user_list[idx].ra * c_rho_list[idx]
            lambda_psd = term_1 / (POWER - term_3 + term_2)
            #            print("lambda psd:", lambda_psd)
            #            print("lambda list:", lambda_list)

            candidate_psd_list = c_rho_list[:]
            for idx in sorted_index[i:]:
                candidate_psd_list[idx] = (
                    1 / (user_list[idx].total_data * lambda_psd)
                    - 1 / pl_over_noise_list[idx]
                )

            # If this lambda_psd is valid
            sorted_lambda_list = [lambda_list[idx] for idx in sorted_index]
            #            print("slambda", sorted_lambda_list)
            #            print("comparison", sorted_lambda_list[i:], lambda_psd, sorted_lambda_list[:i])
            if all(
                lambda_i >= lambda_psd for lambda_i in sorted_lambda_list[i:]
            ) and all(lambda_i < lambda_psd for lambda_i in sorted_lambda_list[:i]):
                if not all(
                    diff >= 0
                    for diff in [b - a for a, b in zip(c_rho_list, candidate_psd_list)]
                ):
                    print(lambda_psd, lambda_list[i])
                    print("pl over noise list", pl_over_noise_list)
                    print("c rho list", c_rho_list)
                    print(candidate_psd_list, i)
                value = self.objective_function(candidate_psd_list, user_list)
                if max_objective_value < value:
                    max_psd_list = candidate_psd_list
                    max_objective_value = value

        for user, psd in zip(user_list, max_psd_list):
            user.psd = psd
            user.snr = self.psd2snr(user.psd, user.pathloss)
            user.se = self.snr2se(user.snr)
        #        print('Minimum power:',sum([user.ra*c_rho_list[idx] for idx,user in enumerate(user_list)]))
        #        print("user:",user_list)
        #        print("muzero user:",[ user_list[idx] for idx in sorted_index[i:]])
        #        print("orig:",[user.psd for user in user_list])
        #        print("psd :", candidate_psd_list)
        #        print("c_rho :", c_rho_list)
        #        print("diff:", [candidate_psd_list[a]-user.psd for a,user in enumerate(user_list)])
        #        print("candidate_psd_list:", candidate_psd_list)
        #        print(f'sum: {sum([user.ra*candidate_psd_list[idx] for idx,user in enumerate(user_list)]):04f}')
        #        print('----')

        return max_psd_list

    def objective_function(self, psd_list, user_list):
        if any(psd < 0 for psd in psd_list):
            print(psd_list)
            exit()

        value = 0
        for psd, user in zip(psd_list, user_list):
            snr = self.psd2snr(psd, user.pathloss)
            se = self.snr2se(snr)
            if 1 + user.ra * se / user.total_data < 0:
                print(
                    "psd, user.ra, se, user.total_data:",
                    psd,
                    user.ra,
                    se,
                    user.total_data,
                )
                print([user.ra for user in self.user_list])
            # user.ra = unit of 20MHz = 20 * unit of MHz
            value += math.log(1 + user.ra * se / user.total_data)
        return value  #


class TrajectoryTree:
    def __init__(
        self,
        root,
        vehicle_velocity,
        time_step,
        grid_size,
        map_width,
        min_altitude,
        max_altitude,
        tree_depth,
        num_node_iter,
        max_timeslot,
        gbs=None,
    ):
        self.root = root
        # Constants
        self.vehicle_velocity = vehicle_velocity
        self.time_step = time_step
        self.grid_size = grid_size
        self.map_width = map_width
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.tree_depth = tree_depth
        self.num_node_iter = num_node_iter
        self.max_timeslot = max_timeslot
        self.gbs = gbs

    # Depth First Search
    def DFS(self, current):
        # Recursive part
        if len(current.leafs) == 0:
            return [current], current.reward

        # Recursive here
        # Theorem : Subpath of optimal path is optimal of subpath.
        max_path = []
        # if reward return of self.DFS(node) MUST BE LARGER THAN -INF
        max_reward = -99999
        for i in range(len(current.leafs)):
            next_node = current.leafs[i]
            path, reward = self.DFS(next_node)
            if max_reward < reward:
                max_path = path
                max_reward = reward
        max_path.append(current)

        return max_path, max_reward + current.reward

    def recursive_find_leaf(self, leafs, node_level):
        # Terminate recursive function when it reaches to depth limit
        if node_level > self.tree_depth:
            # Save the leafs of DEPTH==TREE_DEPTH
            return
        node_level += 1
        # Find leafs of leaf
        for leaf in leafs:
            # Check whether leafs of leaf are already found.
            if len(leaf.leafs) == 0:
                leaf.leafs = self.find_leaf(leaf)
            self.recursive_find_leaf(leaf.leafs, node_level)

    def find_leaf(self, node):
        leafs = []
        appended_table = {}
        x = 0
        y = 0
        z = 0
        # loop for x
        while True:
            # initialize y before loop for y
            y = 0
            too_big_x = False
            # loop for y
            while True:
                # initialize z before loop for z
                z = 0
                too_big_y = False
                # loop for z
                while True:
                    # Check whether UAV can reach to adjacent grid node.
                    if (
                        self.grid_size * (x**2 + y**2 + z**2) ** 0.5
                        <= self.vehicle_velocity * self.time_step
                    ):
                        # add all node with distance |GRID_SIZE*(x,y,z)|^2_2
                        for i in {-1, 1}:
                            for j in {-1, 1}:
                                for k in {-1, 1}:
                                    # calculate leaf position
                                    leaf_position = [
                                        node_axis + self.grid_size * adjacent_axis
                                        for node_axis, adjacent_axis in zip(
                                            node.position, [x * i, y * j, z * k]
                                        )
                                    ]
                                    # Check whether 1. the position is available and 2. already appended.
                                    if (
                                        self.isAvailable(leaf_position)
                                        and tuple(leaf_position) not in appended_table
                                    ):
                                        leaf = TrajectoryNode(
                                            leaf_position,
                                            self.num_node_iter,
                                            parent=node,
                                            gbs=self.gbs,
                                        )
                                        leafs.append(leaf)
                                        appended_table[tuple(leaf_position)] = False
                        z += 1
                    else:
                        if z == 0:
                            too_big_y = True
                        break
                #### while z end
                if too_big_y:
                    if y == 0:
                        too_big_x = True
                    break
                y += 1
            #### while y end
            if too_big_x:
                break
            x += 1
        #### while x end
        return leafs

    def isAvailable(self, position):
        isAvailable = False
        # If the position is in the map, return true.
        isAvailable = (
            0 <= position[0] <= self.map_width
            and 0 <= position[1] <= self.map_width
            and self.min_altitude <= position[2] <= self.max_altitude
        )
        ############################################################
        # If there's any forbidden place in the map, write code here
        # isAvailable = isAvailable and (code here)
        ############################################################
        # otherwise return false.
        return isAvailable

    def pathfinder(self):
        path = []
        path.append(self.root)
        current = 0
        while current < self.max_timeslot:
            # DFS return = reversed path, reward
            # a.DFS(a.root)[0] = reversed path = [last leaf, ... , first leaf, root]
            # reversed path[-2] = first leaf = next root
            start = time.time()
            # Adjust node level according to the left time steps.
            # node_level -> tree_depth :
            # 1 -> tree depth,
            # 2 -> tree depth-1, ...
            node_level = (
                1
                if self.max_timeslot - current > self.tree_depth
                else self.tree_depth + 1 - (self.max_timeslot - current)
            )
            self.recursive_find_leaf([self.root], node_level)
            # sub_path = self.DFS(self.root)[0][:-1] -> [last leaf, ... , first leaf]
            # sub_path.reverse() -> [first leaf, ..., last leaf]
            sub_path = self.DFS(self.root)[0][:-1]
            sub_path.reverse()
            # Append path
            path = path + sub_path
            current += self.tree_depth - node_level + 1
            # Set new root
            self.root = path[-1]
            self.root.elapsed_time = time.time() - start
            # print(
            #     f"current step: {i}, reward: {self.root.reward:.2f}, elapsed time: {self.root.elapsed_time:.2f}",
            #     end="\r",
            #     flush=True,
            # )
            # print(
            #     f"current step: {current}, reward: {self.root.reward:.2f}, elapsed time: {self.root.elapsed_time:.2f}"
            # )
            # self.root.get_info()
        #        print('')
        return path


def fixed_path(
    user_list,
    map_width=600,
    min_altitude=50,
    max_altitude=200,
    max_timeslot=20,
    num_node_iter=0,
):
    path = []
    #    position = [random.randint(0, map_width)//10*10,
    #                 random.randint(0, map_width)//10*10,
    #                 random.randint(min_altitude, max_altitude)//10*10]
    position = [map_width / 2, map_width / 2, 75]
    # Initial grid position of UAV
    node = TrajectoryNode(position, num_node_iter)
    # Make user list
    node.user_list = user_list
    path.append(node)

    prev_node = node
    for time in range(1, max_timeslot):
        node = TrajectoryNode(position, num_node_iter, parent=path[-1])
        path.append(node)

    return path


def circular_path(
    radius,
    user_list,
    map_width=600,
    vehicle_velocity=15,
    max_timeslot=20,
    num_node_iter=0,
):
    path = []
    initial_angle = 2 * math.pi * random.random()
    x = map_width / 2 - radius * math.cos(initial_angle)
    y = map_width / 2 - radius * math.sin(initial_angle)
    # Initial grid position of UAV
    node = TrajectoryNode([x, y, 200], num_node_iter)
    node.user_list = user_list

    path.append(node)

    unit_angle = vehicle_velocity * max_timeslot / radius
    for time in range(1, max_timeslot):
        position = [
            x - radius * math.cos(unit_angle * time),
            y - radius * math.sin(unit_angle * time),
            75,
        ]
        node = TrajectoryNode(position, num_node_iter, parent=path[-1])
        path.append(node)

    return path


def random_path(
    user_list,
    map_width=600,
    min_altitude=50,
    max_altitude=200,
    max_timeslot=20,
    vehicle_velocity=15,
    time_step=3,
    num_node_iter=0,
):
    path = []
    position = [
        random.randint(0, map_width) // 10 * 10,
        random.randint(0, map_width) // 10 * 10,
        random.randint(min_altitude, max_altitude) // 10 * 10,
    ]
    # Initial grid position of UAV
    node = TrajectoryNode(position, num_node_iter)
    # Make user list
    node.user_list = user_list

    path.append(node)

    prev_node = node
    for time in range(1, max_timeslot):
        # spherical coordinate random position
        r = random.uniform(0, vehicle_velocity * time_step)
        theta = random.uniform(0, math.pi)
        pi = random.uniform(0, math.pi * 2)
        # Change of position from previous position
        position = [
            r * math.sin(theta) * math.cos(pi),
            r * math.sin(theta) * math.sin(pi),
            r * math.cos(theta),
        ]
        # Position of current node
        position = [a + b for a, b in zip(position, path[-1].position)]
        # Boundary check
        position[0] = max(0, min(map_width, position[0]))
        position[1] = max(0, min(map_width, position[1]))
        position[2] = max(min_altitude, min(max_altitude, position[2]))
        node = TrajectoryNode(position, num_node_iter, parent=path[-1])
        path.append(node)

    return path

    """
if __name__ =="__main__":
    # Constant for UAV
    VEHICLE_VELOCITY = 4. # m/s
    TIME_STEP = 1 # s
    MAX_TIMESLOT = 10 # unit of (TIME_STEP) s
    ## Constant for map
    MAP_WIDTH = 10 # meter, Both X and Y axis width
    MIN_ALTITUDE = 30 # meter
    MAX_ALTITUDE = 30 # meter
    GRID_SIZE = 1 # meter
    # Constant for user
    NUM_UE = 3
    NUM_NODE_ITER = 0
#    TIME_WINDOW_SIZE = [8, 8]
    TIME_WINDOW_SIZE = [4, 4]
#    TIME_WINDOW_SIZE = [2, 2]
#    TIME_WINDOW_SIZE = [1, 1]
    TIME_PERIOD_SIZE = [MAX_TIMESLOT, MAX_TIMESLOT]
    DATARATE_WINDOW = [10, 10] # Requiring datarate Mb/s
    INITIAL_DATA = 10 # Mb
    TREE_DEPTH = 4
    MAX_DATA = 99999999
    POWER_ORIG = 158

    pf_proposed = 0
    pf_circular = 0
    pf_fixed = 0
    pf_random = 0
    num_exp = 1
    avg_time = 0

    user_list = []
    for i in range(NUM_UE):
        tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
        time_period = random.randint(TIME_PERIOD_SIZE[0], TIME_PERIOD_SIZE[1])
        datarate = random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1])
        user = User(i, # id
                [random.randint(0, MAP_WIDTH), random.randint(0, MAP_WIDTH)], # position
                0, tw_size, time_period, # time window
                datarate, INITIAL_DATA, MAX_DATA) # data
        if i == 0:
            user.position = [0, 0]
        elif i == 1:
            user.position = [MAP_WIDTH, 0]
        else:
            user.position = [0, MAP_WIDTH]
            user.time_start = 6
            user.time_end = 10
            user.time_period = 4
        user_list.append(user)

    import copy
#    gbs = GroundBaseStation()
    for j in range(num_exp):
        position = [MAP_WIDTH//2,
                    MAP_WIDTH//2,
#                     random.randint(MIN_ALTITUDE, MAX_ALTITUDE)//10*10]
                    30]
        # Initial grid position of UAV
        root = TrajectoryNode(position, NUM_NODE_ITER)
        # Make user list
        root.user_list = copy.deepcopy(user_list)

        tree = TrajectoryTree(root, VEHICLE_VELOCITY,\
                                TIME_STEP, GRID_SIZE,\
                                MAP_WIDTH, MIN_ALTITUDE, MAX_ALTITUDE,\
                                TREE_DEPTH, NUM_NODE_ITER, MAX_TIMESLOT, gbs=None)
        start = time.time()
        PATH1 = tree.pathfinder()
        avg_time += time.time()-start
        user_list = PATH1[-1].user_list
        tmp_pf_proposed = sum([math.log(user.total_data-INITIAL_DATA) for user in user_list if user.total_data != INITIAL_DATA])
        tmp_sum_proposed = sum([user.total_data-INITIAL_DATA for user in user_list if user.total_data != INITIAL_DATA])
        pf_proposed += tmp_pf_proposed

    print(PATH1)
    print(f'DFS trajectory pf: {pf_proposed/num_exp}')
"""


if __name__ == "__main__":
    # Constant for UAV
    VEHICLE_VELOCITY = 15.0  # m/s
    TIME_STEP = 3  # s
    MAX_TIMESLOT = 20  # unit of (TIME_STEP) s
    ## Constant for map
    MAP_WIDTH = 600  # meter, Both X and Y axis width
    MIN_ALTITUDE = 50  # meter
    MAX_ALTITUDE = 200  # meter
    GRID_SIZE = 40  # meter
    # Constant for user
    NUM_UE = 10
    NUM_NODE_ITER = 0
    #    TIME_WINDOW_SIZE = [8, 8]
    TIME_WINDOW_SIZE = [4, 8]
    #    TIME_WINDOW_SIZE = [2, 2]
    #    TIME_WINDOW_SIZE = [1, 1]
    TIME_PERIOD_SIZE = [MAX_TIMESLOT, MAX_TIMESLOT]
    DATARATE_WINDOW = [5, 5]  # Requiring datarate Mb/s
    INITIAL_DATA = 10  # Mb
    TREE_DEPTH = 1
    MAX_DATA = 99999999

    pf_proposed = 0
    pf_circular = 0
    pf_fixed = 0
    pf_random = 0
    num_exp = 100
    avg_time = 0

    pl_proposed = 0
    service_count_proposed = 0
    pl_fixed = 0
    service_count_fixed = 0

    import copy

    #    gbs = GroundBaseStation()
    for j in range(num_exp):
        position = [
            random.randint(0, MAP_WIDTH) // 10 * 10,
            random.randint(0, MAP_WIDTH) // 10 * 10,
            random.randint(MIN_ALTITUDE, MAX_ALTITUDE) // 10 * 10,
        ]
        # Initial grid position of UAV
        root = TrajectoryNode(position, NUM_NODE_ITER)
        # Make user list
        user_list = []
        for i in range(NUM_UE):
            tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
            time_period = random.randint(TIME_PERIOD_SIZE[0], TIME_PERIOD_SIZE[1])
            datarate = random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1])
            user = User(
                i,  # id
                [
                    random.randint(0, MAP_WIDTH) // 10 * 10,
                    random.randint(0, MAP_WIDTH) // 10 * 10,
                ],  # position
                random.randint(0, MAX_TIMESLOT - tw_size),
                tw_size,
                time_period,  # time window
                datarate,
                INITIAL_DATA,
                MAX_DATA,
            )  # data
            user_list.append(user)
        root.user_list = copy.deepcopy(user_list)

        tree = TrajectoryTree(
            root,
            VEHICLE_VELOCITY,
            TIME_STEP,
            GRID_SIZE,
            MAP_WIDTH,
            MIN_ALTITUDE,
            MAX_ALTITUDE,
            TREE_DEPTH,
            NUM_NODE_ITER,
            MAX_TIMESLOT,
            gbs=None,
        )
        start = time.time()
        PATH1 = tree.pathfinder()
        avg_time += time.time() - start
        user_list = PATH1[-1].user_list
        tmp_pf_proposed = sum(
            [
                math.log(user.total_data - INITIAL_DATA)
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        tmp_sum_proposed = sum(
            [
                user.total_data - INITIAL_DATA
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        pf_proposed += tmp_pf_proposed

        # sum_reward = 0
        # for node in PATH1[1:]:
        #     sum_reward += node.reward
        #     for user in node.serviced_ua_list:
        #         pl_proposed += user.pathloss
        #     service_count_proposed += len(node.serviced_ua_list)
        # print(sum_reward, pf_proposed)
        # exit()

        for user in user_list:
            user.total_data = INITIAL_DATA
        PATH2 = circular_path(100, copy.deepcopy(user_list))
        user_list = PATH2[-1].user_list
        tmp_pf_circular = sum(
            [
                math.log(user.total_data - INITIAL_DATA)
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        tmp_sum_circular = sum(
            [
                user.total_data - INITIAL_DATA
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        pf_circular += tmp_pf_circular

        for user in user_list:
            user.total_data = INITIAL_DATA
        PATH3 = random_path(copy.deepcopy(user_list))
        user_list = PATH3[-1].user_list
        tmp_pf_random = sum(
            [
                math.log(user.total_data - INITIAL_DATA)
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        tmp_sum_random = sum(
            [
                user.total_data - INITIAL_DATA
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        pf_random += tmp_pf_random

        for user in user_list:
            user.total_data = INITIAL_DATA
        PATH4 = fixed_path(copy.deepcopy(user_list), MAP_WIDTH)
        user_list = PATH4[-1].user_list
        tmp_pf_fixed = sum(
            [
                math.log(user.total_data - INITIAL_DATA)
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        tmp_sum_fixed = sum(
            [
                user.total_data - INITIAL_DATA
                for user in user_list
                if user.total_data != INITIAL_DATA
            ]
        )
        pf_fixed += tmp_pf_fixed

        for node in PATH4:
            for user in node.serviced_ua_list:
                pl_fixed += user.pathloss
            service_count_fixed += len(node.serviced_ua_list)

        print(
            f"Iteration: {j}, Proposed: {tmp_pf_proposed: .2f}, Circular: {tmp_pf_circular: .2f}, random: {tmp_pf_random: .2f}, fixed: {tmp_pf_fixed: .2f}"
        )
    #        print(f'Iteration: {j}, Proposed: {tmp_sum_proposed: .2f}, Circular: {tmp_sum_circular: .2f}, random: {tmp_sum_random: .2f}, fixed: {tmp_sum_fixed: .2f}')
    print(f"DFS trajectory pf: {pf_proposed/num_exp}")
    print(f"Circular trajectory pf: {pf_circular/num_exp}")
    print(f"Random trajectory pf: {pf_random/num_exp}")
    print(f"Fixed trajectory pf: {pf_fixed/num_exp}")
