#!/usr/bin/env python

# Author : Hyeonsu Lyu, POSTECH, Korea
# Original paper: Zeng et al., Trajectory Optimization and Resource Allocation for OFDMA UAV Relay Networks, IEEE Transactions on Wireless Communication, 2021

from dataclasses import dataclass
from typing import List

import math
import random
import numpy as np
from scipy.stats import rice

import os
import json
import argparse
import pickle

@dataclass
class Parameters:
    # Some numbers
    num_ue: int=10
    num_subcarriers: int=10
    num_timeslots: int=60
    # Map
    map_width: int=400 # meter
    min_altitude: int=30 # meter
    max_altitude: int=250 # meter
    # Time, velocity
    time_duration: float=1 #s
    uav_max_dist: float=15 #m
    # Channel parameters
    pathloss_g2g_alpha: float=4
    pathloss_a2g_a: float=9.6
    pathloss_a2g_b: float=0.28
    pathloss_excessive_LoS: float=1 # dB
    pathloss_excessive_NLoS: float=40 # dB
    # Time period
    time_window_size = 4
    time_start_range = [0, num_timeslots-time_window_size]
    # Communication parameters
    SNR_threshold: float=0
    noise: float=-121.45 # dBm
    ICI: float=-110 # dBm
    frequency: float=2 # GHz (Normalized)
    subcarrier_bandwidth: float=0.18 # MHz = 180 KHz
    lightspeed: float=3e-1 # 1e9 m/s (Normalized)
    max_ue_power_mW: int=200 # mW
    max_uav_power_mW: int=200 # mW
    max_ue_power_dB: float=10*math.log10(max_ue_power_mW) # dBm
    max_uav_power_dB: float=10*math.log10(max_uav_power_mW) # dBm
    # rician_k from paper: Air–Ground Channel Characterization for Unmanned Aircraft Systems-
    # Part III: The Suburban and Near-Urban Environments
    rician_K: float= 11.4 # dB

    # iteration threshold
    epsilon_TO: float=0.01
    epsilon_JMSTP: float=0.001

param = Parameters()

@dataclass
class Subcarrier:
    frequency: float=param.frequency
    alpha: int=0
    pathloss: float=0
    power: float=0. # Power per subcarrier
    channel: float=0.

@dataclass
class Position:
    x: float=0.
    y: float=0.
    z: float=0.

    def __add__(self, other):
        return Position(self.x+other.x, self.y+other.y, self.z+other.z)

    def __sub__(self, other):
        return Position(self.x-other.x, self.y-other.y, self.z-other.z)

    def l2_norm(self, other = None):
        if other is None:
            return (self.x**2+self.y**2+self.z**2)**.5
        if other is not None:
            return (self-other).l2_norm()

class Device:
#    position: Position = Position()
#    list_subcarrier_UBS: List[Subcarrier] = None
#    list_subcarrier_GBS: List[Subcarrier] = None
#    mode: int = 0
#    los_prob: float = 0
#    serviced_data: float=1

    def __init__(self, id: int, position: Position=Position(),
            list_subcarrier_UBS: List[Subcarrier]=None, list_subcarrier_GBS: List[Subcarrier]=None,
            mode: int=0, time_start:int=0, tw_size:int=0):
        self.id = id
        self.position = position
        self.list_subcarrier_UBS = list_subcarrier_UBS
        self.list_subcarrier_GBS = list_subcarrier_GBS
        self.time_start = time_start
        self.time_end = self.time_start + tw_size
        self.serviced_time = 0
        self.mode = mode
        self.los_prob = 0
        self.serviced_data = 10

    def __repr__(self):
        return "{}".format(self.id)#}}}

@dataclass
class GroundBasestation:
    position: Position = Position(param.map_width/2, param.map_width/2, 30)
    list_ue: List[Device] = None

    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump(self, f)

    def calc_channel(self):
        for ue in self.list_ue:
            self._calc_channel(ue)

    def _calc_channel(self, user: Device):
        for subcarrier in user.list_subcarrier_GBS:
            subcarrier.pathloss = 10 * math.log10( self.position.l2_norm(user.position)**(param.pathloss_g2g_alpha) )
            subcarrier.channel = 10 * math.log10( random.gauss(0,1)**2 ) - subcarrier.pathloss

@dataclass
class UAVBasestation:
    prev_position: Position = Position()
    position: Position = Position()
    list_ue: List[Device] = None
    list_subcarrier: List[Subcarrier] = None
    GBS: GroundBasestation = None
    gbs_channel: float = 0
    gbs_los_prob: float = 0

    def save(self, path):
        with open(path, 'wb') as f:
            pickle.dump(self, f)

    def calc_pathloss(self):
        for ue in self.list_ue:
            self._calc_pathloss(ue)
        self._calc_pathloss(self.GBS)

    def calc_channel(self):
        for ue in self.list_ue:
            self._calc_channel(ue)
        self._calc_channel(self.GBS)

    def _calc_pathloss(self, user: Device):
        """
            Calculate pathlosses of user's subcarriers.
        """
#        cplog = lambda x: cp.log(x)/cp.log(10)
#        # Different operand required when one of members in self.position is cp.Variable
#        log = cp.log if any( [isinstance(ax[1], cp.Variable) for ax in vars(self.position).items()]) else math.log10
        log=math.log10

        if isinstance(user, Device):
            los_prob = self.los_prob(user.position)
            user.los_prob = los_prob
            list_subcarrier = user.list_subcarrier_UBS
        elif isinstance(user, GroundBasestation):
            los_prob = self.los_prob(user.position)
            self.gbs_los_prob = los_prob
            list_subcarrier = self.list_subcarrier

        for subcarrier in list_subcarrier:
            # fspl: Free space propagation loss
            fspl = 20 * log(4 * math.pi * subcarrier.frequency / param.lightspeed) \
                     + 20 * log(self.position.l2_norm(user.position))

#            avg_excessive_loss = user.los_prob * param.pathloss_excessive_LoS + \
#                                (1 - user.los_prob) * param.pathloss_excessive_NLoS
            avg_excessive_loss = param.pathloss_excessive_NLoS + (param.pathloss_excessive_LoS - param.pathloss_excessive_NLoS) * los_prob

            subcarrier.pathloss = fspl + avg_excessive_loss
        return

    def _calc_channel(self, user: Device):
        """
            Calculate channels of user's subcarriers.
            Pathlosses of the subcarriers should be initialized.
        """
        list_subcarrier = user.list_subcarrier_UBS if isinstance(user, Device) else self.list_subcarrier
        for subcarrier in list_subcarrier:
            los_real = np.random.uniform(low=0.0, high=1.0)
            los_im = (1-los_real**2)**.5
            los_real *= (param.rician_K/(param.rician_K+1))**.5
            los_im *= (param.rician_K/(param.rician_K+1))**.5
            nlos_phase = (1/(param.rician_K+1))**.5 * np.random.multivariate_normal(np.zeros(2), 0.5*np.eye(2), size=1)[0]
            nlos_real = nlos_phase[0]
            nlos_im = nlos_phase[1]
            fading = ( (los_real+nlos_real)**2 + (los_im+nlos_im)**2)*.5
            subcarrier.channel = 10*math.log10(fading) - subcarrier.pathloss
#            print(10*math.log10(fading) - subcarrier.pathloss)
    
    def los_prob(self, pos_other: Position):
#        if any( [isinstance(ax[1], cp.Variable) for ax in vars(self.position).items()]):
#            print("self.position must be real value, not variable.")
#            print("If you want to put cp.Variable at los_prob, please use approx_los_prob.")

        a = param.pathloss_a2g_a
        b = param.pathloss_a2g_b

        return 1 / (1 + a * math.exp( -b * (180 / math.pi * math.asin( (self.position.z - pos_other.z) / self.position.l2_norm(pos_other)) - a)))

    def approx_los_prob(self, pos_other: Position):

        def approx_theta():
            # C^{j-1} at paper at eq. (45)
            C = self.prev_position.l2_norm(pos_other) / abs(self.prev_position.z - pos_other.z)

            return 180 / math.pi * ( math.asin(1/C) - 1 / (C * (C**2-1)**.5) * ( self.position.l2_norm(pos_other) / abs(self.position.z - pos_other.z) - C) )

#        exp = cp.exp if any( [isinstance(ax[1], cp.Variable) for ax in vars(self.position).items()]) else math.exp
        exp = math.exp
        tmp = approx_theta()

        a = param.pathloss_a2g_a
        b = param.pathloss_a2g_b
        prev_theta = 180 / math.pi * math.asin( (self.prev_position.z - pos_other.z) / self.prev_position.l2_norm(pos_other) )
        D = 1 + a * math.exp( -b * (prev_theta - a) )
        
        return 2 / D - 1 / D**2 - a / D**2 * exp( -b * (approx_theta() - a))

def initialize_users(path: str=None):
    if path==None:
        list_ue = []
        # Make default subcarrier
        for i in range(param.num_ue):
            position = Position(random.randint(0, param.map_width), random.randint(0, param.map_width), 0)
            list_subcarrier_UBS = [Subcarrier(param.frequency + 1.5e-5 * i) for i in range(param.num_subcarriers)]
            list_subcarrier_GBS = [Subcarrier(param.frequency + 1.5e-5 * i) for i in range(param.num_subcarriers)]
            ue = Device(i, position, list_subcarrier_UBS, list_subcarrier_GBS, random.randint(0,1), random.randint(param.time_start_range[0], param.time_start_range[1]), param.time_window_size)
            list_ue.append(ue)
    else:
        with open(path) as f:
            env = json.load(f)
            user_dict_list = env['user_list']
            list_ue = []
            # Make default subcarrier
            for user_dict in user_dict_list[0:param.num_ue]:
                position = Position(*user_dict['position'], 0)
                list_subcarrier_UBS = [Subcarrier(param.frequency + 1.5e-5 * i) for i in range(param.num_subcarriers)]
                list_subcarrier_GBS = [Subcarrier(param.frequency + 1.5e-5 * i) for i in range(param.num_subcarriers)]
                ue = Device(user_dict['id'], position, 
                        list_subcarrier_UBS, list_subcarrier_GBS, 0, 
                        user_dict['time_start'], user_dict['tw_size'])
                list_ue.append(ue)

    return list_ue

def initialize_network(path: str=None):
    if path == None:
        prev_position = Position(random.randint(0, param.map_width), random.randint(0, param.map_width), 50)
        position = Position(random.randint(0, param.map_width), random.randint(0, param.map_width), 50)
    #    position = Position(random.randint(0, param.map_width), cp.Variable(pos=True, name='x'), 49)
    #    is_variable = any( [isinstance(ax[1], cp.Variable) for ax in vars(position).items()])

        list_ue = initialize_users()
    else:
        with open(path) as f:
            env = json.load(f)
            prev_position = Position(*env['root_position'])
            position = prev_position
        list_ue = initialize_users(path)

    list_subcarrier = [Subcarrier(param.frequency + 1.5e-5 * i) for i in range(param.num_subcarriers)]
    GBS = GroundBasestation(Position(param.map_width/2, param.map_width/2, 10), list_ue)
    UBS = UAVBasestation(prev_position, position, list_ue, list_subcarrier, GBS = GBS)
    UBS.calc_pathloss()
    UBS.calc_channel()
    GBS.calc_channel()
#    print(list_ue[1].__dict__)
    return UBS, GBS, list_ue


if __name__=="__main__":
    
    def los_prob_approximation_test():
        UAV = UAVBasestation(Position(10,30,50), Position(0,30,50))

        for i in range(10):
            print('------')
            pos_other = Position(10*i, 25*i, 1)
            print(UAV.approx_los_prob(pos_other))
            print(UAV.los_prob(pos_other))
        print('------')

    def initialization_validity_check(cvx_mode = False):
#        log = lambda x: cp.log(x) / cp.log(10) if cvx_mode else math.log10
        log = math.log10

        print("----UAV status----")
        print(f'{UBS.position = }')
        print(f'{UBS.prev_position = }')
        print(f'{UBS.gbs_los_prob = }')
        for subcarrier in UBS.list_subcarrier:
            print(f'\t\t{subcarrier.frequency = } (GHz)')
            print(f'\t\t{subcarrier.power = } (dB)')
            print(f'\t\t{subcarrier.pathloss = } (dB)')
            print(f'\t\t{subcarrier.channel = } (dB)')
            print('\t\t----------')

        print("----GBS status----")
        print(f'{GBS.position = }')

        print("----User status----")
        for user in list_ue: 
            print(f'\n\t{user.position = }')
            print(f'\t{user.mode = }')
            print(f'\t{user.los_prob = }')
            print('\t---------- subcarrier_UBS')

            for subcarrier in user.list_subcarrier_UBS:
                print(f'\t\t{subcarrier.alpha = } ')
                print(f'\t\t{subcarrier.frequency = } (GHz)')
                print(f'\t\t{subcarrier.power = } (dB)')
                print(f'\t\t{subcarrier.pathloss = } (dB)')
                print(f'\t\t{subcarrier.channel = } (dB)')
                print('\t\t----------')

            print('\t---------- subcarrier_GBS')
            for subcarrier in user.list_subcarrier_GBS:
                print(f'\t\t{subcarrier.alpha = } ')
                print(f'\t\t{subcarrier.frequency = } (GHz)')
                print(f'\t\t{subcarrier.power = } (dB)')
                print(f'\t\t{subcarrier.pathloss = } (dB)')
                print(f'\t\t{subcarrier.channel = } (dB)')
                print('\t\t----------')

    param.num_ue = 2
    param.num_subcarriers = 2

    UBS, GBS, list_ue = initialize_network('/home/hslyu/dbspf/data/env/env_0000.json')
#    UBS, GBS, list_ue = initialize_network()
#    los_prob_approximation_test()
    initialization_validity_check()
