#!/usr/bin/env python

# Author : Hyeonsu Lyu, POSTECH, Korea
# Original paper: Zeng et al., Trajectory Optimization and Resource Allocation for OFDMA UAV Relay Networks, IEEE Transactions on Wireless Communication, 2021

from dataclasses import dataclass
from typing import List

import math
import random
import cvxpy as cp
from scipy.stats import rice

@dataclass
class Parameters:
    # Some numbers
    num_ue: int=5
    num_subcarriers: int=10
    num_timeslots: int=10
    # Map
    map_width: int=200 # meter
    # Time, velocity
    time_duration: float=1 #s
    uav_max_dist: float=15 #m
    # Channel parameters
    pathloss_g2g_alpha: float=4
    pathloss_a2g_a: float=9.6
    pathloss_a2g_b: float=0.28
    pathloss_excessive_LoS: float=1 # dB
    pathloss_excessive_NLoS: float=20 # dB
    # Communication parameters
    SNR_threshold: float=300
    noise: float=-96 # dBm
    ICI: float=-110 # dBm
    frequency: float=2 # GHz (Normalized)
    lightspeed: float=3e-1 # 1e9 m/s (Normalized)
    max_ue_power: float=24.77 # dBm
    max_uav_power: float=17 # dBm
    # rician_k from paper: Air–Ground Channel Characterization for Unmanned Aircraft Systems-
    # Part III: The Suburban and Near-Urban Environments
    rician_K: float= 12.4 # dB

    # iteration threshold
    epsilon_TO: float=0.01
    epsilon_JMSTP: float=0.001

param = Parameters()

@dataclass
class Subcarrier:
    frequency: float=param.frequency
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

    def l2_norm(self, other=None):
        if other is None:
            return (self.x**2+self.y**2+self.z**2)**.5
        if other is not None:
            return (self-other).l2_norm()

@dataclass
class Device:
    position: Position = Position()
    list_subcarrier: List[Subcarrier] = None
    mode: int = 0
    alpha: list[int] = None
    los_prob: float = 0
    serviced_data: float=.1

@dataclass
class UAVBasestation:
    prev_position: Position=Position()
    position: Position=Position()
    list_ue: List[Device]=None

    def calc_pathloss(self, user: Device):
        """
            Calculate pathlosses of user's subcarriers.
        """
        cplog = lambda x: cp.log(x)/cp.log(10)
        # Different operand required when one of members in self.position is cp.Variable
        log = cp.log if any( [isinstance(ax[1], cp.Variable) for ax in vars(self.position).items()]) else math.log10

        for i, subcarrier in enumerate(user.list_subcarrier):
            # fspl: Free space propagation loss
            fspl = 20 * log(4 * math.pi * subcarrier.frequency / param.lightspeed) \
                     + 20 * log(self.position.l2_norm(user.position))

#            avg_excessive_loss = user.los_prob * param.pathloss_excessive_LoS + \
#                                (1 - user.los_prob) * param.pathloss_excessive_NLoS
            avg_excessive_loss = param.pathloss_excessive_NLoS + (param.pathloss_excessive_LoS - param.pathloss_excessive_NLoS) * user.los_prob

            subcarrier.pathloss = fspl + avg_excessive_loss
        return

    def calc_channel(self, user: Device):
        """
            Calculate channels of user's subcarriers.
            Pathlosses of the subcarriers should be initialized.
        """
        for subcarrier in user.list_subcarrier:
            pathloss = 10**(subcarrier.pathloss / 10)
            subcarrier.channel = rice.rvs( 10**(param.rician_K / 10) ) / pathloss
    
    def los_prob(self, pos_other: Position):
        if any( [isinstance(ax[1], cp.Variable) for ax in vars(self.position).items()]):
            print("self.position must be real value, not variable.")
            print("If you want to put cp.Variable at los_prob, please use approx_los_prob.")

        a = param.pathloss_a2g_a
        b = param.pathloss_a2g_b

        return 1 / (1 + a * math.exp( -b * (180 / math.pi * math.asin( (self.position.z - pos_other.z) / self.position.l2_norm(pos_other)) - a)))

    def approx_los_prob(self, pos_other: Position):

        def approx_theta():
            # C^{j-1} at paper at eq. (45)
            C = self.prev_position.l2_norm(pos_other) / abs(self.prev_position.z - pos_other.z)

            return 180 / math.pi * ( math.asin(1/C) - 1 / (C * (C**2-1)**.5) * ( self.position.l2_norm(pos_other) / abs(self.position.z - pos_other.z) - C) )

        exp = cp.exp if any( [isinstance(ax[1], cp.Variable) for ax in vars(self.position).items()]) else math.exp
        tmp = approx_theta()

        a = param.pathloss_a2g_a
        b = param.pathloss_a2g_b
        prev_theta = 180 / math.pi * math.asin( (self.prev_position.z - pos_other.z) / self.prev_position.l2_norm(pos_other) )
        D = 1 + a * math.exp( -b * (prev_theta - a) )
        
        return 2 / D - 1 / D**2 - a / D**2 * exp( -b * (approx_theta() - a))

def initialize_users():
    list_ue = []
    # Make default subcarrier
    for _ in range(param.num_ue):
        position = Position(random.randint(0, param.map_width), random.randint(0, param.map_width), 0)
        list_subcarrier = [Subcarrier(param.frequency + 1.5e-5 * i) for i in range(param.num_subcarriers)]

        ue = Device(position, list_subcarrier, random.randint(0,1), [random.randint(0,1) for _ in range(param.num_subcarriers)])
        list_ue.append(ue)

    return list_ue

def initialize_UAV():
    prev_position = Position(random.randint(0, param.map_width), random.randint(0, param.map_width), 50)
    position = Position(random.randint(0, param.map_width), cp.Variable(pos=True, name='x'), 50)
    is_variable = any( [isinstance(ax[1], cp.Variable) for ax in vars(position).items()])

    list_ue = initialize_users()

    UAVBS = UAVBasestation(prev_position, position, list_ue)
    for ue in UAVBS.list_ue:
        # los_prob is quasiconvex
        ue.los_prob = UAVBS.approx_los_prob(ue.position) if is_variable else UAVBS.los_prob(ue.position) 
        UAVBS.calc_pathloss(ue)
#        UAVBS.calc_channel(ue)

    return UAVBS, list_ue

def horizontal_optimization(list_user: list[Device], UAV: UAVBasestation):
    # 0 <= x, y <= param.map_width
    UAV.position.x = cp.Variable(pos=True, name='x')
    UAV.position.y = cp.Variable(pos=True, name='y')

    #  design eq (47)
    #  design eq (48)

    return


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
        log = lambda x: cp.log(x) / cp.log(10) if cvx_mode else math.log10

        print("----UAV status----")
        print(f'{UAVBS.position = }')
        print(f'{UAVBS.prev_position = }')
        print("----User status----")
        for user in UAVBS.list_ue: 
            print(f'\t{user.position = }')
            print(f'\t{user.mode = }')
            print(f'\t{user.los_prob = }')
            print('\t----------')

            for i, subcarrier in enumerate(user.list_subcarrier):
                print(f'\t\t{user.alpha[i] = }')
                print(f'\t\t{subcarrier.frequency = } (GHz)')
                print(f'\t\t{subcarrier.power = } (dB)')
                print(f'\t\t{subcarrier.pathloss = } (dB)')
#                print(f'\t\t{10*log(subcarrier.channel) = } (dB)')
                print('\t\t----------')

    UAVBS, list_ue = initialize_UAV()
#    los_prob_approximation_test()
    initialization_validity_check(True)
