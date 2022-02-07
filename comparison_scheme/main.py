#!/usr/bin/env python

# Author : Hyeonsu Lyu, POSTECH, Korea
# Original paper: Zeng et al., Trajectory Optimization and Resource Allocation for OFDMA UAV Relay Networks, IEEE Transactions on Wireless Communication, 2021

import system_model
import cvxpy as cp
import math

param = system_model.param

if __name__=="__main__":
    UAV = system_model.Basestation(system_model.Position(cp.Variable(name='x'),130,50))

    list_user = []
    for i in range(5):
        list_user.append(system_model.Device(1,system_model.Position(4*i,13*i,0),[system_model.Subchannel(1+1.5e-5*j,1,0) for j in range(param.num_subchannels)],0.1))

#    for user in list_user:
#        print(user.calc_pathloss(UAV.position))
#        print(user.approx_los_prob(UAV.position))
