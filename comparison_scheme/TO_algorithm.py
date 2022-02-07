#!/usr/bin/env python

# Author : Hyeonsu Lyu, POSTECH, Korea
# Original paper: Zeng et al., Trajectory Optimization and Resource Allocation for OFDMA UAV Relay Networks, IEEE Transactions on Wireless Communication, 2021

import system_model
import cvxpy as cp
import math

param = system_model.param

def horizontal_optimization(list_user: list[system_model.Device], UAV: system_model.Device):


    return calc_pathloss(False)

#def vertical_optimization()
#
#def trajectory_optimization(list_user: list[system_model.Device], UAV: Device):
#    """
#        Successively optimizing the trajectory.
#    Params:
#        list_user (list[system_model.Device]): list of users.
#        UAV (system_model.Device): Information about UAV
#    returns:
#        position (system_model.Position): Next position of UAV
#    """
#
#    
#    while 
#
#    return Position(x_opt, y_opt, z_opt)


