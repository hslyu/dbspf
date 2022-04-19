import system_model as sm
import math
import numpy as np
from geneticalgorithm import geneticalgorithm as ga

param = sm.param
def dB2orig(var_dB: float=0):
    return 10**(var_dB/10)

def softmax_array(arr: np.ndarray):
    y = np.exp(arr - np.max(arr))
    regularized_arr = y / ( np.sum(np.exp(arr-np.max(arr))) + 1e-10)
    return regularized_arr

def print_gene_expression(list_ue_mode, list_ue_alpha, list_ue_power, \
                          list_UBS_power, position_x, position_y, position_z):
    print(f'{list_ue_mode = }')
    for i in range(param.num_ue): 
        print(f'sum(list_ue_alpha[{i}]) : {sum(list_ue_alpha[i]) : 4.2f}')
    for i in range(param.num_ue):
        print(f'list_ue_power[{i}] : {sum(list_ue_power[i]) : 4.2f}')
    print(f'{sum(list_UBS_power) = :4.2f}')
    print(f'position : ({position_x:4.2f}, {position_y:4.2f}, {position_z:4.2f})')

def split_ga_vector(X: np.ndarray):
    a = param.num_ue
    b = a + param.num_subcarriers
    c = b + param.num_ue * param.num_subcarriers
    d = c + param.num_subcarriers

    list_ue_mode = X[:a] 
    list_ue_alpha_encoded = X[a:b]
    list_ue_alpha = (np.arange(param.num_ue).reshape([-1,1]) == list_ue_alpha_encoded).astype(int)

    list_ue_power_encoded = X[b:c] # before softmax
    list_ue_power_encoded = list_ue_power_encoded.reshape((param.num_ue,-1))
    list_ue_power = np.apply_along_axis(softmax_array, 1, list_ue_power_encoded) * param.max_ue_power_mW

    list_UBS_power_encoded = X[c:d]
    list_UBS_power = softmax_array(list_UBS_power_encoded) * param.max_uav_power_mW

    theta = math.radians(X[d] * 5)
    pi = math.radians(X[d + 1] * 5)
    radius = param.uav_max_dist * X[d + 2] / 100
    position_x = UBS.prev_position.x + radius * math.sin(theta) * math.cos(pi)
    position_y = UBS.prev_position.y + radius * math.sin(theta) * math.sin(pi)
    position_z = UBS.prev_position.z + radius * math.cos(pi)
#    print(f'Position differences: {radius} {radius * math.sin(theta) * math.cos(pi)}, {radius * math.sin(theta) * math.sin(pi)}, {radius * math.cos(pi)}')

    return list_ue_mode, list_ue_alpha, list_ue_power, list_UBS_power, position_x, position_y, position_z

def objective_function(X: np.ndarray):
    penalty = 0
    # split vector
    # list_ue_mode - size : param.num_ue
    # list_ue_{alpha, power} - size : param.num_ue * param.num_subcarriers
    # list_UBS_power - size : param.num_subcarriers
    # position_{x,y,z} - size : 1
    list_ue_mode, list_ue_alpha, list_ue_power, list_UBS_power, position_x, position_y, position_z = split_ga_vector(X)
#    print_gene_expression(list_ue_mode, list_ue_alpha, list_ue_power, \
#                          list_UBS_power, position_x, position_y, position_z)

    """
    # eq (35b) penalty function : UBS power
    if sum(list_UBS_power) < param.max_uav_power_mW:
        return -99999

    # eq (35d) penalty function : each subcarrier is occupied by at most 1 ue
    list_sum_subcarrier = [ sum([ list_ue_alpha[i, j] for i in range(param.num_ue)]) for j in range(param.num_subcarriers)]
    for sum_subcarrier in list_sum_subcarrier:
        if sum_subcarrier > 1:
            penalty += 12345 / param.num_ue
    if not all(sum_subcarrier <= 1 for sum_subcarrier in list_sum_subcarrier):
        return -penalty

    # eq (35c) penalty function : UE power
    list_ue_sum_power = [sum([ alpha*power for alpha, power in zip(list_ue_alpha[i], list_ue_power[i]) ]) for i in range(param.num_ue)]
    for sum_power in list_ue_sum_power:
        if sum_power < param.max_ue_power_mW:
            penalty += 99999 / param.num_ue
#    if not all(sum_power <= param.max_ue_power_mW for sum_power in list_ue_sum_power):
#        return -penalty

    
    """
    UBS.position = sm.Position(position_x, position_y, position_z)
    # eq (35e) penality function
    if UBS.position.l2_norm(UBS.prev_position) > param.uav_max_dist:
        penalty += 10 * UBS.position.l2_norm(UBS.prev_position)

    UBS.calc_pathloss() 
    UBS.calc_channel() 

    list_rate = []
    for i, ue in enumerate(list_ue):
        # Find sum-rate 
        sumrate = 0
        if list_ue_mode[i] == 1:
            for j, subcarrier in enumerate(ue.list_subcarrier_UBS):
                if list_ue_alpha[i,j] == 1:
                    carrier_rate = 0
                    # R^k_{n,R} in paper. eq (31)
                    power_ue_UBS = list_ue_power[i, j] * 1e-3 * dB2orig(subcarrier.channel)
                    power_UBS_GBS = list_UBS_power[j] * 1e-3 * dB2orig(UBS.list_subcarrier[j].channel)
                    # eq (35h) penalty function : QoS of ue - UBS and UBS - GBS link
                    if power_ue_UBS / dB2orig(param.noise) < 300 or power_UBS_GBS / (dB2orig(param.ICI) + dB2orig(param.noise)) < 300:
                        continue

                    power = power_ue_UBS * power_UBS_GBS
                    noise = dB2orig(UBS.list_subcarrier[j].channel) * 1e-3 * list_UBS_power[j] * 1e-3
                    noise += ( dB2orig(param.ICI - param.noise) * 1e-3 + 1 ) * dB2orig(subcarrier.channel) * list_ue_power[i, j] * 1e-3
                    noise += dB2orig(param.ICI) * 1e-3 + dB2orig(param.noise) * 1e-3
                    noise *= dB2orig(param.noise) * 1e-3
                    snr = power / noise
                    #print(f'{list_ue_power[i,j] = :2.3f} (mW), {power = }, {noise = }, {snr = }')
                    #print(f'{list_ue_power[i,j] = :2.3f} (mW), {snr = :4.2f}')
                    carrier_rate += math.log2(1 + snr) / 2 # Datarate of k-th subcarrier for i-th user
                    sumrate += carrier_rate
                    

        elif list_ue_mode[i] == 0:
            for j, subcarrier in enumerate(ue.list_subcarrier_GBS):
                if list_ue_alpha[i,j] == 1:
                    carrier_rate = 0
                    # R^k_{n,D} in paper. eq (15)
                    power = list_ue_power[i, j] * 1e-3 * dB2orig(subcarrier.channel)
                    # eq (35g) penalty function : QoS of ue - GBS link
                    if power / ( dB2orig(param.noise) + dB2orig(param.ICI) ) < param.SNR_threshold:
                        continue

                    carrier_rate += math.log2(1 + power / ( dB2orig(param.noise) * 1e-3 ) ) / 2
                    carrier_rate += math.log2(1 + power / ( dB2orig(param.noise) * 1e-3 + dB2orig(param.ICI) * 1e-3 ) ) / 2
                    sumrate += carrier_rate

        list_rate.append(sumrate)

    pf = sum([ rate / ue.serviced_data for rate, ue in zip(list_rate, list_ue)])
            
    return - pf + penalty
    

UBS, GBS, list_ue = sm.initialize_network()


#           mode (\beta)   subcarrier allocation             power allocation             UBS power allocation    x   y   z
dimension = param.num_ue + param.num_subcarriers + param.num_ue * param.num_subcarriers + param.num_subcarriers + 1 + 1 + 1

ue_mode_bound = [0, 1]
ue_alpha_bound = [0, param.num_ue]
ue_power_bound = [0, 100] # The number 0 and 100 are not power bounds. It is softmax ratio
UBS_power_bound = [0, 100] # The number 0 and 100 are not power bounds. It is softmax ratio
"""
position_x_bound = [max(UBS.prev_position.x - param.uav_max_dist, 0), 
                    min(UBS.prev_position.x + param.uav_max_dist, param.map_width)]
position_y_bound = [max(UBS.prev_position.y - param.uav_max_dist, 0), 
                    min(UBS.prev_position.y + param.uav_max_dist, param.map_width)]
position_z_bound = [max(UBS.prev_position.z - param.uav_max_dist, param.min_altitude), 
                    min(UBS.prev_position.z + param.uav_max_dist, param.max_altitude)]
"""
theta_bound = [0, 72] # Shperical coordinate primal angle which is discretized by unit of 5°
pi_bound = [0, 72] # Shperical coordinate primal angle which is discretized by unit of 5°
radius_bound = [0, 100] 

varbound = []
for _ in range(param.num_ue):
    varbound.append(ue_mode_bound)
for _ in range(param.num_subcarriers):
    varbound.append(ue_alpha_bound)
for _ in range(param.num_ue * param.num_subcarriers):
    varbound.append(ue_power_bound)
for _ in range(param.num_subcarriers):
    varbound.append(UBS_power_bound)
"""
varbound.append(position_x_bound)
varbound.append(position_y_bound)
varbound.append(position_z_bound)
"""
varbound.append(theta_bound)
varbound.append(pi_bound)
varbound.append(radius_bound)
varbound = np.array(varbound)
vartype = [['int']] * dimension

#print(f'{param.num_ue = }, {param.num_subcarriers = }')
# Nono = infty
algorithm_parameters = {'max_num_iteration' : 5000,
                        'population_size' : 100,\
                        'mutation_probability' : 0.8,\
                        'elit_ratio' : 0.03,\
                        'crossover_probability' : 0.5,\
                        'parents_portion' : 0.3,\
                        'crossover_type' : 'uniform',\
                        'max_iteration_without_improv' : 600}

model = ga(objective_function, dimension, 'int', varbound, 
           convergence_curve = True, algorithm_parameters = algorithm_parameters)

import time
start = time.time()
model.run()
print_gene_expression(*split_ga_vector(model.best_variable))
print(f'-------\nElapsed time: {time.time() - start}')
