#! /usr/bin/env python
import system_model as sm
import math
import numpy as np
import pygad
import copy
import argparse

import os, json
import time
import pickle

param = sm.param

# ./no_GBS_iterative_ga_optimizer.py --index_end 1 --datarate 3
def get_parser():
    parser = argparse.ArgumentParser(description='Simulate drone base station with specific depth',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--env_path', type=str, default='/home/hslyu/dbspf/data', help='Path of the environment directory')
    parser.add_argument('--result_path', type=str, default='/home/hslyu/dbspf/comparison_scheme/result', help='Path of the result directory')
    parser.add_argument('--index_start', type=int, default=0, help='Iteration start index')
    parser.add_argument('--index_end', type=int, help='Iteration end index')
    parser.add_argument('--datarate', type=int, help='Datarate')
    parser.add_argument('--num_ue', type=int, default=20, help='number of user')
    return parser

def get_valid_user(user_list: list[sm.Device], time_index):
    valid_user_list = []
    for user in user_list:
        if user.time_start <= time_index <= user.time_end:
            valid_user_list.append(user)
    return valid_user_list

def dB2orig(var_dB: float=0):
    return 10**(var_dB/10)

def softmax_array(arr: np.ndarray):
    arr = arr.astype(int)
    y = np.exp(arr - np.max(arr))
    regularized_arr = y / ( np.sum(np.exp(arr-np.max(arr))) + 1e-10)
    return regularized_arr

def proportion_array(arr: np.ndarray):
    if sum(arr) != 0:
        return arr/sum(arr)
    else:
        return arr

def print_gene_expression(list_ue_mode, list_ue_alpha, list_ue_power, \
                          list_UBS_power, position_x, position_y, position_z):
    print(f'{list_ue_mode = }')
    for i in range(param.num_ue): 
        print(f'sum(list_ue_alpha[{i}]) : {sum(list_ue_alpha[i]) : 4.2f}')
    for i in range(param.num_ue):
        print(f'list_ue_power[{i}] : {sum(list_ue_power[i]) : 4.2f}')
    print(f'{sum(list_UBS_power) = :4.2f}')
    print(f'position : ({position_x:4.2f}, {position_y:4.2f}, {position_z:4.2f})')

def encode_gene_max_snr(UBS, GBS, valid_user_list):
    valid_user_list = get_valid_user(list_ue, time_index)

    list_ue_UBS_alpha = np.empty(param.num_subcarriers)
    for i in range(param.num_subcarriers): 
        max_snr = -99999999
        max_user_index = 0
        for j, user in enumerate(valid_user_list):
            if max_snr <= user.list_subcarrier_UBS[i].channel:
                max_snr = user.list_subcarrier_UBS[i].channel
                max_user_index = j
        list_ue_UBS_alpha[i] = max_user_index
        
    list_ue_power = np.ones(len(valid_user_list) * param.num_subcarriers)
    list_position = np.zeros(3)

    gene = np.concatenate((list_ue_UBS_alpha, list_ue_power, list_position))
#    print(f"{list_ue_UBS_alpha= }")
#    print(f"{list_ue_power = }")
#    print(f"{list_position = }")
#    print(len(gene))
    return gene

def decode_gene(X):
    valid_user_list = get_valid_user(list_ue, time_index)
    #alpha power theta pi radius
    a = param.num_subcarriers
    b = a + len(valid_user_list) * param.num_subcarriers

    list_ue_UBS_alpha_encoded = X[:a]
    list_ue_UBS_alpha = (np.arange(len(valid_user_list)).reshape([-1,1]) == list_ue_UBS_alpha_encoded).astype(int)

    list_ue_power_encoded = X[a:b] # before softmax
    list_ue_power_encoded = list_ue_power_encoded.reshape((len(valid_user_list),-1))
    list_ue_power_encoded = np.multiply(list_ue_UBS_alpha, list_ue_power_encoded )
    list_ue_power = list_ue_power_encoded/sum(list_ue_power_encoded.flatten()) * param.max_ue_power_mW if sum(list_ue_power_encoded.flatten()) !=0 else list_ue_power_encoded
#    list_ue_power = np.apply_along_axis(proportion_array, 1, list_ue_power_encoded) * param.max_ue_power_mW

    theta = math.radians(X[b])
    pi = math.radians(X[b + 1])
    radius = X[b + 2]
    position_x = UBS.prev_position.x + radius * math.sin(theta) * math.cos(pi)
    position_y = UBS.prev_position.y + radius * math.sin(theta) * math.sin(pi)
    position_z = UBS.prev_position.z + radius * math.cos(pi)

    return list_ue_UBS_alpha, list_ue_power, position_x, position_y, position_z, valid_user_list

def sol2rate(solution):
    """
    Args:
        solution (list): Array of genes
    Returns:
        rate_list (list[float]): list of sumrate for users
    """

    # split vector
    # list_ue_mode - size : param.num_ue
    # list_ue_{alpha, power} - size : param.num_ue * param.num_subcarriers
    # list_UBS_power - size : param.num_subcarriers
    # position_{x,y,z} - size : 1
    list_ue_UBS_alpha, list_ue_power, position_x, position_y, position_z, valid_user_list = decode_gene(solution)
    if param.min_altitude > position_z or param.max_altitude < position_z:
        return [0]*len(valid_user_list)

    UBS.position = sm.Position(position_x, position_y, position_z)

    UBS.calc_pathloss() 
    UBS.calc_channel() 

    list_rate = []
    for i, user in enumerate(valid_user_list):
        # Find sum-rate 
        sumrate = 0
        for j, subcarrier in enumerate(user.list_subcarrier_UBS):
            if list_ue_UBS_alpha[i,j] == 1:
                # R^k_{n,R} in paper. eq (31)
                snr = list_ue_power[i, j] * dB2orig(subcarrier.channel) / dB2orig(param.noise)
                # eq (35h) penalty function : QoS of ue - UBS and UBS - GBS link
                if snr < param.SNR_threshold:
                    continue
                sumrate += param.subcarrier_bandwidth*math.log2(1 + snr) # Datarate of k-th subcarrier for i-th user
#        if sumrate < args.datarate:
#            sumrate = 0
        list_rate.append(sumrate)

    return list_rate

def fitness_func(solution, solution_idx):
    list_rate = sol2rate(solution)
    pf = sum([ rate / ue.serviced_data for rate, ue in zip(list_rate, list_ue)])
            
    return pf


def callback_generation(ga_instance):
    global last_fitness
    Generation = ga_instance.generations_completed
    Fitness = ga_instance.best_solution()[1]
    Change = ga_instance.best_solution()[1] - last_fitness
#    print( f'{Generation = :02d}, {Fitness = :.4f}, {Change = :.4f}, User: {len(valid_user_list)}')
    if Generation % 20 == 0:
        print( f'{Generation = :02d}, {Fitness = :.4f}, User: {valid_user_list}')
    last_fitness = ga_instance.best_solution()[1]

parser = get_parser()
args = parser.parse_args()
param.SNR_threshold = 2**(args.datarate/(param.subcarrier_bandwidth*param.num_subcarriers))-1

ue_alpha_bound   = {'low' : 0,'high' : param.num_ue + 1} # int
ue_power_bound   = {'low' : 0,'high' : 10 + 1} # int, The number 0 and 100 are not power bounds. It is softmax ratio
theta_bound      = {'low' : 0,'high' : 300, 'step': 60} # int
pi_bound         = {'low' : 0,'high' : 300, 'step': 60} # int
#radius_bound     = {'low' : 0,'high' : param.uav_max_dist + 1e-8, 'step' : 1} # real
radius_bound     = {'low' : 0,'high' : param.uav_max_dist, 'step':15} # int

num_generations = 10000 # Number of generations.
num_parents_mating = 20 # Number of solutions to be selected as parents in the mating pool.
sol_per_pop = 50 # Number of solutions in the population.

idx_start = args.index_start
idx_end = args.index_end
avg_time=0
avg_obj=0
current_obj = 0
for i in range(idx_start, idx_end):#{{{
    envname = f'env_{i:04d}'
    dirname = os.path.join(args.result_path, f'tw{param.num_timeslots}_user{param.num_ue}/datarate_{args.datarate}/{envname}')
    UBS, GBS, list_ue = sm.initialize_network(os.path.join(args.env_path, f'env/{envname}.json'))
    #UBS, GBS, list_ue = sm.initialize_network()

    global time_index 
    for time_index in range(param.num_timeslots):#{{{
        start = time.time()
        valid_user_list = get_valid_user(list_ue, time_index)
        if valid_user_list == []:
            continue

        gene_space = []
        gene_type = []
        # alpha - UBS
        for _ in range(param.num_subcarriers):
            gene_space.append(ue_alpha_bound)
            gene_type.append(int)
        # UE power
        for _ in range(len(valid_user_list) * param.num_subcarriers):
            gene_space.append(ue_power_bound)
            gene_type.append(int)

        gene_space.append(theta_bound)
        gene_type.append(int)
        gene_space.append(pi_bound)
        gene_type.append(int)
        gene_space.append(radius_bound)
        gene_type.append(int)

        #           subcarrier allocation          power allocation             theta pi radius
        num_genes = param.num_subcarriers + len(valid_user_list) * param.num_subcarriers + 1 + 1 + 1

        last_fitness = 0

        ga_instance = pygad.GA(num_generations=num_generations, 
                               num_parents_mating=num_parents_mating, 
                               parent_selection_type = 'rank',
                               fitness_func=fitness_func,
                               sol_per_pop=sol_per_pop, 
                               num_genes=num_genes,
                               on_generation=callback_generation,
                               crossover_type = 'two_points',
                               gene_type = copy.deepcopy(gene_type),
                               gene_space = gene_space,
                               stop_criteria = ["saturate_500"]
                               )
        
        # Running the GA to optimize the parameters of the function.
        ga_instance.population[0] = encode_gene_max_snr(UBS, GBS, valid_user_list)
        ga_instance.run()


        # After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
        #ga_instance.plot_fitness()

        # Returning the details of the best solution.
        solution, solution_fitness, solution_idx = ga_instance.best_solution()
#        print("Parameters of the best solution : {solution}".format(solution=solution))
#        print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
#        print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))
#        if ga_instance.best_solution_generation != -1:
#            print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))

        # Applying solution to system model
        list_ue_UBS_alpha, list_ue_power, position_x, position_y, position_z, _ = decode_gene(solution)
        list_rate = sol2rate(solution)

        for rate, ue in zip(list_rate, valid_user_list):
            ue.serviced_data += rate
            ue.serviced_time += 1

        UBS.prev_position = UBS.position
        UBS.position = sm.Position(position_x, position_y, position_z)

        if not os.path.exists(dirname):
            os.makedirs(dirname)
        ga_instance.save(filename=f'{dirname}/ga_{time_index}')
        UBS.save(f'{dirname}/UBS_{time_index}.pkl')

        current_obj = sum([math.log(ue.serviced_data-10) for ue in list_ue if ue.serviced_data != 10])
        print(f'Current episode: {i}, time: {time_index}, elapsed time: {time.time()-start:.3f}, current obj: {current_obj: .4f}')
        avg_time += time.time()-start

    avg_obj += sum([math.log(ue.serviced_data-10) for ue in list_ue if ue.serviced_data != 10])#}}}

avg_time /= idx_end-idx_start
avg_obj /= idx_end-idx_start
print(f'Average elapsed time: {avg_time}, average PF: {avg_obj:.4f}')
