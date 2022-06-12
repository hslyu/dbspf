import system_model as sm
import math
import numpy as np
import pygad
import copy

import os, json
import time
import pickle

param = sm.param
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

def encode_gene_max_snr(UBS, GBS, list_ue, time_index):
    valid_user_list = get_valid_user(list_ue, time_index)

    list_ue_UBS_alpha = np.empty(param.num_subcarriers)
    for i in range(param.num_subcarriers): 
        max_snr = -99999999
        max_user_index = 0
        for user in valid_user_list:
            if max_snr <= user.list_subcarrier_UBS[i].channel:
                max_snr = user.list_subcarrier_UBS[i].channel
                max_user_index = user.id
        list_ue_UBS_alpha[i] = max_user_index
        
    list_ue_power = np.ones(param.num_ue * param.num_subcarriers)
    list_position = np.zeros(3)

    gene = np.concatenate((list_ue_UBS_alpha, list_ue_power, list_position))
#    print(f"{list_ue_mode= }")
#    print(f"{list_ue_UBS_alpha= }")
#    print(f"{list_ue_GBS_alpha= }")
#    print(f"{list_ue_power = }")
#    print(f"{list_UBS_power = }")
#    print(f"{list_position = }")
#    print(len(gene))
    return gene

def decode_gene(X):
    #alpha power theta pi radius
    a = param.num_subcarriers
    b = a + param.num_ue * param.num_subcarriers

    list_ue_UBS_alpha_encoded = X[:a]
    list_ue_UBS_alpha = (np.arange(param.num_ue).reshape([-1,1]) == list_ue_UBS_alpha_encoded).astype(int)

    list_ue_power_encoded = X[a:b] # before softmax
    list_ue_power_encoded = list_ue_power_encoded.reshape((param.num_ue,-1))
    list_ue_power_encoded = np.multiply(list_ue_UBS_alpha, list_ue_power_encoded )
    list_ue_power = list_ue_power_encoded/sum(list_ue_power_encoded.flatten()) * param.max_ue_power_mW
#    list_ue_power = np.apply_along_axis(proportion_array, 1, list_ue_power_encoded) * param.max_ue_power_mW

    theta = math.radians(X[b] * 10)
    pi = math.radians(X[b + 1] * 10)
    radius = X[b + 2]
    position_x = UBS.prev_position.x + radius * math.sin(theta) * math.cos(pi)
    position_y = UBS.prev_position.y + radius * math.sin(theta) * math.sin(pi)
    position_z = UBS.prev_position.z + radius * math.cos(pi)

    return list_ue_UBS_alpha, list_ue_power, position_x, position_y, position_z


def sol2rate(solution, mode_penalty=True):
    """
    Args:
        solution (list): Array of genes
    Returns:
        rate_list (list[float]): list of sumrate for users
    """

    penalty = 0

    # split vector
    # list_ue_mode - size : param.num_ue
    # list_ue_{alpha, power} - size : param.num_ue * param.num_subcarriers
    # list_UBS_power - size : param.num_subcarriers
    # position_{x,y,z} - size : 1
    list_ue_UBS_alpha, list_ue_power, position_x, position_y, position_z = decode_gene(solution)

    UBS.position = sm.Position(position_x, position_y, position_z)
    # eq (35e) penality function
    if UBS.position.l2_norm(UBS.prev_position) > param.uav_max_dist:
        penalty += 10 * UBS.position.l2_norm(UBS.prev_position)

    UBS.calc_pathloss() 
    UBS.calc_channel() 

    list_rate = []
    for i, ue in enumerate(get_valid_user(list_ue, time_index)):
        # Find sum-rate 
        sumrate = 0
        for j, subcarrier in enumerate(ue.list_subcarrier_UBS):
            if list_ue_UBS_alpha[i,j] == 1:
                carrier_rate = 0
                # R^k_{n,R} in paper. eq (31)
                power_ue_UBS = list_ue_power[i, j] * dB2orig(subcarrier.channel)
#                print(list_ue_power[i,j])
                snr = power_ue_UBS / dB2orig(param.noise)
                # eq (35h) penalty function : QoS of ue - UBS and UBS - GBS link
                if snr < param.SNR_threshold:
                    continue
                carrier_rate += param.subcarrier_bandwidth*math.log2(1 + snr) / 2 # Datarate of k-th subcarrier for i-th user
                sumrate += carrier_rate

        list_rate.append(sumrate)

    if mode_penalty:
        return list_rate, penalty
    else:
        return list_rate


def fitness_func(solution, solution_idx):
    list_rate, penalty = sol2rate(solution)
    pf = sum([ rate / ue.serviced_data for rate, ue in zip(list_rate, list_ue)])
            
    return pf - penalty


def callback_generation(ga_instance):
    global last_fitness
    Generation = ga_instance.generations_completed
    Fitness = ga_instance.best_solution()[1]
    Change = ga_instance.best_solution()[1] - last_fitness
#    print( f'{Generation = :02d},\t {Fitness = },\t\t {Change = }')
    last_fitness = ga_instance.best_solution()[1]

ue_alpha_bound   = {'low' : 0,'high' : param.num_ue + 1} # int
ue_power_bound   = {'low' : 0,'high' : 10 + 1} # int, The number 0 and 100 are not power bounds. It is softmax ratio
theta_bound      = {'low' : 0,'high' : 36} # int
pi_bound         = {'low' : 0,'high' : 36} # int
#radius_bound     = {'low' : 0,'high' : param.uav_max_dist + 1e-8, 'step' : 1} # real
radius_bound     = {'low' : 0,'high' : param.uav_max_dist} # int

gene_space = []
gene_type = []
# alpha - UBS
for _ in range(param.num_subcarriers):
    gene_space.append(ue_alpha_bound)
    gene_type.append(int)
# UE power
for _ in range(param.num_ue * param.num_subcarriers):
    gene_space.append(ue_power_bound)
    gene_type.append(int)

gene_space.append(theta_bound)
gene_type.append(int)
gene_space.append(pi_bound)
gene_type.append(int)
gene_space.append(radius_bound)
gene_type.append(int)

num_generations = 10 # Number of generations.
num_parents_mating = 1 # Number of solutions to be selected as parents in the mating pool.
sol_per_pop = 4 # Number of solutions in the population.
#           subcarrier allocation          power allocation             theta pi radius
num_genes = param.num_subcarriers + param.num_ue * param.num_subcarriers + 1 + 1 + 1

num_exp = 10
avg_time=0
avg_obj=0
current_obj = 0
for i in range(num_exp):#{{{
    envname = f'env_{i:04d}'
    dirname = f'result/tw{param.num_timeslots}_user{param.num_ue}/{envname}'
    UBS, GBS, list_ue = sm.initialize_network(f'/home/hslyu/dbspf/data/tw60/env/{envname}.json')
    #UBS, GBS, list_ue = sm.initialize_network()

    start = time.time()
    global time_index 
    for time_index in range(param.num_timeslots):#{{{
        if get_valid_user(list_ue, time_index) == []:
            continue

        last_fitness = 0

        ga_instance = pygad.GA(num_generations=num_generations, 
                               num_parents_mating=num_parents_mating, 
                               parent_selection_type = 'rank',
                               fitness_func=fitness_func,
                               sol_per_pop=sol_per_pop, 
                               num_genes=num_genes,
                               on_generation=callback_generation,
                               crossover_type = 'single_point',
                               gene_type = copy.deepcopy(gene_type),
                               gene_space = gene_space,
                               stop_criteria = ["saturate_30"]
                               )
        
        # Running the GA to optimize the parameters of the function.
        ga_instance.population[0] = encode_gene_max_snr(UBS, GBS, list_ue, time_index)
        ga_instance.run()

        # After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
        #ga_instance.plot_fitness()

        # Returning the details of the best solution.
        solution, solution_fitness, solution_idx = ga_instance.best_solution()
#        print("Parameters of the best solution : {solution}".format(solution=solution)){{{
#        print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
#        print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))
#        if ga_instance.best_solution_generation != -1:
#            print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))}}}

        # Applying solution to system model
        list_ue_UBS_alpha, list_ue_power, position_x, position_y, position_z = decode_gene(solution)
        list_rate = sol2rate(solution, False)

        for rate, ue in zip(list_rate, list_ue):
            ue.serviced_data += rate
        UBS.prev_position = UBS.position
        UBS.position = sm.Position(position_x, position_y, position_z)

        if not os.path.exists(dirname):
            os.makedirs(dirname)
        ga_instance.save(filename=f'{dirname}/ga_{time_index}')
        UBS.save(f'{dirname}/UBS_{time_index}.pkl')

        current_obj = sum([math.log2(ue.serviced_data-10) for ue in list_ue if ue.serviced_data != 10])
        if i == 0:
            print(f'Current episode: {i}, time: {time_index}, reward: {solution_fitness:.4f}, current obj: {current_obj: .4f}, avg. obj: {avg_obj:.4f}, avg. elapsed time: {avg_time:.4f}')
        else:
            print(f'Current episode: {i}, time: {time_index}, reward: {solution_fitness:.4f}, current obj: {current_obj/i: .4f},, avg. obj: {avg_obj/i:.4f}, avg. elapsed time: {avg_time/i:.4f}')
        

    avg_time += time.time()-start
    avg_obj += sum([math.log2(ue.serviced_data-10) for ue in list_ue])#}}}

avg_time /= num_exp
avg_obj /= num_exp
print(f'{avg_time = }, {avg_obj = }')
