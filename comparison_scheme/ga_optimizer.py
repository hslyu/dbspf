import system_model as sm
import math
import numpy as np
import pygad
import copy

param = sm.param
def get_valid_user(user_list: list[sm.Device], time):
    valid_user_list = []
    for user in user_list:
        if user.time_start <= time <= user.time_end:
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

def decode_gene(X):
    a = param.num_ue
    b = a + param.num_subcarriers
    c = b + param.num_ue * param.num_subcarriers
    d = c + param.num_subcarriers

    list_ue_mode = X[:a] 
    list_ue_alpha_encoded = X[a:b]
    list_ue_alpha = (np.arange(param.num_ue).reshape([-1,1]) == list_ue_alpha_encoded).astype(int)

    list_ue_power_encoded = X[b:c] # before softmax
    list_ue_power_encoded = list_ue_power_encoded.reshape((param.num_ue,-1))
    list_ue_power_encoded = np.multiply(list_ue_alpha, list_ue_power_encoded )
#    list_ue_power = np.apply_along_axis(softmax_array, 1, list_ue_power_encoded) * param.max_ue_power_mW
    list_ue_power = np.apply_along_axis(proportion_array, 1, list_ue_power_encoded) * param.max_ue_power_mW
#    print(list_ue_alpha)
#    print(list_ue_power_encoded )
#    print(np.multiply(list_ue_alpha, list_ue_power_encoded ))
#    print(list_ue_power)
#    print("")

    list_UBS_power_encoded = X[c:d]
#    list_UBS_power = softmax_array(list_UBS_power_encoded) * param.max_uav_power_mW
    list_UBS_power = proportion_array(list_UBS_power_encoded) * param.max_uav_power_mW

    theta = math.radians(X[d] * 5)
    pi = math.radians(X[d + 1] * 5)
    radius = param.uav_max_dist * X[d + 2] / 100
    position_x = UBS.prev_position.x + radius * math.sin(theta) * math.cos(pi)
    position_y = UBS.prev_position.y + radius * math.sin(theta) * math.sin(pi)
    position_z = UBS.prev_position.z + radius * math.cos(pi)
#    print(f'Position differences: {radius} {radius * math.sin(theta) * math.cos(pi)}, {radius * math.sin(theta) * math.sin(pi)}, {radius * math.cos(pi)}')

    return list_ue_mode, list_ue_alpha, list_ue_power, list_UBS_power, position_x, position_y, position_z

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
    list_ue_mode, list_ue_alpha, list_ue_power, list_UBS_power, position_x, position_y, position_z = decode_gene(solution)
#    print_gene_expression(list_ue_mode, list_ue_alpha, list_ue_power, \
#                          list_UBS_power, position_x, position_y, position_z)

    UBS.position = sm.Position(position_x, position_y, position_z)
    # eq (35e) penality function
    if UBS.position.l2_norm(UBS.prev_position) > param.uav_max_dist:
        penalty += 10 * UBS.position.l2_norm(UBS.prev_position)

    UBS.calc_pathloss() 
    UBS.calc_channel() 

    list_rate = []
#    for i, ue in enumerate(list_ue):
    for i, ue in enumerate(get_valid_user(list_ue, time)):
        # Find sum-rate 
        sumrate = 0
        if list_ue_mode[i] == 1:
            for j, subcarrier in enumerate(ue.list_subcarrier_UBS):
                if list_ue_alpha[i,j] == 1:
                    carrier_rate = 0
                    # R^k_{n,R} in paper. eq (31)
                    power_ue_UBS = list_ue_power[i, j] * dB2orig(subcarrier.channel)
                    power_UBS_GBS = list_UBS_power[j] * dB2orig(UBS.list_subcarrier[j].channel)
#                    print(f"UBS power {list_ue_power[i, j]:2.3f}, {list_UBS_power[j]:2.3f}" )
#                    print("ubs", subcarrier.channel,UBS.list_subcarrier[j].channel)
                   # eq (35h) penalty function : QoS of ue - UBS and UBS - GBS link
                    if power_ue_UBS / dB2orig(param.noise) < param.SNR_threshold or power_UBS_GBS / (dB2orig(param.ICI) + dB2orig(param.noise)) < param.SNR_threshold:
#                        print(power_ue_UBS / dB2orig(param.noise))
#                        print(power_UBS_GBS /  (dB2orig(param.ICI) + dB2orig(param.noise)))
#                        print("")
                        continue
#                    print(power_ue_UBS / dB2orig(param.noise) )
#                    print(power_UBS_GBS /  (dB2orig(param.ICI) + dB2orig(param.noise)) )
#                    print("")
#                    power = power_ue_UBS * power_UBS_GBS
#                    noise = dB2orig(UBS.list_subcarrier[j].channel) * list_UBS_power[j]
#                    noise += ( dB2orig(param.ICI - param.noise) + 1 ) * dB2orig(subcarrier.channel) * list_ue_power[i, j] 
#                    noise += dB2orig(param.ICI) + dB2orig(param.noise)
#                    noise *= dB2orig(param.noise)
                    power = power_ue_UBS
                    noise = dB2orig(param.noise)
                    snr = power / noise
                    carrier_rate += param.subcarrier_bandwidth*math.log2(1 + snr) / 2 # Datarate of k-th subcarrier for i-th user
                    sumrate += carrier_rate
#                    print(carrier_rate)

        elif list_ue_mode[i] == 0:
            for j, subcarrier in enumerate(ue.list_subcarrier_GBS):
                if list_ue_alpha[i,j] == 1:
                    carrier_rate = 0
                    # R^k_{n,D} in paper. eq (15)
                    power = list_ue_power[i, j] * dB2orig(subcarrier.channel)
#                    print("gbs power",list_ue_power[i, j])
#                    print("gbs", subcarrier.channel)
                    #print(f'{list_ue_power[i,j] = :2.3f} th : {power / ( dB2orig(param.noise) + dB2orig(param.ICI) ) : 3.1f}')
                    # eq (35g) penalty function : QoS of ue - GBS link
                    if power / (dB2orig(param.noise) + dB2orig(param.ICI)) < param.SNR_threshold:
                        continue

                    carrier_rate += param.subcarrier_bandwidth*math.log2(1 + power /  dB2orig(param.noise)  ) / 2
                    carrier_rate += param.subcarrier_bandwidth*math.log2(1 + power / ( dB2orig(param.noise) + dB2orig(param.ICI)) ) / 2
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

num_generations = 10 # Number of generations.
num_parents_mating = 10 # Number of solutions to be selected as parents in the mating pool.

# To prepare the initial population, there are 2 ways:
# 1) Prepare it yourself and pass it to the initial_population parameter. This way is useful when the user wants to start the genetic algorithm with a custom initial population.
# 2) Assign valid integer values to the sol_per_pop and num_genes parameters. If the initial_population parameter exists, then the sol_per_pop and num_genes parameters are useless.
sol_per_pop = 50 # Number of solutions in the population.
#           mode (\beta)   subcarrier allocation             power allocation             UBS power allocation    x   y   z
num_genes = param.num_ue + param.num_subcarriers + param.num_ue * param.num_subcarriers + param.num_subcarriers + 1 + 1 + 1

def callback_generation(ga_instance):
    global last_fitness
    Generation = ga_instance.generations_completed
    Fitness = ga_instance.best_solution()[1]
    Change = ga_instance.best_solution()[1] - last_fitness
    print( f'{Generation = :02d},\t {Fitness = },\t\t {Change = }')
    last_fitness = ga_instance.best_solution()[1]

ue_mode_bound    = [0, 1] # int
ue_alpha_bound   = {'low' : 0,'high' : param.num_ue + 1} # int
ue_power_bound   = {'low' : 0,'high' : 10 + 1} # int, The number 0 and 100 are not power bounds. It is softmax ratio
UBS_power_bound  = {'low' : 0,'high' : 10 + 1} # int, The number 0 and 100 are not power bounds. It is softmax ratio
theta_bound      = {'low' : 0,'high' : 360} # int
pi_bound         = {'low' : 0,'high' : 360} # int
radius_bound     = {'low' : 0,'high' : param.uav_max_dist + 1e-8, 'step' : 1} # real

gene_space = []
gene_type = []
for _ in range(param.num_ue):
    gene_space.append(ue_mode_bound)
    gene_type.append(int)
for _ in range(param.num_subcarriers):
    gene_space.append(ue_alpha_bound)
    gene_type.append(int)
for _ in range(param.num_ue * param.num_subcarriers):
    gene_space.append(ue_power_bound)
    gene_type.append(int)
for _ in range(param.num_subcarriers):
    gene_space.append(UBS_power_bound)
    gene_type.append(int)

gene_space.append(theta_bound)
gene_type.append(int)
gene_space.append(pi_bound)
gene_type.append(int)
gene_space.append(radius_bound)
gene_type.append(float)

UBS, GBS, list_ue = sm.initialize_network()

global time
for time in range(param.num_timeslots):
    last_fitness = 0

    ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating, 
                           fitness_func=fitness_func,
                           sol_per_pop=sol_per_pop, 
                           num_genes=num_genes,
                           on_generation=callback_generation,
                           crossover_type = 'uniform',
                           gene_type = copy.deepcopy(gene_type),
                           gene_space = gene_space,
                           stop_criteria = ["saturate_50"]
                           )

    # Running the GA to optimize the parameters of the function.
    ga_instance.run()

    # After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
    #ga_instance.plot_fitness()

    # Returning the details of the best solution.
    solution, solution_fitness, solution_idx = ga_instance.best_solution()
    print("Parameters of the best solution : {solution}".format(solution=solution))
    print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
    print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

    # Applying solution to system model
    list_rate = sol2rate(solution, False)
    for rate, ue in zip(list_rate, list_ue):
        ue.serviced_data += rate

    if ga_instance.best_solution_generation != -1:
        print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))

    # Saving the GA instance.
    filename = 'genetic' # The filename to which the instance is saved. The name is without extension.
    ga_instance.save(filename=filename)

    # Loading the saved GA instance.
#    loaded_ga_instance = pygad.load(filename=filename)
#    loaded_ga_instance.plot_fitness()
    convergence = np.zeros(shape=[1,model.num_generations+1])
    conv = np.array(model.best_solutions_fitness)
    convergence[:,0:len(conv)] = conv


print(f"Final PF value: {sum([math.log2(ue.serviced_data) for ue in list_ue])}")
