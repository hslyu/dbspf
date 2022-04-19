import system_model as sm
import math
import numpy as np
import pygad

param = sm.param
def dB2orig(var_dB: float=0):
    return 10**(var_dB/10)

def softmax_array(arr: np.ndarray):
    arr = arr.astype(int)
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

def fitness_func(solution, solution_idx):
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
                    if power_ue_UBS / (dB2orig(param.noise) * 1e-3) < 300 or power_UBS_GBS / ( (dB2orig(param.ICI) + dB2orig(param.noise)) * 1e-3) < 300:
                        continue

                    power = power_ue_UBS * power_UBS_GBS
                    noise = dB2orig(UBS.list_subcarrier[j].channel) * 1e-3 * list_UBS_power[j] * 1e-3
                    noise += ( dB2orig(param.ICI - param.noise) * 1e-3 + 1 ) * dB2orig(subcarrier.channel) * list_ue_power[i, j] * 1e-3
                    noise += dB2orig(param.ICI) * 1e-3 + dB2orig(param.noise) * 1e-3
                    noise *= dB2orig(param.noise) * 1e-3
                    snr = power / noise
                    carrier_rate += math.log2(1 + snr) / 2 # Datarate of k-th subcarrier for i-th user
                    sumrate += carrier_rate

        elif list_ue_mode[i] == 0:
            for j, subcarrier in enumerate(ue.list_subcarrier_GBS):
                if list_ue_alpha[i,j] == 1:
                    carrier_rate = 0
                    # R^k_{n,D} in paper. eq (15)
                    power = list_ue_power[i, j] * 1e-3 * dB2orig(subcarrier.channel)
                    #print(f'{list_ue_power[i,j] = :2.3f} th : {power / ( dB2orig(param.noise) + dB2orig(param.ICI) ) : 3.1f}')
                    # eq (35g) penalty function : QoS of ue - GBS link
                    if power / ( (dB2orig(param.noise) + dB2orig(param.ICI)) * 1e-3) < param.SNR_threshold:
                        continue

                    carrier_rate += math.log2(1 + power / ( dB2orig(param.noise) * 1e-3 ) ) / 2
                    carrier_rate += math.log2(1 + power / ( dB2orig(param.noise) * 1e-3 + dB2orig(param.ICI) * 1e-3 ) ) / 2
                    sumrate += carrier_rate

        list_rate.append(sumrate)

    pf = sum([ rate / ue.serviced_data for rate, ue in zip(list_rate, list_ue)])
            
    return pf - penalty

num_generations = 10000 # Number of generations.
num_parents_mating = 10 # Number of solutions to be selected as parents in the mating pool.

# To prepare the initial population, there are 2 ways:
# 1) Prepare it yourself and pass it to the initial_population parameter. This way is useful when the user wants to start the genetic algorithm with a custom initial population.
# 2) Assign valid integer values to the sol_per_pop and num_genes parameters. If the initial_population parameter exists, then the sol_per_pop and num_genes parameters are useless.
sol_per_pop = 50 # Number of solutions in the population.
#           mode (\beta)   subcarrier allocation             power allocation             UBS power allocation    x   y   z
num_genes = param.num_ue + param.num_subcarriers + param.num_ue * param.num_subcarriers + param.num_subcarriers + 1 + 1 + 1


last_fitness = 0
def callback_generation(ga_instance):
    global last_fitness
    Generation = ga_instance.generations_completed
    Fitness = ga_instance.best_solution()[1]
    Change = ga_instance.best_solution()[1] - last_fitness
    print( f'{Generation = :02d},\t {Fitness = },\t\t {Change = }')
    last_fitness = ga_instance.best_solution()[1]

ue_mode_bound    = [0, 1] # int
ue_alpha_bound   = {'low' : 0,'high' : param.num_ue + 1} # int
ue_power_bound   = {'low' : 0,'high' : 100 + 1} # int, The number 0 and 100 are not power bounds. It is softmax ratio
UBS_power_bound  = {'low' : 0,'high' : 100 + 1} # int, The number 0 and 100 are not power bounds. It is softmax ratio
theta_bound      = {'low' : 0,'high' : 360} # real
pi_bound         = {'low' : 0,'high' : 360} # real
radius_bound     = {'low' : 0,'high' : param.uav_max_dist + 1e-8, 'step' : 0.1} # real

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
# Creating an instance of the GA class inside the ga module. Some parameters are initialized within the constructor.
ga_instance = pygad.GA(num_generations=num_generations,
                       num_parents_mating=num_parents_mating, 
                       fitness_func=fitness_func,
                       sol_per_pop=sol_per_pop, 
                       num_genes=num_genes,
                       on_generation=callback_generation,
                       crossover_type = 'uniform',
                       gene_type = gene_type,
                       gene_space = gene_space,
                       stop_criteria = ["saturate_500"]
                       )

# Running the GA to optimize the parameters of the function.
ga_instance.run()

# After the generations complete, some plots are showed that summarize the how the outputs/fitenss values evolve over generations.
ga_instance.plot_fitness()

# Returning the details of the best solution.
solution, solution_fitness, solution_idx = ga_instance.best_solution()
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

if ga_instance.best_solution_generation != -1:
    print("Best fitness value reached after {best_solution_generation} generations.".format(best_solution_generation=ga_instance.best_solution_generation))

# Saving the GA instance.
filename = 'genetic' # The filename to which the instance is saved. The name is without extension.
ga_instance.save(filename=filename)

# Loading the saved GA instance.
loaded_ga_instance = pygad.load(filename=filename)
loaded_ga_instance.plot_fitness()
