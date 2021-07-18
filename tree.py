#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import numpy as np 
import random
import math
import time
import copy

# Constant for UAV
VEHICLE_VELOCITY = 10. # m/s
TIME_STEP = 1 # s
MAX_TIME = 100
## Constant for map
MAP_WIDTH = 200 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 100 # meter
GRID_SIZE = 10 # meter
# Constant for user
NUM_UE = 80
TIME_WINDOW_SIZE = [5,5]
DATARATE_WINDOW = [35, 60] # Requiring datarate Mb/s
INITIAL_DATA = 10 # Mb
# Tree depth
TREE_DEPTH = 2
# Constant for wirless communication
FREQUENCY = 2.0*1e9 # Hz
LIGHTSPEED = 3*1e8 # m/s
BANDWIDTH = 1. # 20 MHz per unit
POWER = 1. # 200 mW per unit
NOISE_DENSITY = -174+50 # dBm/20MHz , noise spectral density, https://en.wikipedia.org/wiki/Johnson%E2%80%93Nyquist_noise 
LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 40 # dB, excessive pathloss of nlos link
SURROUNDING_A = 9.64 # Envrionmental parameter for probablistic LOS link//10
SURROUNDING_B = 0.04 # Envrionmental parameter for probablistic LOS link
# Optimization hyperparameter
EPSILON = 1e-9
STEP_SIZE = 1e-1
THRESHOLD = 1e-7
# etc
INF = 1e8-1

class User:
    def __init__(self, uid=0, loc_x=0, loc_y=0, time_start=0, tw_size=0, datarate=0):#{{{
        self.id = uid
        self.position = [loc_x, loc_y]
        self.time_start = time_start
        self.time_end = self.time_start + tw_size
        # Requiring datarate b/s
        self.datarate = datarate
        # Temporary variables for easy implementation
        self.se = 0.
        self.ra = 0.
        self.psd = 0.
        self.snr = 0.
        # Total received throughput
        # Initial is not zero for our formulation
        self.total_data = INITIAL_DATA
#        self.max_data = 3*self.datarate
        self.max_data = INF
    
    def __str__(self):
        return "id: {}, (x,y) : ({:.2f}, {:.2f}), tw : [{}, {}], tw_size : {}".format(
            self.node_id, self.location[0], self.location[1], self.time_start, self.time_end, self.tw_size)
    
    def __repr__(self):
        return "{}".format(self.id)#}}}

class TrajectoryNode:
    def __init__(self, position, parent=None):
        # link
        self.parent = parent
        self.leafs = []
        
        # value
        self.position = position
        self.user_list = [User() for i in range(NUM_UE)]
        if parent is not None:
            self.user_copy(parent.user_list)
            self.current_time = parent.current_time+1
            self.reward = self.get_reward()
#            self.reward = self.get_random_reward()
            start = time.time()
#            print("Reward is calculated from node and elapsed time:", self.position, time.time()-start)
        # If this node is root
        else:
            self.reward = 0
            self.current_time = 0

#        print(self.current_time)
    def user_copy(self, user_list):
        for idx, user in enumerate(user_list):
            self.user_list[idx].id = user.id
            self.user_list[idx].position = user.position
            self.user_list[idx].time_start = user.time_start
            self.user_list[idx].time_end = user.time_end
            self.user_list[idx].datarate = user.datarate
            self.user_list[idx].max_data = user.max_data
            self.user_list[idx].total_data = user.total_data

    def get_info(self):
        print("User throughput list (Mbps)")
        print([user.total_data//max(user.time_start-user.time_end,self.current_time) for user in self.user_list])
        print("User total data (Mb)")
        print([user.total_data//1 for user in self.user_list])

    def __repr__(self):
        return "{}".format(self.position)
    
    def get_random_reward(self):
        return random.randint(0,10)

    # Distance between the UAV and i-th User
    def distance_from_leaf(self, position, user):
        return math.sqrt( (position[0] - user.position[0])**2 +\
                 (position[1] - user.position[1])**2 + position[2]**2 )

    # Caculate pathloss --> snr --> spectral efficiency
    def get_pathloss(self, position, user):
        distance = self.distance_from_leaf(position, user)#{{{
        angle = math.pi/2 - math.acos(position[2]/distance)
        los_prob = 1/(1 + SURROUNDING_A * \
                math.exp(-SURROUNDING_B*(180/math.pi*angle - SURROUNDING_A)))
        pathloss = 20*math.log10(4*math.pi*FREQUENCY*distance/LIGHTSPEED) + los_prob*LOS_EXCESSIVE + \
                    (1-los_prob)*NLOS_EXCESSIVE#}}}
        return pathloss
    
    # Because unit of psd is 200mW/20MHz, we should convert it to mw/Hz
    def psd2snr(self, psd, pathloss):
        return 10*math.log10(psd) - pathloss - NOISE_DENSITY
    # Because unit of resource is 20MHz,
    # we should convert the unit of se from bps/Hz to Mbps/20MHz
    def snr2se(self,snr):
        return math.log(1+pow(10,snr/10),2)*20

    def get_valid_user(self):
        valid_users = []#{{{
        # Find valid user set
        for i in range(NUM_UE):
            if self.user_list[i].time_start <= self.current_time <=self.user_list[i].time_end and \
                    self.user_list[i].max_data > self.user_list[i].total_data:
                valid_users.append(self.user_list[i])
        # LIST OF VALID USERS}}}
        return valid_users

    # Calculate reward
    # start & end are 3d grid position.
    def get_reward(self):
        max_reward = 0#{{{
        max_ra = []
        max_psd = []
        max_user_list = []
        for user in self.user_list:
            # initial psd of users
            user.psd = POWER/BANDWIDTH
            # SNR in dBm
            user.pathloss = self.get_pathloss(self.position, user)
            user.snr = self.psd2snr(user.psd, user.pathloss)
            # Mbit/s/20MHz
            user.se = self.snr2se(user.snr)

        # Iteration of all possible user set.
        # Size of power set is 2^(size of set)
        # i is a decimal index of an element of power set
        # ex) Size of valid_users: 5 --> Size of power set = 2^5
        # if i=5 = 00101 (2) --> candidate_valid_user_list = [valid_users[0], valid_users[2]]
        # for more explanation, https://gist.github.com/hslyu/4b4267a76cd2a90d22fbb9957c915f1f 
        valid_users = self.get_valid_user()
        for i in range(2**len(valid_users)):
            candidate_valid_user_list = []

            # Convert i to the candidate_valid_user_list
            for j in range(len(valid_users)): 
                if i & 1<<j:
                    candidate_valid_user_list.append(valid_users[j])

            # Check feasibility of the candidate_valid_user_list
            required_resource = 0
            for user in candidate_valid_user_list:
                required_resource += user.datarate/user.se
            # If required resource exceeds available bandwidth, it is infeasible
            if required_resource >= BANDWIDTH:
                continue
            if len(candidate_valid_user_list) == 0:
                continue

#            print("Current test user set:", candidate_valid_user_list)
            candidate_reward = 0
            ra = self.kkt_ra(candidate_valid_user_list)
#            print("ra:",ra)
            prev_candidate_reward = self.objective_function([user.psd for user in candidate_valid_user_list],\
                    candidate_valid_user_list)
            count=0
            while True:
                count += 1
                psd = self.kkt_psd(candidate_valid_user_list) # return psd
                ra = self.kkt_ra(candidate_valid_user_list) # return ra
                candidate_reward = self.objective_function(psd, candidate_valid_user_list)
                if abs(prev_candidate_reward-candidate_reward) < THRESHOLD or\
                    count > 10:
                    break;
                prev_candidate_reward = candidate_reward

            if max_reward < candidate_reward:
                max_reward = candidate_reward
                max_ra = ra
                max_psd = psd
                max_user_list = candidate_valid_user_list

        for user, ra in zip(max_user_list, max_ra):
            self.user_list[user.id].total_data += ra*user.se
#        print("Current time:", self.current_time)
#        print("max_psd:", max_psd)
#        print("power:", sum([psd*ra for psd, ra in zip(max_psd,max_ra)]))
#        print("Resource:", max_ra)
#        print("Sum resource:", sum(max_ra))
#        print("Throughput List", [user.total_data//10 for user in self.user_list])
        # find max obj. ftn value (reward) and resource and power control}}}
        return max_reward

    def kkt_ra(self, user_list):
        # Use projected RMSProp for KKT dual problem{{{

        # return 2+len(user_list) length gradient
        def regularized_gradient(lambda_1, lambda_2, mu_list, user_list):
            grad_list = []#{{{
            grad_lambda_1 = 1.
            grad_lambda_2 = 1.
            for i, user in enumerate(user_list):
                grad_lambda_1 += - 1./(lambda_1+user.psd*lambda_2-mu_list[i])\
                        + user.total_data/user.se
                grad_lambda_2 += - user.psd/(lambda_1+user.psd*lambda_2-mu_list[i])\
                        + user.psd*user.total_data/user.se
            grad_list.append(grad_lambda_1)
            grad_list.append(grad_lambda_2)

            for i, user in enumerate(user_list):
                grad_mu_i = 1./(lambda_1+user.psd*lambda_2-mu_list[i])\
                        - user.datarate/user.se - user.total_data/user.se
                grad_list.append(grad_mu_i)#}}}
            return grad_list

        def regularized_dual_function(lambda_1, lambda_2, mu_list, user_list):
            value = 0.#{{{
            for i, user in enumerate(user_list):
                if lambda_1+user.psd*lambda_2-mu_list[i] < 0:
                    print(user.id)
                    print(lambda_1+user.psd*lambda_2-mu_list[i])
                    print(lambda_1)
                    print(user.psd*lambda_2)
                    print(mu_list[i])
                value -= math.log(lambda_1+user.psd*lambda_2-mu_list[i])
                value -= mu_list[i]*(user.datarate/user.se + user.total_data/user.se)
                value += lambda_1*user.total_data/user.se
                value += lambda_2*user.psd*user.total_data/user.se
            value += lambda_1
            value += lambda_2#}}}
            return value

        # Initialize arguments of the dual function
        lambda_1 = .5
        lambda_2 = .5
        mu_list=[.1]*len(user_list)
        weight_list = [0]*(2+len(mu_list))

        import time
        start = time.time()
        count=0
        while True:
            count+=1
            # Save before update
            prev_lambda_1 = lambda_1
            prev_lambda_2 = lambda_2
            prev_mu_list = mu_list

            # RMSPROP update rule
            grad_list = regularized_gradient(lambda_1, lambda_2, mu_list, user_list)
            weight_list = [weight + grad_list[i]**2 for i, weight in enumerate(weight_list)]
            lambda_1 = max(0, lambda_1 - STEP_SIZE/(weight_list[0]+EPSILON)**.5*grad_list[0])
            lambda_2 = max(0, lambda_2 - STEP_SIZE/(weight_list[1]+EPSILON)**.5*grad_list[1])
#            mu_list = [ min(lambda_1+user_list[idx].psd*lambda_2 - 0.001, \
#                    max(0, mu - 10*STEP_SIZE/(weight_list[2+idx]+EPSILON)**.5*grad_list[2+idx]) for idx, mu in enumerate(mu_list)]
            mu_list = [ min(lambda_1+user_list[idx].psd*lambda_2 - .1, \
                    max(0, mu - 1.3*STEP_SIZE/(weight_list[2+idx]+EPSILON)**.5*grad_list[2+idx])) for idx, mu in enumerate(mu_list)]

            # Print option
#            if count%2000==0 :
#                print("Gradient:", grad_list)
#                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
#                print("Input:", [lambda_1, 1e-5*lambda_2]+mu_list )
#                print("Count:",count)
#                print("-------------------------")

            # Stop condition
            gap = regularized_dual_function(prev_lambda_1, prev_lambda_2, prev_mu_list, user_list) \
                   - regularized_dual_function(lambda_1, lambda_2, mu_list, user_list)
            if gap < THRESHOLD:
#                print("Gradient:", grad_list)
#                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
#                print("Input:", [lambda_1, 1e-5*lambda_2]+mu_list )
#                print("Count:",count)
#                print("-------------------------")
                break;


        resource_list = [max(user.datarate/user.se,BANDWIDTH/(lambda_1+user.psd*lambda_2-mu_list[idx])-user.total_data/user.se) for idx, user in enumerate(user_list)]
#        if count > 500:
#        print("Final values:", [lambda_1, lambda_2]+mu_list )
#        print("Count:", count)
#        print("Time:", time.time()-start)
#        print("Origianl resource list, sum:",resource_list, sum(resource_list))
#        sum_resource = sum(resource_list)
#        resource_list = [resource + (BANDWIDTH-sum_resource)/len(resource_list) for resource in resource_list]
#        resource_list = [BANDWIDTH*resource/sum_resource for resource in resource_list]
        # Normalize the resource
        diff_list = [resource - user.datarate/user.se for resource, user in zip(resource_list,user_list)]
        sum_diff_list = sum(diff_list)
        if sum_diff_list > EPSILON:
            available_bandwidth = BANDWIDTH - sum([user.datarate/user.se for user in user_list])
            resource_list = [user.datarate/user.se + available_bandwidth*diff/sum_diff_list for diff,user in zip(diff_list,user_list)]
        else:
            sum_resource = sum(resource_list)
            resource_list = [BANDWIDTH*resource/sum_resource for resource in resource_list]
#        print("Normalized resource list, sum:", resource_list, sum(resource_list))
#        print("self, self.parent:", self, self.parent)
#        print("-------------------------")
        for user, resource in zip(user_list, resource_list):
            user.ra = resource
        #}}}
        return resource_list

    def kkt_psd(self, user_list):
        #{{{

        lambda_list=[]
        # 10^(-\xi/10)/n_0
        pl_over_noise_list = []
        c_rho_list = []
        for idx, user in enumerate(user_list):
            pl_over_noise = 10**(user.pathloss/NOISE_DENSITY/10.)
            pl_over_noise_list.append(pl_over_noise)
            # user.ra = unit of 20MHz = 20 * unit of MHz
            c_rho_list.append((pow(2,user.datarate/(user.ra*20))-1)/pl_over_noise)
            lambda_list.append(pl_over_noise/pow(2,user.datarate/(user.ra*20))/user.total_data)
        # return the sorted index list of the user lambda_list
        sorted_index = sorted(range(len(user_list)), key=lambda k: lambda_list[k])

        max_objective_value = 0
        max_psd_list = []
        max_sorted_index = []
        for i in range(len(user_list)):
            term_1 = 0
            term_2 = 0
            term_3 = 0
            # Indexes of users such that lambda_psd < lambda_list[idx]
            # If lambda_psd < lambda_list[idx], mu_i = 0
            for idx in sorted_index[i:]:
                term_1 += user_list[idx].ra/user_list[idx].total_data
                term_2 += user_list[idx].ra/pl_over_noise_list[idx]
            # Indexes of users such that lambda_psd > lambda_list[idx]
            # If lambda_psd < lambda_list[idx], mu_i != 0
            for idx in sorted_index[:i]:
                term_3 += user_list[idx].ra*c_rho_list[idx]
            lambda_psd = term_1/(POWER-term_3+term_2)
#            print("lambda psd:", lambda_psd)
#            print("lambda list:", lambda_list)

            candidate_psd_list = c_rho_list.copy()
            for idx in sorted_index[i:]:
                candidate_psd_list[idx] = 1/(user.total_data*lambda_psd)-1/pl_over_noise_list[idx]

            # If this lambda_psd is valid
            if (i>=1 and lambda_list[i-1] <= lambda_psd <= lambda_list[i]) or \
                    (i==0 and 0 <= lambda_psd <= lambda_list[i]) or \
                    (i==len(user_list)-1 and lambda_list[i] < lambda_psd) :
#                print("candidate_psd_list:", candidate_psd_list)
                value = self.objective_function(candidate_psd_list, user_list)
                if max_objective_value < value:
                    max_psd_list = candidate_psd_list

        for user, psd in zip(user_list, max_psd_list):
            user.psd = psd
#        print('Minimum power:',sum([user.ra*c_rho_list[idx] for idx,user in enumerate(user_list)]))
#        print("user:",user_list)
#        print("muzero user:",[ user_list[idx] for idx in sorted_index[i:]])
#        print("orig:",[user.psd for user in user_list])
#        print("psd :", candidate_psd_list)
#        print("c_rho :", c_rho_list)
#        print("diff:", [candidate_psd_list[a]-user.psd for a,user in enumerate(user_list)])
#        print(sum([user.ra*candidate_psd_list[idx] for idx,user in enumerate(user_list)]))
#        print('----')
        #}}}
        return max_psd_list

    def objective_function(self, psd_list, user_list):
        value=0#{{{
        for psd, user in zip(psd_list, user_list):
            snr = self.psd2snr(psd, user.pathloss)
            se = self.snr2se(snr)
            if 1+user.ra*20*se/user.total_data<0:
                print("psd, user.ra, se, user.total_data:", psd, user.ra, se, user.total_data)
                print([user.ra for user in self.user_list])
            # user.ra = unit of 20MHz = 20 * unit of MHz
            value += math.log(1+(user.ra*20)*se/user.total_data,2)#}}}
        return value

#class UAV
class TrajectoryTree:
    def __init__(self, root):
        self.root = root
        # Make adjacent node tree with TREE_DEPTH
        # Parenthesize self.root for recursive function implementation.
        self.recursive_find_leaf([self.root], node_level=0) 
#        for i in range(100):
#            print("first node reward calculation is done")
#            print(root.leafs)

    # Depth First Search
    def DFS(self, current):
        # Recursive part{{{
        if len(current.leafs) == 0:
            return [current], current.reward

        # Recursive here
        # Theorem : Subpath of optimal path is optimal of subpath.
        max_path = []
        # if reward return of self.DFS(node) MUST BE LARGER THAN -INF
        max_reward = -INF
        for i in range(len(current.leafs)):
            next_node = current.leafs[i]
            path, reward = self.DFS(next_node)
            if max_reward < reward:
                max_path = path
                max_reward = reward
        max_path.append(current)
#}}}
        return max_path, max_reward+current.reward

    def recursive_find_leaf(self, leafs, node_level):
        # Terminate recursive function when it reaches to depth limit{{{
        if node_level == TREE_DEPTH+1:
            # Save the leafs of DEPTH==TREE_DEPTH
            return
        node_level += 1
        # Find leafs of leaf
        for leaf in leafs:
            # Check whether leafs of leaf are already found.
            if len(leaf.leafs) == 0:
                leaf.leafs = self.find_leaf(leaf)
            self.recursive_find_leaf(leaf.leafs, node_level)#}}}

    def find_leaf(self, node):
        leafs = []#{{{
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
                    if np.linalg.norm(GRID_SIZE*np.array([x,y,z])) <= VEHICLE_VELOCITY*TIME_STEP:
                        # add all node with distance np.linalg.norm(GRID_SIZE*np.array([x,y,z]))
                        for i in {-1,1}:
                            for j in {-1,1}:
                                for k in {-1, 1}:
                                    # calculate leaf position
                                    leaf_position = node.position + GRID_SIZE*np.array([x*i, y*j, z*k])
                                    # Check whether 1. the position is available and 2. already appended.
                                    if self.isAvailable(leaf_position) and tuple(leaf_position) not in appended_table:
                                        leaf = TrajectoryNode(leaf_position, parent=node)
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
            x +=1
        #### while x end}}}
        return leafs

    def isAvailable(self, position):
        # If the position is in the map, return true.{{{
        if 0 <= position[0] <= MAP_WIDTH and \
           0 <= position[1] <= MAP_WIDTH and \
           MIN_ALTITUDE <= position[2] <= MAX_ALTITUDE:
               return True
        # If there's any forbidden place in the map, write code here

        # otherwise return false.
        return False#}}}

    def pathfinder(self, time_step):
        PATH = []#{{{
        for i in range(MAX_TIME):
            PATH.append(self.root)
            # DFS return = reversed path, reward
            # a.DFS(a.root)[0] = reversed path = [last leaf, ... , first leaf, root]
            # reversed path[-2] = first leaf = next root
            start=time.time()
            self.root = self.DFS(self.root)[0][-2]
            self.recursive_find_leaf([self.root], 1) 
            print("current time:", i)
            print("1 Unit recursive tree elapsed time:",time.time()-start)#}}}
            self.root.get_info()

        return PATH


if __name__ =="__main__":
    import time
    position = np.array([random.randint(0, MAP_WIDTH)//10*10,
                         random.randint(0, MAP_WIDTH)//10*10,
                         random.randint(MIN_ALTITUDE, MAX_ALTITUDE)//10*10])
    # Initial grid position of UAV
    root = TrajectoryNode(position)
    user_list = []
    for i in range(NUM_UE):
        tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
        user = User(i,
                random.randint(0, MAP_WIDTH),
                random.randint(0, MAP_WIDTH),
                random.randint(0, MAX_TIME-tw_size), 
                tw_size,
                random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1]))
        user_list.append(user)
    root.user_list = user_list

    tree = TrajectoryTree(root)

    PATH = tree.pathfinder(MAX_TIME)
    reward = 0
    for leaf in PATH:
       reward += leaf.reward
    print(PATH)
    print(reward)
#    print TREE_DEPTH, time.time()-start}}}
