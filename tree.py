#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import numpy as np 
import random
import math
import time

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
NUM_UE = 20
TIME_WINDOW_SIZE = [30,30]
DATARATE_WINDOW = [35, 60] # Requiring datarate Mb/s
INITIAL_THROUGHPUT = 30*1024**2 # Mb
# Tree depth
TREE_DEPTH = 5
# Constant for wirless communication
FREQUENCY = 2.0*1e9 # Hz
LIGHTSPEED = 3*1e8 # m/s
BANDWIDTH = 2e7 # MHz
POWER = 200. # mW
NOISE = -174 # dBm, noise spectral density, https://en.wikipedia.org/wiki/Johnson%E2%80%93Nyquist_noise 
LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 40 # dB, excessive pathloss of nlos link
SURROUNDING_A = 9.64 # Envrionmental parameter for probablistic LOS link
SURROUNDING_B = 0.04 # Envrionmental parameter for probablistic LOS link
# Optimization hyperparameter
#GAMMA = 0.12
#STEP_SIZE = 4e-3
EPSILON = 1e-3
GAMMA = 0.2
STEP_SIZE = 1e-2
THRESHOLD = 1e-8
# etc
INF = 1e8-1

class User:
    def __init__(self, user_id, loc_x, loc_y, time_start, tw_size, datarate):#{{{
        self.user_id = user_id
        self.position = [loc_x, loc_y]
        self.time_start = time_start
        self.time_end = self.time_start + tw_size
        # Requiring datarate b/s
        self.datarate = datarate*1024*1024
        # Temporary variables for easy implementation
        self.se = 0
        self.ra = 0
        self.psd = 0
        self.snr = 0
        # Total received throughput
        # Initial is not zero for our formulation
        self.total_throughput = INITIAL_THROUGHPUT
    
    def __str__(self):
        return "id: {}, (x,y) : ({:.2f}, {:.2f}), tw : [{}, {}], tw_size : {}".format(
            self.node_id, self.location[0], self.location[1], self.time_start, self.time_end, self.tw_size)
    
    def __repr__(self):
        return "{}".format(self.user_id)#}}}

class TrajectoryNode:
    def __init__(self, position, parent=None):
        # link
        self.parent = parent
        self.leafs = []
        
        # value
        self.position = position
        self.user_list = []
        if parent is not None:
            self.user_list = parent.user_list.copy()
            self.current_time = parent.current_time+1
            self.reward = self.get_reward()
#            self.reward = self.get_random_reward()
            start = time.time()
#            print("Reward is calculated from node and elapsed time:", self.position, time.time()-start)
        # If this node is root
        else:
            self.reward = 0
            self.current_time = 0

    def get_info(self):
        print("User throughput list")
        print([user.total_throughput//1024**2 for user in self.user_list])

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

    def psd2snr(self, psd, pathloss):
        return 10*math.log10(psd) - pathloss - NOISE
    def snr2se(self,snr):
        return math.log(1+pow(10,snr/10),2)

    def get_valid_user(self):
        valid_users = []#{{{
        # Find valid user set
        for i in range(NUM_UE):
            if self.user_list[i].time_start <= self.current_time <=self.user_list[i].time_end:
                valid_users.append(self.user_list[i])
        # LIST OF VALID USERS}}}
        return valid_users

    # Calculate reward
    # start & end are 3d grid position.
    def get_reward(self):
        max_reward = 0#{{{
        max_ra = []
        max_psd = []
        max_user_set = []
        for user in self.user_list:
            # initial psd of users
            user.psd = POWER/BANDWIDTH
            # SNR in dBm
            user.pathloss = self.get_pathloss(self.position, user)
            user.snr = self.psd2snr(user.psd, user.pathloss)
            # bit/s/Hz
            user.se = self.snr2se(user.snr)

        # Iteration of all possible user set.
        # Size of power set is 2^(size of set)
        # i is a decimal index of an element of power set
        # ex) Size of valid_users: 5 --> Size of power set = 2^5
        # if i=5 = 00101 (2) --> candidate_valid_user_set = [valid_users[0], valid_users[2]]
        # for more explanation, https://gist.github.com/hslyu/4b4267a76cd2a90d22fbb9957c915f1f 
        valid_users = self.get_valid_user()
        for i in range(2**len(valid_users)):
            candidate_valid_user_set = []

            # Convert i to the candidate_valid_user_set
            for j in range(len(valid_users)): 
                if i & 1<<j:
                    candidate_valid_user_set.append(valid_users[j])

            # Check feasibility of the candidate_valid_user_set
            required_resource = 0
            for user in candidate_valid_user_set:
                required_resource += user.datarate/user.se
            # If required resource exceeds available bandwidth, it is infeasible
            if required_resource >= BANDWIDTH:
                continue
            if len(candidate_valid_user_set) == 0:
                continue

            print("Current test user set:", candidate_valid_user_set)
            self.kkt_ra(candidate_valid_user_set)
            candidate_reward = 0
            prev_candidate_reward = self.objective_function([user.psd for user in candidate_valid_user_set],\
                    candidate_valid_user_set)
            while True:
                ra = self.kkt_ra(candidate_valid_user_set) # return ra
                psd = self.kkt_psd(candidate_valid_user_set) # return psd
                candidate_reward = self.objective_function(psd, candidate_valid_user_set)
                if abs(prev_candidate_reward-candidate_reward) < EPSILON:
                    break;
                prev_candidate_reward = candidate_reward

            if max_reward < candidate_reward:
                max_reward = candidate_reward
                max_ra = ra
                max_psd = psd
                max_user_set = candidate_valid_user_set

        for user, ra in zip(max_user_set, max_ra):
            self.user_list[user.user_id].total_throughput += ra*user.se
        self.get_info()
        # find max obj. ftn value (reward) and resource and power control}}}
        return max_reward

    def kkt_ra(self, user_set):
        # Use projected RMSProp for KKT dual problem{{{

        # return 2+len(user_set) length gradient
        def regularized_gradient(lambda_1, lambda_2, mu_list, user_set):
            grad_list = []#{{{
            grad_lambda_1 = 1
            grad_lambda_2 = POWER/BANDWIDTH
            for i, user in enumerate(user_set):
                grad_lambda_1 += - 1/(lambda_1+user.psd*lambda_2-mu_list[i])\
                        + user.total_throughput/user.se/BANDWIDTH
                grad_lambda_2 += - user.psd/(lambda_1+user.psd*lambda_2-mu_list[i])\
                        + user.total_throughput/user.se/BANDWIDTH
            grad_list.append(grad_lambda_1)
            grad_list.append(grad_lambda_2)

            for i, user in enumerate(user_set):
                grad_mu_i = 1/(lambda_1+user.psd*lambda_2-mu_list[i])\
                        - user.datarate/user.se/BANDWIDTH - user.total_throughput/user.se/BANDWIDTH
                grad_list.append(grad_mu_i)#}}}
            return grad_list

        def regularized_dual_function(lambda_1, lambda_2, mu_list, user_set):
            value = 0#{{{
            for i, user in enumerate(user_set):
                if lambda_1+user.psd*lambda_2-mu_list[i] < 0:
                    print(user.user_id)
                    print(lambda_1+user.psd*lambda_2-mu_list[i])
                    print(lambda_1)
                    print(user.psd*lambda_2)
                    print(mu_list[i])
                value -= math.log(lambda_1+user.psd*lambda_2-mu_list[i])
                value -= mu_list[i]*(user.datarate/user.se + user.total_throughput/user.se)/BANDWIDTH
                value += lambda_1*user.total_throughput/user.se/BANDWIDTH
                value += lambda_2*user.psd*user.total_throughput/user.se/BANDWIDTH
            value += BANDWIDTH*lambda_1/BANDWIDTH
            value += POWER*lambda_2/BANDWIDTH#}}}
            return value

        # Initialize arguments of the dual function
        GAMMA = 0.01
        STEP_SIZE = 1e-1
        lambda_1 = 10
        lambda_2 = 1000
        mu_list=[0]*len(user_set)
        weight_list = [0]*(2+len(mu_list))

#        import time
#        start = time.time()
        count=0
        while True:
            count+=1
            # Save before update
            prev_lambda_1 = lambda_1
            prev_lambda_2 = lambda_2
            prev_mu_list = mu_list

            # RMSPROP update rule
            grad_list = regularized_gradient(lambda_1, lambda_2, mu_list, user_set)
            weight_list = [GAMMA*weight + (1-GAMMA)*grad_list[i]**2 for i, weight in enumerate(weight_list)]
            lambda_1 = max(0, lambda_1 - STEP_SIZE/(weight_list[0]+EPSILON)**.5*grad_list[0])
            lambda_2 = max(0, lambda_2 - STEP_SIZE/(weight_list[1]+EPSILON)**.5*grad_list[1])
            mu_list = [ max(0, mu - STEP_SIZE/(weight_list[2+idx]+EPSILON)**.5*grad_list[2+idx]) for idx, mu in enumerate(mu_list)]

            # Print option
            if count%200==0 :
                print("Gradient:", grad_list)
                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
                print("Input:", [lambda_1, 1e-5*lambda_2]+mu_list )
#                print("Count:",count)
                print("-------------------------")

            # Stop condition
            gap = regularized_dual_function(prev_lambda_1, prev_lambda_2, prev_mu_list, user_set) \
                   - regularized_dual_function(lambda_1, lambda_2, mu_list, user_set)
            if gap < THRESHOLD:
                print("Gradient:", grad_list)
                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
                print("Input:", [lambda_1, 1e-5*lambda_2]+mu_list )
#                print("Count:",count)
                print("-------------------------")
                break;

#        print("Time:", time.time()-start)

        resource_list = [BANDWIDTH/(lambda_1+user.psd*lambda_2-mu_list[idx])-user.total_throughput/user.se for idx, user in enumerate(user_set)]
        print("Final values:", [lambda_1, lambda_2]+mu_list )
        print("Resource list:", resource_list)
        print("Sum resource", sum(resource_list))
        print("Count:", count)
        print("-------------------------")
        resource_list = [resource+(BANDWIDTH-sum(resource_list))/len(resource_list) for resource in resource_list]
        for user, resource in zip(user_set, resource_list):
            user.ra = resource
        #}}}
        return resource_list

    def kkt_psd(self, user_set):
        #{{{

        lambda_list=[]
        # 10^(-\xi/10)/n_0
        pl_over_noise_list = []
        c_rho_list = []
        for idx, user in enumerate(user_set):
            pl_over_noise = 10**(user.snr/10)/user.psd
            pl_over_noise_list.append(pl_over_noise)
            c_rho_list.append( (pow(2,user.datarate/user.ra)-1)/pl_over_noise )
            lambda_list.append(pl_over_noise/pow(2,user.datarate/user.ra)/user.total_throughput)
        # return the sorted index list of the user lambda_list
        sorted_index = sorted(range(len(user_set)), key=lambda k: lambda_list[k])

        max_objective_value = -1
        max_psd_list = []
        max_sorted_index = []
        for i in range(len(user_set)):
            term_1 = 0
            term_2 = 0
            term_3 = 0
            # Indexes of users such that lambda_psd < lambda_list[idx]
            # If lambda_psd < lambda_list[idx], mu_i = 0
            for idx in sorted_index[i:]:
                term_1 += user_set[idx].ra/user_set[idx].total_throughput
                term_2 += user_set[idx].ra/pl_over_noise_list[idx]
            # Indexes of users such that lambda_psd > lambda_list[idx]
            # If lambda_psd < lambda_list[idx], mu_i != 0
            for idx in sorted_index[:i]:
                term_3 += user_set[idx].ra*c_rho_list[idx]
            lambda_psd = term_1/(POWER-term_3+term_2)

            candidate_psd_list = c_rho_list.copy()
            for idx in sorted_index[i:]:
                candidate_psd_list[idx] = 1/(user.total_throughput*lambda_psd)-1/pl_over_noise_list[idx]

            # If this lambda_psd is valid
            if (i>=1 and lambda_list[i-1] <= lambda_psd <= lambda_list[i]) or \
                    (i==0 and 0 <= lambda_psd <= lambda_list[i]):
                value = self.objective_function(candidate_psd_list, user_set)
                if max_objective_value < value:
                    max_psd_list = candidate_psd_list

        for user, psd in zip(user_set, max_psd_list):
            user.psd = psd
#            print('Minimum power:',sum([user.ra*c_rho_list[idx] for idx,user in enumerate(user_set)])){{{
#            print("user:",user_set)
#            print("muzero user:",[ user_set[idx] for idx in sorted_index[i:]])
#            print("orig:",[user.psd for user in user_set])
#            print("psd :", candidate_psd_list)
#            print("c_rho :", c_rho_list)
#            print("diff:", [candidate_psd_list[a]-user.psd for a,user in enumerate(user_set)])
#            print(sum([user.ra*candidate_psd_list[idx] for idx,user in enumerate(user_set)]))
#            print('----')}}}
#}}}
        return max_psd_list

    def objective_function(self, psd_list, user_set):
        value=0#{{{
        for psd, user in zip(psd_list, user_set):
            snr = self.psd2snr(psd, user.pathloss)
            se = self.snr2se(snr)
            value += math.log(1+user.ra*se/user.total_throughput,2)#}}}
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
        if node_level == TREE_DEPTH:
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
            path = self.DFS(self.root)[0]
            self.root = path[-2]
#            self.root = self.DFS(self.root)[0][-2]
            self.recursive_find_leaf([self.root], 1) 
            print("current time:", i)
            print("1 Unit recursive tree elapsed time:",time.time()-start)#}}}

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
