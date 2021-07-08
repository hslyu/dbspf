#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import numpy as np 
import random
import math

# Constant for UAV
VEHICLE_VELOCITY = 10. # m/s
TIME_STEP = 1 # s
MAX_TIME = 200
## Constant for map
MAP_WIDTH = 200 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 100 # meter
GRID_SIZE = 10 # meter
# Constant for user
NUM_UE = 40
TIME_WINDOW_SIZE = [10,15]
DATARATE_WINDOW = [15, 30] # Requiring datarate Mb/s
INITIAL_THROUGHPUT = 30*1024*1024
# Tree depth
TREE_DEPTH = 3
# Constant for wirless communication
FREQUENCY = 2.0*1e9 # Hz
LIGHTSPEED = 3*1e8 # m/s
BANDWIDTH = 2e7 # 20MHz
POWER = 200. # mW
NOISE = -174 # dBm, noise spectral density
LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 40 # dB, excessive pathloss of nlos link
SURROUNDING_A = 9.64 # Envrionmental parameter for probablistic LOS link
SURROUNDING_B = 0.04 # Envrionmental parameter for probablistic LOS link
EPSILON = 1e-3
# Optimization hyperparameter
GAMMA = 0.04
STEP_SIZE = 22e-3
THRESHOLD = 1e-7

class User:
    def __init__(self, node_id, loc_x, loc_y, time_start, tw_size, datarate):
        self.node_id = node_id
        self.position = [loc_x, loc_y]
        self.time_start = time_start
        self.time_end = self.time_start + tw_size
        # Requiring datarate b/s
        self.datarate = datarate*1024*1024
        # Temporary variables for easy implementation
        self.se = 0
        self.ra = 0
        self.psd = 0
        # Total received throughput
        # Initial is not zero for our formulation
        self.total_throughput = INITIAL_THROUGHPUT
    
    def __str__(self):
        return "id: {}, (x,y) : ({:.2f}, {:.2f}), tw : [{}, {}], tw_size : {}".format(
            self.node_id, self.location[0], self.location[1], self.time_start, self.time_end, self.tw_size)
    
    def __repr__(self):
        return "{}".format(self.node_id)

class TrajectoryNode:
    def __init__(self, position, reward=0, parent=None):
        # value
        self.position = position
        self.reward = reward
        # link
        self.parent = parent
        self.leafs = []

    def __repr__(self):
        return "{}".format(self.position)

class TrajectoryTree:
    def __init__(self, position):
        # Initial grid position of UAV{{{
        self.root = TrajectoryNode(position)
        # Make adjacent node tree with TREE_DEPTH
        # Parenthesize self.root for recursive function implementation.
        self.recursive_find_leaf([self.root], 1) 

        # List of pathloss of all UEs
        self.pathloss = []

        # Generate array of node with size num_node
        self.ue = []
        for i in range(NUM_UE):
            tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
            user = User(i,
                    random.randint(0, MAP_WIDTH),
                    random.randint(0, MAP_WIDTH),
                    random.randint(0, MAX_TIME-tw_size), 
                    tw_size,
                    random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1]))
            self.ue.append(user)
#            print(user.position)

        # [x, y, z]
        self.UAV_position = [GRID_SIZE*random.randint(0, MAP_WIDTH/GRID_SIZE),
                                GRID_SIZE*random.randint(0, MAP_WIDTH/GRID_SIZE),
                                GRID_SIZE*random.randint(MIN_ALTITUDE/GRID_SIZE, MAX_ALTITUDE/GRID_SIZE)]
        self.current_time = 10#}}}

    # Depth First Search
    def DFS(self, current):
        # Recursive part{{{
        if len(current.leafs) == 0:
            return [current], current.reward

        # Recursive here
        # Theorem : Subpath of optimal path is optimal of subpath.
        max_path = []
        max_reward = -1
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
        append_table = {}
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
                                    if self.isAvailable(leaf_position) and tuple(leaf_position) not in append_table:
                                        leaf = TrajectoryNode(leaf_position, reward=random.randint(0,10), parent=node)
                                        leafs.append(leaf)
                                        append_table[tuple(leaf_position)] = 0
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
            PATH.append(a.root)
            # DFS return = reversed path, reward
            # a.DFS(a.root)[0] = reversed path = [last leaf, ... , first leaf, root]
            # reversed path[-2] = first leaf = next root
            a.root = a.DFS(a.root)[0][-2]
            a.recursive_find_leaf([a.root], 1) #}}}
        return PATH

    # Distance between the UAV and i-th User
    def distance_from_leaf(self, position, user):
        return math.sqrt( (position[0] - user.position[0])**2 +\
                 (position[1] - user.position[1])**2 + position[2]**2 )

    # Caculate pathloss --> snr --> spectral efficiency
    def get_pathloss(self, position, user):
        distance = self.distance_from_leaf(position, user)
        angle = math.pi/2 - math.acos(position[2]/distance)
        los_prob = 1/(1 + SURROUNDING_A * \
                math.exp(-SURROUNDING_B*(180/math.pi*angle - SURROUNDING_A)))
        pathloss = 20*math.log10(4*math.pi*FREQUENCY*distance/LIGHTSPEED) + los_prob*LOS_EXCESSIVE + \
                    (1-los_prob)*NLOS_EXCESSIVE

        return pathloss

    def get_valid_user(self):
        valid_users = []
        # Find valid user set
        for i in range(NUM_UE):
            if self.ue[i].time_start <= self.current_time <=self.ue[i].time_end:
                valid_users.append(self.ue[i])
#        print(valid_users)
        # LIST OF VALID USERS
        return valid_users

    # Calculate reward
    # start & end are 3d grid position.
    def get_reward(self, position):
        reward = 0
        valid_users = self.get_valid_user()

        # Iteration of all possible user set.
        # Size of power set is 2^(size of set)
        # i is a decimal index of an element of power set
        # ex) Size of valid_users: 5 --> Size of power set = 2^5
        # if i=5 = 00101 (2) --> candidate_valid_user_set = [valid_users[0], valid_users[2]]
        # for more explanation, https://gist.github.com/hslyu/4b4267a76cd2a90d22fbb9957c915f1f 
        for i in range(2**len(valid_users)):
            candidate_reward = 0
            candidate_valid_user_set = []

            # Convert i to the candidate_valid_user_set
            for j in range(len(valid_users)): 
                if i & 1<<j:
                    candidate_valid_user_set.append(valid_users[j])

            # KKT - Resource allocation
            for user in candidate_valid_user_set:
                # initial psd of users
                user.psd = POWER/BANDWIDTH
                # SNR in dBm
                snr = 10*math.log10(user.psd) - self.get_pathloss(position, user) - NOISE
                # bit/s/Hz
                user.se = math.log10(1+pow(10,snr/10))/math.log10(2)

            # Check feasibility of the candidate_valid_user_set
            required_resource = 0
            for user in candidate_valid_user_set:
                required_resource += user.datarate/user.se
            if required_resource >= BANDWIDTH:
                print("infeasible candidate set")
                continue
            print("Required resource:",required_resource)
            print("Candidate valid user set:",candidate_valid_user_set)
            if len(candidate_valid_user_set) != 0:
                self.kkt_ra(candidate_valid_user_set)
#            while abs(prev_candidate_reward-candidate_reward) > EPSILON:
#                ra = KKT_1(psd, candidate_valid_user_set) # return ra
#                psd = KKT_2(ra, candidate_valid_user_set) # return psd
#                prev_candidate_reward = candidate_reward
            # find max obj. ftn value (reward) and resource and power control
        # return reward, resource allocation, power control, admission control at current time

    def kkt_ra(self, user_set):
        # Use projected RMSProp for KKT dual problem{{{

        # return 2+len(user_set) length gradient
        def regularized_gradient(lambda_1, lambda_2, mu_list, user_set):
            grad_list = []#{{{
            grad_lambda_1 = BANDWIDTH/BANDWIDTH
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
#            print("log input:", lambda_1+POWER/BANDWIDTH*lambda_2-mu_list[0])
#            print("value:", lambda_1,POWER/BANDWIDTH*lambda_2,mu_list[0])
            value = 0#{{{
            for i, user in enumerate(user_set):
                value -= math.log(lambda_1+user.psd*lambda_2-mu_list[i])
                value -= mu_list[i]*(user.datarate/user.se + user.total_throughput/user.se)/BANDWIDTH
                value += lambda_1*user.total_throughput/user.se/BANDWIDTH
                value += lambda_2*user.psd*user.total_throughput/user.se/BANDWIDTH
            value += BANDWIDTH*lambda_1/BANDWIDTH
            value += POWER*lambda_2/BANDWIDTH#}}}
            return value

        # Initialize arguments of the dual function
        lambda_1 = 1.1
        lambda_2 = 1
        mu_list=[0.0]*len(user_set)
        weight_list = [0]*(2+len(mu_list))

        print("First input:", [lambda_1, lambda_2]+mu_list )
        print("First input:", [lambda_1, lambda_2]+mu_list )
        print("First input:", [lambda_1, lambda_2]+mu_list )

#        import time
#        start = time.time()
#        count=0
        while True:
#            count+=1
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
#            if count%200==0 :
#                print("Gradient:", grad_list)
#                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
#                print("Input:", [lambda_1, user.psd*lambda_2]+mu_list )
#                print("-------------------------")

            # Stop condition
            gap = regularized_dual_function(prev_lambda_1, prev_lambda_2, prev_mu_list, user_set) \
                    - regularized_dual_function(lambda_1, lambda_2, mu_list, user_set)
            if gap < THRESHOLD:
#                print("Gradient:", grad_list)
#                print("Changes:",[lambda_1-prev_lambda_1, lambda_2-prev_lambda_2]+[b-a for a,b in zip(prev_mu_list, mu_list)])
#                print("Input:", [lambda_1, user.psd*lambda_2]+mu_list )
#                print("-------------------------")
                break;
#            if count > 50000:
#                break;

#        print("Final values:", [lambda_1, lambda_2]+mu_list )
#        print("Count:", count)
#        print("Time:", time.time()-start)

        resource_list = [BANDWIDTH/(lambda_1+user.psd*lambda_2-mu_list[idx])-user.total_throughput/user.se for idx, user in enumerate(user_set)]
        resource_list = [resource+(BANDWIDTH-sum(resource_list))/len(resource_list) for resource in resource_list]
        #}}}
        return resource_list

    def kkt_psd(self, ra):
        
        return power_list 

if __name__ =="__main__":
    import time
    start = time.time()
#    random.seed(a=50)
# TEST CODE FOR DFS
    position = np.array([30,50,70])
    a = TrajectoryTree(position)
    a.get_reward(position)
#    avg = 0
#    for i in range(2000):
#        position = np.array([random.randint(0,MAP_WIDTH),random.randint(0,MAP_WIDTH),random.randint(MIN_ALTITUDE,MAX_ALTITUDE)])
#        a = TrajectoryTree(position)
#        a.get_pathloss()
#        avg += a.get_reward()
#    print(avg/2000)
        
#    PATH = a.pathfinder(MAX_TIME)
#    reward = 0
#    for leaf in PATH:
#       reward += leaf.reward
#    print PATH
#    print reward
#    print TREE_DEPTH, time.time()-start

#    TREE_DEPTH=5{{{
#    start = time.time()
#    position = np.array([50,50,70])
#    a = TrajectoryTree(position)
#    PATH = a.pathfinder(MAX_TIME)
#    reward = 0
#    for leaf in PATH:
#       reward += leaf.reward
#    print PATH
#    print reward
#    print TREE_DEPTH,time.time()-start
#
#    random.seed(a=50)
#    TREE_DEPTH=7
#    start = time.time()
#    position = np.array([50,50,70])
#    a = TrajectoryTree(position)
#    PATH = a.pathfinder(MAX_TIME)
#    reward = 0
#    for leaf in PATH:
#       reward += leaf.reward
#    print PATH
#    print reward
#    print TREE_DEPTH,time.time()-start}}}
