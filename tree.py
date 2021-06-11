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
MAX_TIME = 30
# Constant for map
MAP_WIDTH = 200 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 100 # meter
GRID_SIZE = 10 # meter
# Constant for user
NUM_UE = 5
TIME_WINDOW_SIZE = [10,15]
# Tree depth
TREE_DEPTH = 10
# Constant for wirless communication
FREQUENCY = 2.0*1e9 # Hz
LIGHTSPEED = 3*1e8 # m/s
BANDWIDTH = 1e7 # 10MHz
POWER = 200 # mW
NOISE = -174 # dBm, noise spectral density
LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 10 # dB, excessive pathloss of nlos link
SURROUNDING_A = 7.37 # Envrionmental parameter for probablistic LOS link
SURROUNDING_B = 0.06 # Envrionmental parameter for probablistic NLOS link

class User:
    def __init__(self, node_id, loc_x, loc_y, time_start, tw_size):
        self.node_id = node_id
        self.position = [loc_x, loc_y]
        self.time_start = time_start
        self.time_end = self.time_start + tw_size
    
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
        # Initial grid position of UAV
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
                    tw_size)
            self.ue.append(user)

        # [x, y, z]
        self.UAV_position = [GRID_SIZE*random.randint(0, MAP_WIDTH/GRID_SIZE),
                                GRID_SIZE*random.randint(0, MAP_WIDTH/GRID_SIZE),
                                GRID_SIZE*random.randint(MIN_ALTITUDE/GRID_SIZE, MAX_ALTITUDE/GRID_SIZE)]
        self.current_time = 0

    # Depth First Search
    def DFS(self, current):
        # Recursive part
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

        return max_path, max_reward+current.reward


    def recursive_find_leaf(self, leafs, node_level):
        # Terminate recursive function when it reaches to depth limit
        if node_level == TREE_DEPTH:
            # Save the leafs of DEPTH==TREE_DEPTH
            return
        node_level += 1
        # Find leafs of leaf
        for leaf in leafs:
            # Check whether leafs of leaf are already found.
            if len(leaf.leafs) == 0:
                leaf.leafs = self.find_leaf(leaf)
            self.recursive_find_leaf(leaf.leafs, node_level)

    def find_leaf(self, node):
        leafs = []
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
        #### while x end
        return leafs

    def isAvailable(self, position):
        # If the position is in the map, return true.
        if 0 <= position[0] <= MAP_WIDTH and \
           0 <= position[1] <= MAP_WIDTH and \
           MIN_ALTITUDE <= position[2] <= MAX_ALTITUDE:
               return True
        # If there's any forbidden place in the map, write code here

        # otherwise return false.
        return False

    def pathfinder(self, time_step):
        PATH = []
        for i in range(MAX_TIME):
            PATH.append(a.root)
            a.root = a.DFS(a.root)[0][-2]
            a.recursive_find_leaf([a.root], 1) 
        return PATH

    # Distance between the UAV and i-th User
    def distance_from_UAV(self, i):
        return math.sqrt( (self.UAV_position[0] - self.ue[i].position[0])**2 +\
                 (self.UAV_position[1] - self.ue[i].position[1])**2 + self.UAV_position[2]**2 )

    # Caculate pathloss
    def get_pathloss(self):
        # list of distance of all ue
        distance = [self.distance_from_UAV(i) for i in range(NUM_UE)]
        for i in range(NUM_UE):
            angle = math.pi/2 - math.acos(self.UAV_position[2]/distance[i])
            los_prob = 1/(1 + SURROUNDING_A * \
                    math.exp(-SURROUNDING_B*(180/math.pi*angle - SURROUNDING_A)))
            pathloss = 20*math.log10(4*math.pi*FREQUENCY*distance[i]/LIGHTSPEED) + los_prob*LOS_EXCESSIVE + \
                        (1-los_prob)*NLOS_EXCESSIVE
            self.pathloss.append(pathloss)

    # Calculate reward
    # start & end are 3d grid position.
    def get_reward(self):
        # Find valid user set
        valid = [1 if self.ue[i].time_start <= self.current_time \
                        <=self.ue[i].time_end else 0 for i in range(NUM_UE)]
        # Allocate uniform power density to valid user
        density = POWER/BANDWIDTH
        # SNR in dBm
        snr = [10*math.log10(density) - self.pathloss[i] - NOISE for i in range(NUM_UE)]
        # For subset of valid user
            # obj. ftn. 
            # while converge
                # KKT of frequency
                # KKT of power density
            # find max obj. ftn value (reward) and resource and power control
        # return reward, resource allocation, power control, admission control at current time

if __name__ =="__main__":
    import time
    start = time.time()
    random.seed(a=50)
# TEST CODE FOR DFS
    position = np.array([50,50,70])
    a = TrajectoryTree(position)
    PATH = a.pathfinder(MAX_TIME)
    reward = 0
    for leaf in PATH:
       reward += leaf.reward
    print PATH
    print reward
    print TREE_DEPTH, time.time()-start

#    TREE_DEPTH=5
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
#    print TREE_DEPTH,time.time()-start
