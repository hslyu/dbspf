#! /usr/bin/env python3

# Travelling Salesman Problem with Time Window (revised for UAV comm.)
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import random
import numpy as np
import math

#### To do list
# (OK) Discritize time slot
#        - EAT = round up, LDT = round down?
# (Ongoing) Realize the recurrence
# (TBD) Formulate the reward from the resource allocaiton.
# () Initial vehicle location setting
####

NUM_NODE = 4
VEHICLE_VELOCITY = 10. # m/s
SERVICE_TIME = 0 # s
MAP_SIZE = [0, 30]
TIME_WINDOW_SIZE = [10, 15]
# To prevent node.time_end exceeds MAX_START_TIME, subtract max time window size.
MAX_TIME = 500
MAX_START_TIME = MAX_TIME - TIME_WINDOW_SIZE[1]
INF = 999999

class TSPNode:
    def __init__(self, node_id, loc_x, loc_y, time_start, time_end):
        self.node_id = node_id
        self.location = np.array([loc_x, loc_y])
        self.tw_size = time_end - time_start
        self.time_start = time_start
        self.time_end = time_end
    
    def __str__(self):
        return "id: {}, (x,y) : ({:.2f}, {:.2f}), tw : [{}, {}], tw_size : {}".format(
            self.node_id, self.location[0], self.location[1], self.time_start, self.time_end, self.tw_size)
    
    def __repr__(self):
        return "{}".format(self.node_id)

class TSPGraph:
    def __init__(self, num_node, tw_size = TIME_WINDOW_SIZE, 
            x_range = MAP_SIZE, y_range = MAP_SIZE):
        self.num_node = num_node

        # Generate array of node with size num_node
        self.node_list = []
        for i in range(num_node):
            # Service end time should be within the 
            time_start = random.randint(0, MAX_START_TIME)
            node = TSPNode(i,
                    random.randint(x_range[0], x_range[1]),
                    random.randint(y_range[0], y_range[1]),
                    time_start, 
                    time_start + random.randint(tw_size[0], tw_size[1]))
            self.node_list.append(node)

###check
        self.node_list[0].time_start = 0
        self.node_list[0].time_end   = MAX_TIME
        
        # Generate 2D array of distance between i-th and j-th nodes.
        self.distance = np.zeros([self.num_node, self.num_node])
        self.get_distance()

        # Space to save the dynamic programming result
        self.cost_cache = {}

    def get_distance(self):
        for i in range(self.num_node):
            for j in range(i, self.num_node):
                    self.distance[i, j] = int(math.ceil(np.linalg.norm(self.node_list[i].location - self.node_list[j].location)))
                    self.distance[j, i] = self.distance[i, j]

    # The earliest arrival time from node i to node j.
    def EAT(self, i, j):
        return int(math.ceil(self.node_list[i].time_start + SERVICE_TIME + self.distance[i][j]/VEHICLE_VELOCITY))

    # The latest feasible departure time from i to j.
    def LDT(self, i, j):
        float_LDT = self.node_list[j].time_end - SERVICE_TIME - self.distance[j][i]/VEHICLE_VELOCITY
        return int(float_LDT)

    # The first time value that the vehicle travles all nodes of set S and ends at i-th node.
    def first(self, S, i):
        for t in range(self.node_list[i].tw_size):
            if (S,j,t) in self.cost_cache and self.cost_cache[(S,j,t)] != INF:
                return t
        return -1

    # Set of all nodes which must necessarily be visited before j
    def before(self, j):
        before_node_list = []
        for i in range(self.num_node):
            if self.EAT(i, j) > self.node_list[j].time_end:
                before_node_list.append(self.node_list[i])

        return before_node_list

    # Recurrence equation for the path finding algorithm.
    # Minimum path length of a path which starts at node 0, ends at node j
    # and serves node j at time t or later.
    # S = bit masking of node set. ex)if num_node=5, S = {0,3} = 01001 
    # j is an integer node id.
    # t is an integer ime slot
    def min_path_cost(self, S, j, t):
        # Check feasibility of t
        if t > self.node_list[j].time_end:
            return INF
            
        # Initial state of the reccurence
        if S == (1<<0) + (1<<j):
            self.cost_cache[(S,j,t)] = self.cost(0,j)
            return self.cost(0,j)
        
        if (S,j,t) in self.cost_cache:
            return self.cost_cache[(S,j,t)]

        self.cost_cache[(S,j,t)] = INF
        for i in range(1,self.num_node):
            # If i is element of S,
            if S & 1<<i and i!=j:
                # See note "max_feasible_time"
                max_feasible_time = min(self.node_list[i].tw_size,
                    self.node_list[j].time_start + t - self.node_list[i].time_start - SERVICE_TIME - int(math.ceil(self.distance[i][j]/VEHICLE_VELOCITY)))
#                print(self.node_list[i].time_start, max_feasible_time)
                for t_ in range(self.node_list[i].time_start, max_feasible_time):
                    self.cost_cache[(S,j,t)] = min(self.cost_cache[(S,j,t)], 
                                            self.min_path_cost(S-(1<<j), i, t_) + self.cost(i, j))

        return self.cost_cache[(S,j,t)]


    def cost(self, i, j):
        return self.distance[i][j]

    def path_finding(self, S, j, t):
        path = []
        path.insert(0, j)

        if S == (1<<0) + (1<<j) :
            return 0
        break_flag = False
        for i in range(1,self.num_node):
            for t_ in range(self.node_list[i].time_start,self.node_list[i].time_end):
                if (S-(1<<j), i, t_) in self.cost_cache and self.cost_cache[(S,j,t)] == self.cost_cache[(S-(1<<j), i, t_)] + self.cost(i,j):
                    path.insert(0, path_finding(S-(1<<j), i, t_))
                    break_flag = True
                    break;
            if break_flag:
                break;

        return path

    def __print_test__(self):
        print(self.distance)
#        print("Node example 1  :::    {}".format(self.node_list[1]))
#        print("Node example 2  :::    {}".format(self.node_list[2]))
#        print("Distance 1,2    :::    {}".format(self.distance[1,2]))
#        print("EAT(1,2)        :::    {}".format(self.EAT(1,2)))
#        print("LDT(1,2)        :::    {}".format(self.LDT(1,2)))
#        print("before(1)       :::    {}".format(self.before(1)))
#        print("cost            :::    {}".format((self.cost_cache[1<<self.num_node-1][1][1])))
#        print("cost list shape:::::     {}".format(len(self.cost_cache)))


if __name__=="__main__":
    # Generate random TSP graph
    a=TSPGraph(NUM_NODE)

    # Print distance array
    a.__print_test__()

    # Print Basic Information
    print(((1<<NUM_NODE)-1,NUM_NODE-1,10))
    for i in range(NUM_NODE):
        print a.node_list[i].time_start, a.node_list[i].time_end

    # Search all node
    for node_idx in range(1,NUM_NODE):
        for time_idx in range(a.node_list[node_idx].time_start, a.node_list[node_idx].time_end):
            a.min_path_cost((1<<NUM_NODE)-1, node_idx, time_idx)
            print(a.cost_cache[((1<<NUM_NODE)-1, node_idx, time_idx)])

    for key in a.cost_cache.keys():
        print key, a.cost_cache[key]
