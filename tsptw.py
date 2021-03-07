#! /usr/bin/env python3
# Travelling Salesman Problem with Time Window (revised for UAV comm.)
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import random
import numpy as np

#### To do list
# 1. Discritize time slot
#   - EAT = round up, LDT = round down?
# 2. Realize the recurrence
# 3. Formulate the reward from the resource allocaiton. (TBD)
####

VEHICLE_VELOCITY = 10 # m/s
VEHICLE_SERVICE_TIME = 2 # s
INF = 99999999

class TSPNode:
    def __init__(self, node_id, loc_x, loc_y, time_start, time_end):
        self.node_id = node_id
        self.location = np.array([loc_x, loc_y])
        self.time_start = time_start
        self.time_end = time_end
    
    def __str__(self):
        return "id: {}, (x,y) : ({:.2f}, {:.2f}), tw : [{}, {}]".format(
            self.node_id, self.location[0], self.location[1], self.time_start, self.time_end)
    
    def __repr__(self):
        return "{}".format(self.node_id)

class TSPGraph:
    def __init__(self, num_node, time_window_size, 
            x_min = 0, x_max = 200, y_min = 0, y_max = 200):
        self.num_node = num_node
        self.time_window_size = time_window_size

        # Generate array of node with size num_node
        self.node_list = []
        for i in range(num_node):
            time_start = random.randint(0,100)
            node = TSPNode(i,
                    random.randint(x_min, x_max),
                    random.randint(y_min, y_max),
                    time_start, 
                    time_start+time_window_size)
            self.node_list.append(node)
        
        # Generate 2D array of distance between i-th and j-th nodes.
        self.distance = np.zeros([self.num_node, self.num_node])
        self.get_distance()

        # Space to save the dynamic programming result
        # cost_cache.shape == (2^num_node, num_node, time_window_size)
        self.cost_cache = [[[INF for _ in range(self.time_window_size]
                                 for _ in range(self.num_node)]
                                 for _ in range(1<<self.num_node)]

    def get_distance(self):
        for i in range(self.num_node):
            for j in range(i, self.num_node):
                    self.distance[i, j] = np.linalg.norm(self.node_list[i].location - self.node_list[j].location)
                    self.distance[j, i] = self.distance[i, j]

    # The earliest arrival time from node i to node j.
    def EAT(self, i, j):
        return self.node_list[i].time_start + VEHICLE_SERVICE_TIME + self.distance[i][j]/VEHICLE_VELOCITY

    # The latest feasible departure time from i to j.
    def LDT(self, i, j):
        return self.node_list[j].time_end - VEHICLE_SERVICE_TIME - self.distance[j][i]/VEHICLE_VELOCITY

    # The first time value that the vehicle travles all nodes of set S and ends at i-th node.
    def first(self, S, i):
        for j in range(self.time_window_size)
            if self.cost_cache[S][i][j] != INF:
                return j

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
    # t is an integer time slot
    def min_path_cost(self, S, j, t)
        # Initial state of the reccurence
        if S == (1<<0) + (1<<j):
            # return cost(0,j)
            return self.distance[0,j]

        if self.cost_cache[j][S][t] != None:
            return cost_cache[j][S][t]
        else:
            for i in range(self.num_node):
                

        
    def __print_test__(self):
        print(self.distance)
        print("Node example 1 :::::       {}".format(self.node_list[1]))
        print("Node example 2 :::::       {}".format(self.node_list[2]))
        print("Distance 1,2   :::::       {}".format(self.distance[1,2]))
        print("EAT(1,2)       :::::       {}".format(self.EAT(1,2)))
        print("LDT(1,2)       :::::       {}".format(self.LDT(1,2)))
        print("before(1)      :::::       {}".format(self.before(1)))

if __name__=="__main__":
    a=TSPGraph(5, 20)
    a.__print_test__()
