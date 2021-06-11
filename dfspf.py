#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import random
import math

#### To do list
####
# Constant for map
MAP_WIDTH = 200 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 100 # meter
GRID_SIZE = 10 # meter
# Constant for UAV
VEHICLE_VELOCITY = 10. # m/s
TIME_STEP = 1 # s
MAX_TIME = 30 # s
# Constant for node
NUM_UE = 5
TIME_WINDOW_SIZE = [10,15]
# Tree depth
TREE_DEPTH = 5
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

#args = parser.parse_args()
#
#args_dumped = {'map_width': 200, }
#user = User(1,2,3,4,5, **args_dumped)

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

class UAVBaseStation:
    def __init__(self, num_ue=NUM_UE):
        self.num_ue = num_ue
        # List of pathloss of all UEs
        self.pathloss = []

        # Generate array of node with size num_node
        self.ue = []
        for i in range(self.num_ue):
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

# TEST CODE FOR PATHLOSS
        self.ue[0].position = [0,0]
        self.UAV_position = [0,80,60]

    # Distance between the UAV and i-th User
    def distance_from_UAV(self, i):
        return math.sqrt( (self.UAV_position[0] - self.ue[i].position[0])**2 +\
                 (self.UAV_position[1] - self.ue[i].position[1])**2 + self.UAV_position[2]**2 )

    # Caculate pathloss
    def get_pathloss(self):
        # list of distance of all ue
        distance = [self.distance_from_UAV(i) for i in range(self.num_ue)]
        for i in range(self.num_ue):
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
                        <=self.ue[i].time_end else 0 for i in range(self.num_ue)]
        # Allocate uniform power density to valid user
        density = POWER/BANDWIDTH
        # SNR in dBm
        snr = [10*math.log10(density) - self.pathloss[i] - NOISE for i in range(self.num_ue)]
        # For subset of valid user
            # obj. ftn. 
            # while converge
                # KKT of frequency
                # KKT of power density
            # find max obj. ftn value (reward) and resource and power control
        # return reward, resource allocation, power control, admission control at current time

if __name__=="__main__":
    # Generate random graph
    a=UAVBaseStation()
    a.get_pathloss()
    a.get_reward()
