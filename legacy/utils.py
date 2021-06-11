#!/usr/bin/python

import math
import numpy as np
import itertools

def distance(a,b):
    return np.linalg.norm(a-b)

def calculate_internal_point(coordinate, center, radius):
    relative_vector = coordinate - center
    return center + radius * relative_vector / distance(coordinate, center)

if __name__ == "__main__":
#### UE location generation ####
    #### These UE location generation can be modularized later ####
    a = np.array([0,0])
    b = np.array([140,0])
    c = np.array([70,0])
################################

#### Permute UE location ####
    ue_locations = [a,b,c]
    visit_orders = list(itertools.permutations(ue_locations))
#############################

    for orders in visit_orders:
        # Initialize 
        total_distance = 0
        current_position = np.array([140,30])
        # Calculate Path length
        prev_position = current_position

        for location in orders:
            # Calculate the path length
            togo_position = calculate_internal_point(current_position, location, 25)
            total_distance += distance(togo_position, current_position)
            print(distance(togo_position, location))

            # Move drone close to the location
            current_position = togo_position
            # Inform
#            print("{}     {} : {:.2f}".format(prev_position, location, distance(prev_position, current_position)))
            prev_position = location
#        print("---")
#        print("Total length of trajectory for path {} is {:.2f}".format(orders, total_distance))
