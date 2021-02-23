#!/usr/bin/python

import math
import numpy as np

def distance(a,b):
    return np.linalg.norm(a-b)

def calculate_internal_point(coordinate, center, radius):
    relative_vector = coordinate - center
    return center + radius * relative_vector / distance(coordinate, center)

if __name__ == "__main__":
    a = numpy.array([0,0])
    b = numpy.array([30,40])
    print(distance(a,b))

