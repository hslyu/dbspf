#! /usr/bin/env python3
import numpy as np
from sklearn.metrics.pairwise import euclidean_distances


class TSPGraph:
    """Generate a Traveling Salesman Problem Graph with Time Window
    """
    def __init__(self, num_ues, range_x, range_y, time_low, time_high, random_state=None):
        self.random_state = random_state
        np.random.seed(self.random_state)

        # Number of nodes
        self.num_ues = num_ues

        # Position of nodes
        self.x = np.random.randint(range_x, size=self.num_ues)
        self.y = np.random.randint(range_y, size=self.num_ues)
        self.coords = np.column_stack((self.x, self.y))

        # Time window of nodes
        self.start_time = np.zeros(self.num_ues)
        self.end_time = np.random.randint(low=time_low, high=time_high, size=self.num_ues)

    def get_distance(self):
        return np.int32(euclidean_distances(self.coords))

if __name__=="__main__":
    a=TSPGraph(5, 20, 20, 20, 30)
    print(a.get_distance())
    print(a.coords)
    print(a.end_time)
