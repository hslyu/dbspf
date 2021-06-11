import numpy as np 
import random

# Constant for UAV
VEHICLE_VELOCITY = 10. # m/s
TIME_STEP = 1 # s
MAX_TIME = 30
# Constant for map
MAP_WIDTH = 200 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 100 # meter
GRID_SIZE = 10 # meter
# Tree depth
TREE_DEPTH = 2

# TODO implement reward

class TrajectoryNode:
    def __init__(self, position, reward=0, parent=None):
        # value
        self.position = position
        self.reward = reward
        self.cumulated_reward
        # link
        self.parent = parent
        self.leafs = []

    def __repr__(self):
        return "{}".format(self.position)

class TrajectoryTree:
    def __init__(self, position):
        # Initial grid position of UAV
        self.root = TrajectoryNode(position)
        # Leafs of leaf with depth == TREE_DEPTH
        self.last_leafs = []
        # Make adjacent node tree with TREE_DEPTH
        # Parenthesize self.root for recursive function implementation.
        self.recursive_find_leaf([self.root], 1) 

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

    def move_to_next_node(self):
        # Find the largest leaf
        max_cumulated_reward = -1
        max_index = -1
        for i in range(len(self.last_leafs)):
            if max_cumulated_reward < self.last_leafs[i].cumulated_reward:
                max_cumulated_reward = self.last_leafs[i].cumulated_reward
                max_index = i

        # Find (TREE_DEPTH-2)-th parent, which is leaf of root.
        max_n_parent = self.last_leafs[max_index]
        for _ in range(TREE_DEPTH-2):
            max_n_parent = max_n_parent.parent
        next_root = max_n_parent 

        # Move root to next root
        self.root = next_root

        # Initialize self.last_leafs, since leafs in self.last_leafs are not of depth == TREE_DEPTH.
        # They are of depth == TREE_DEPTH-1
        self.last_leafs = []
        # Find leafs of depth == DEPTH
        self.recursive_find_leaf([self.root], 1)

        return self.root

    def recursive_find_leaf(self, leafs, node_level):
        # Terminate recursive function when it reaches to depth limit
        if node_level == TREE_DEPTH:
            # Save the leafs of DEPTH==TREE_DEPTH
            self.last_leafs += leafs
            return
        node_level += 1
        # Find leafs of leaf
        for leaf in leafs:
            # Check whether leafs of leaf are already found.
            if len(leaf.leafs) == 0:
                leaf.leafs = self.find_leaf(leaf)
# TEST CODE FOR find_leaf
#            print leaf
#            print leaf.leafs
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
#                                        leaf.cumulated_reward = leaf.reward + leaf.parent.cumulated_reward
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
        # otherwise return false.
        return False

if __name__ =="__main__":
    import time
    start = time.time()
    random.seed(a=50)
# TEST CODE FOR DFS
    position = np.array([50,50,70])
    a = TrajectoryTree(position)
    PATH = []
    for i in range(MAX_TIME):
        PATH.append(a.root)
        a.root = a.DFS(a.root)[0][-2]
        a.recursive_find_leaf([a.root], 1) 
    print TREE_DEPTH, time.time()-start
    print PATH

    random.seed(a=50)
    TREE_DEPTH=5
    start = time.time()
    position = np.array([50,50,70])
    a = TrajectoryTree(position)
    PATH = []
    for i in range(MAX_TIME):
        PATH.append(a.root)
        a.root = a.DFS(a.root)[0][-2]
        a.recursive_find_leaf([a.root], 1) 
    print TREE_DEPTH,time.time()-start
    print PATH

    random.seed(a=50)
    TREE_DEPTH=7
    start = time.time()
    position = np.array([50,50,70])
    a = TrajectoryTree(position)
    PATH = []
    for i in range(MAX_TIME):
        PATH.append(a.root)
        a.root = a.DFS(a.root)[0][-2]
        a.recursive_find_leaf([a.root], 1) 
    print TREE_DEPTH,time.time()-start
    print PATH

