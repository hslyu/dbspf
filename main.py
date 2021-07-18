#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import time
from drone_basestation import *
import argparse

# Constant for UAV
VEHICLE_VELOCITY = 10. # m/s
TIME_STEP = 1 # s
MAX_TIME = 40
## Constant for map
MAP_WIDTH = 200 # meter, Both X and Y axis width
MIN_ALTITUDE = 50 # meter
MAX_ALTITUDE = 100 # meter
GRID_SIZE = 10 # meter
# Constant for user
NUM_UE = 20
TIME_WINDOW_SIZE = [5,5]
DATARATE_WINDOW = [35, 60] # Requiring datarate Mb/s
INITIAL_DATA = 10 # Mb
# Tree depth
TREE_DEPTH = 1

def get_parser():
    parser = argparse.ArgumentParser(description="Extract bounding boxes using Detectron2",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--video_input", help="Path to video file.")
    parser.add_argument("--videos_input-dir", help="A directory of input videos. It also extracts ROI pooled features from the Faster R-CNN object detector.")
    parser.add_argument("--images_input-dir", type=str, help="A directory of input images with extension *.jpg. The file names should be the frame number (e.g. 00000000001.jpg)")
    parser.add_argument("--model_weights", type=str, default="detectron2://COCO-Detection/faster_rcnn_R_101_FPN_3x/137851257/model_final_f6e8b1.pkl", help="Detectron2 object detection model.")
    return parser

if __name__ =="__main__":
    position = np.array([random.randint(0, MAP_WIDTH)//10*10,
                         random.randint(0, MAP_WIDTH)//10*10,
                         random.randint(MIN_ALTITUDE, MAX_ALTITUDE)//10*10])
    # Initial grid position of UAV
    root = TrajectoryNode(position)
    # Make user list
    user_list = []
    for i in range(NUM_UE):
        tw_size = random.randint(TIME_WINDOW_SIZE[0], TIME_WINDOW_SIZE[1])
        user = User(i, # id
                random.randint(0, MAP_WIDTH), random.randint(0, MAP_WIDTH), # map
                random.randint(0, MAX_TIME-tw_size), tw_size, # time window
                random.randint(DATARATE_WINDOW[0], DATARATE_WINDOW[1]), # data
                INITIAL_DATA, INF) # data
        user_list.append(user)
    root.user_list = user_list

    tree = TrajectoryTree(root, VEHICLE_VELOCITY,\
                            TIME_STEP, GRID_SIZE,\
                            MAP_WIDTH, MIN_ALTITUDE, MAX_ALTITUDE,\
                            TREE_DEPTH, MAX_TIME)

    PATH = tree.pathfinder()
    reward = 0
    for leaf in PATH:
       reward += leaf.reward
    print(PATH)
    print(reward)
