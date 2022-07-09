import pickle
import system_model as sm
import math
import numpy as np
import os

path = "/home/hslyu/storage/result_twc21_07_07/tw20_user20/datarate_10/env_0001/"
path = "./test_result"
for t in range(0,20):
    filename = f"UBS_{t}.pkl"
    if os.path.exists(os.path.join(path,filename)):
        with open(os.path.join(path, filename), 'rb') as f:
            UBS = pickle.load(f)
            print(UBS.position)
            print([int(user.serviced_data-10) for user in UBS.list_ue ])

