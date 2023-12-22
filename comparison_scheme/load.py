import math
import os
import pickle

import numpy as np
import system_model as sm

path = "/home/hslyu/storage/dbspf/bw2/twc-rate/datarate_10/env_0069"
for t in range(0, 20):
    filename = f"UBS_{t}.pkl"
    if os.path.exists(os.path.join(path, filename)):
        with open(os.path.join(path, filename), "rb") as f:
            UBS = pickle.load(f)
            print(UBS.position)
            print([int(user.serviced_data - 10) for user in UBS.list_ue])
