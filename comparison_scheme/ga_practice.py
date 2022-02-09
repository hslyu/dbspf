import system_model as sm
import math
import numpy as np
from geneticalgorithm import geneticalgorithm as ga

def dB2Watt(var_dB: float=0):
    return 10**(var_dB/10)

def f(X):
    pen=0
    if X[0]+X[1]<2:
        pen=500+1000*(2-X[0]-X[1])
    return np.sum(X)+pen
    
varbound=np.array([[0,10]]*3)

#model=ga(function=f,dimension=3,variable_type='real',variable_boundaries=varbound)

#model.run()

UAV, list_ue = sm.initialize_UAV()
sm.initialize_users
