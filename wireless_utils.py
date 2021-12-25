#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import random
import math
import time

FREQUENCY = 2.0*1e9 # Hz
LIGHTSPEED = 3*1e8 # m/s
SURROUNDING_A = 9.64 # Envrionmental parameter for probablistic LOS link
SURROUNDING_B = 0.13 # Envrionmental parameter for probablistic LOS link
LOS_EXCESSIVE = 1 # dB, excessive pathloss of los link
NLOS_EXCESSIVE = 40 # dB, excessive pathloss of nlos link
NOISE_DENSITY = -174

def get_pathloss(distance, altitude):
    """
    Caculate pathloss --> snr --> spectral efficiency
    """
    angle = math.pi/2 - math.acos(altitude/distance)
    los_prob = 1/(1 + SURROUNDING_A * \
            math.exp(-SURROUNDING_B*(180/math.pi*angle - SURROUNDING_A)))
#    print(los_prob)
    pathloss = 20*math.log10(4*math.pi*FREQUENCY*distance/LIGHTSPEED) + los_prob*LOS_EXCESSIVE + \
                (1-los_prob)*NLOS_EXCESSIVE
    return pathloss

def psd2snr(pathloss):
    """ 
    Because unit of psd is 200mW/20MHz, we should convert it to mw/Hz
    """
    psd = 1e-5
    return 10*math.log10(psd) - pathloss - NOISE_DENSITY

def snr2se(snr):
    """
    Because unit of resource is 20MHz,
    we should convert the unit of se from bps/Hz to Mbps/20MHz
    """
    return math.log(1+pow(10,snr/10),2)

if __name__ =="__main__":
#    (get_pathloss(300,100))
#    (get_pathloss(50,50))
    print(get_pathloss(150,50))
    print(get_pathloss(50,50))
    print(psd2snr(get_pathloss(150,50)))
    print(psd2snr(get_pathloss(50,50)))
    print(snr2se(psd2snr(get_pathloss(150,50))))
    print(snr2se(psd2snr(get_pathloss(50,50))))
