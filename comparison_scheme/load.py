import pickle
import system_model as sm
import math
import numpy as np
def initialization_validity_check(cvx_mode = False):
#        log = lambda x: cp.log(x) / cp.log(10) if cvx_mode else math.log10
    log = math.log10

    print("----UAV status----")
    print(f'{UBS.position = }')
    print(f'{UBS.prev_position = }')
    print(f'{UBS.gbs_los_prob = }')
    for subcarrier in UBS.list_subcarrier:
        print(f'\t\t{subcarrier.frequency = } (GHz)')
        print(f'\t\t{subcarrier.power = } (dB)')
        print(f'\t\t{subcarrier.pathloss = } (dB)')
        print(f'\t\t{subcarrier.channel = } (dB)')
        print('\t\t----------')

    print("----User status----")
    for user in list_ue: 
        print(f'\n\t{user.position = }')
        print(f'\t{user.mode = }')
        print(f'\t{user.los_prob = }')
        print('\t---------- subcarrier_UBS')

        for subcarrier in user.list_subcarrier_UBS:
            print(f'\t\t{subcarrier.alpha = } ')
            print(f'\t\t{subcarrier.frequency = } (GHz)')
            print(f'\t\t{subcarrier.power = } (dB)')
            print(f'\t\t{subcarrier.pathloss = } (dB)')
            print(f'\t\t{subcarrier.channel = } (dB)')
            print('\t\t----------')

#            print('\t---------- subcarrier_GBS')
#            for subcarrier in user.list_subcarrier_GBS:
#                print(f'\t\t{subcarrier.alpha = } ')
#                print(f'\t\t{subcarrier.frequency = } (GHz)')
#                print(f'\t\t{subcarrier.power = } (dB)')
#                print(f'\t\t{subcarrier.pathloss = } (dB)')
#                print(f'\t\t{subcarrier.channel = } (dB)')
#                print('\t\t----------')

with open("./result/tw20_user10/env_0050/UBS_18.pkl", 'rb') as f:
    UBS = pickle.load(f)
    print(UBS.position)
    for user in UBS.list_ue:
        print(user.serviced_data)
