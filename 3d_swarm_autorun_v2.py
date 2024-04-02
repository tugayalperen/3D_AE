import numpy as np
import scipy.io as sio
import time
import copy
from multiprocessing import Pool
import os

def run_process(input_dict):
    cmd = 'python 3d_swarm_v2_cupy.py --parameters "%s"' % str(input_dict)
    os.system(cmd)


if __name__ == '__main__':
    p = Pool(4)
    input_dics = []

    base_input_dict = {"if_plot": 0, "seed_rndm": 1234, "n_agent": 20, "spacing": 0.8, "mean_noise": 0.25,
                           "init_x": 500, "init_y": 500, "init_z": 500, "sigma_const": 0.5, "umax_const": 0.3,
                           "wmax": np.pi, "alpha": 1.0, "beta": 2.0, "k1": 0.5, "k2": 0.1, "run_wall_time": 600,
                           "h_alignment": 1, "boun_x": 100, "boun_y": 100, "boun_z": 100, "noise_pos": 0.05,
                           "noise_h": np.pi/72, "exp_index": 0}

    exp_index = 0
    for k1 in [0.01, 0.1, 1.0]:
        _temp_input_dict = copy.deepcopy(base_input_dict)
        _temp_input_dict["k2"] = k1
        _temp_input_dict["exp_index"] = exp_index
        exp_index += 1
        input_dics.append(_temp_input_dict)

    # for i in range(len(input_dics)):
    #     input_dics.append(base_input_dict)

    p.map(run_process, input_dics)

