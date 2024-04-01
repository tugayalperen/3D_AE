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
    p = Pool(1)
    input_dics = []

    base_input_dict = {"if_plot": 0, "seed_rndm": 1234, "n_agent": 10, "spacing": 0.8, "mean_noise": 0.25,
                           "init_x": 4.0, "init_y": 2.0, "init_z": 1.0, "sigma_const": 0.5, "umax_const": 0.1,
                           "wmax": 1.57, "alpha": 1.0, "beta": 2.0, "k1": 0.6, "k2":0.1, "run_wall_time": 100,
                           "h_alignment": 1, "boun_x": 200, "boun_y": 200, "boun_z": 200, "noise_pos":0.1,
                           "noise_h":np.pi/36, "exp_index":0}

    for k1 in np.linspace(0.1, 1.0, 1):
        _temp_input_dict = copy.deepcopy(base_input_dict)
        _temp_input_dict["k1"] = k1
        input_dics.append(_temp_input_dict)

    for i in range(1, 1):
        input_dics.append(base_input_dict)

    p.map(run_process, input_dics)

