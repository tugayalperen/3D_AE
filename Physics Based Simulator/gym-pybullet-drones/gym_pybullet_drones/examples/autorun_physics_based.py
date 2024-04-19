import numpy as np
import scipy.io as sio
import time
import copy
from multiprocessing import Pool
import os

def run_process(input_dict):
    print("INDEX: ", input_dict["exp_index"])
    cmd = os.system('conda run -n ants24_cf_sim python ./3d_flocking_vectorized.py --input_dict "%s"' % str(input_dict))
    os.system(cmd)


if __name__ == '__main__':
    p = Pool(1)
    input_dics = []

    base_input_dict = {"if_plot": 1, "seed_rndm": 1, "n_agent": 5, "spacing": 0.75, "mean_noise": 0.25,
                           "init_x": 1, "init_y": 1, "init_z": 2, "sigma_const": 0.5, "umax_const": 0.0,
                           "wmax": np.pi, "alpha": 1.0, "beta": 1.0, "k1": 0.5, "k2": 0.1, "run_wall_time": 300,
                           "h_alignment": 1, "boun_x": 1000, "boun_y": 1000, "boun_z": 1000, "noise_pos": 0.05,
                           "noise_h": np.pi/72, "exp_index": 0, "param_level": 0, "exp_name": "snapshot"}

    exp_index = 0
    base_input_dict["seed_rndm"] = int(np.random.rand() * 10000)
    # print("Initial seed:", base_input_dict["seed_rndm"])
    param_flag = 0
    # input_dics.append(base_input_dict)

    # for a_b in [0.01, 0.1, 1.0]:
    #     if not param_flag == 0:
    #         base_input_dict["param_level"] += 1
    #     for repeat in range(50):
    #         base_input_dict["seed_rndm"] += 1
    #         _temp_input_dict = copy.deepcopy(base_input_dict)
    #         _temp_input_dict["alpha"] = 1.0 * a_b
    #         _temp_input_dict["exp_index"] = exp_index
    #
    #         input_dics.append(_temp_input_dict)
    #         exp_index += 1
    #
    #     if param_flag == 0:
    #         param_flag += 1
    #
    # for k1 in [0.05, 0.5, 5.0]:
    #     if not param_flag == 0:
    #         base_input_dict["param_level"] += 1
    #     for repeat in range(50):
    #         base_input_dict["seed_rndm"] += 1
    #         _temp_input_dict = copy.deepcopy(base_input_dict)
    #         _temp_input_dict["k1"] = k1
    #         _temp_input_dict["exp_index"] = exp_index
    #         input_dics.append(_temp_input_dict)
    #         exp_index += 1
    #
    #     if param_flag == 0:
    #         param_flag += 1

    # for k2 in [0.01, 0.1, 1.0]:
    #     if not param_flag == 0:
    #         base_input_dict["param_level"] += 1
    #     for repeat in range(50):
    #         base_input_dict["seed_rndm"] += 1
    #         _temp_input_dict = copy.deepcopy(base_input_dict)
    #         _temp_input_dict["k2"] = k2
    #         _temp_input_dict["exp_index"] = exp_index
    #         # print("Experiment seed:", _temp_input_dict["seed_rndm"])
    #         input_dics.append(_temp_input_dict)
    #         exp_index += 1
    #
    #     if param_flag == 0:
    #         param_flag += 1

    # # NO HEADING
    # for repeat in range(20):
    #     base_input_dict["seed_rndm"] += 1
    #     _temp_input_dict = copy.deepcopy(base_input_dict)
    #     _temp_input_dict["exp_index"] = exp_index
    #     print("Exp index: ", exp_index)
    #     input_dics.append(_temp_input_dict)
    #     exp_index += 1


    for i in range(1):
        input_dics.append(base_input_dict)

    p.map(run_process, input_dics)

