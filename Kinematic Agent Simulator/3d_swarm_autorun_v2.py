import numpy as np
from multiprocessing import Pool
import os

def run_process(input_dict):
    cmd = 'python 3d_swarm_v2_cupy.py --parameters "%s"' % str(input_dict)
    os.system(cmd)


if __name__ == '__main__':
    p = Pool(1)
    input_dics = []

    base_input_dict = {"if_plot": 1, "seed_rndm": 1, "n_agent": 5, "spacing": 0.75, "mean_noise": 0.25,
                           "init_x": 500, "init_y": 500, "init_z": 500, "sigma_const": 0.5, "umax_const": 0.3,
                           "wmax": np.pi, "alpha": 1.0, "beta": 1.0, "k1": 0.5, "k2": 0.1, "run_wall_time": 600,
                           "h_alignment": 1, "boun_x": 1000, "boun_y": 1000, "boun_z": 1000, "noise_pos": 0.05,
                           "noise_h": np.pi/72, "exp_index": 0, "param_level": 0, "exp_name": "5_agents"}

    exp_index = 0
    base_input_dict["seed_rndm"] = int(np.random.rand() * 10000)
    # print("Initial seed:", base_input_dict["seed_rndm"])
    param_flag = 0
    input_dics.append(base_input_dict)

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

    # NO HEADING
    # for repeat in range(50):
    #     base_input_dict["seed_rndm"] += 1
    #     _temp_input_dict = copy.deepcopy(base_input_dict)
    #     _temp_input_dict["exp_index"] = exp_index
    #     # print("Experiment seed:", _temp_input_dict["seed_rndm"])
    #     input_dics.append(_temp_input_dict)
    #     exp_index += 1


    # for i in range(len(input_dics)):
    #     input_dics.append(base_input_dict)

    p.map(run_process, input_dics)

