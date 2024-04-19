"""CrazyFlie software-in-the-loop control example.

Setup
-----
Step 1: Clone pycffirmware from https://github.com/utiasDSL/pycffirmware
Step 2: Follow the install instructions for pycffirmware in its README

Example
-------
In terminal, run:
python gym_pybullet_drones/examples/cf.py

"""

###
# Logging is added!
###

import time
import argparse
import numpy as np
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync, str2bool
from flocking_utils import FlockingUtilsVec
import pybullet as p
from datetime import datetime
import ast
import os

global _id
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_PLOT = False
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_OUTPUT_FOLDER = 'results'
DURATION_SEC = 600

def create_empty_directory(directory_path):
    try:
        os.makedirs(directory_path)
        print(f"Directory '{directory_path}' created successfully.")
    except FileExistsError:
        print(f"Directory '{directory_path}' already exists.")


def run(
        input_dict,
        num_drones,
        save_dir,
        drone=DEFAULT_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        duration_sec=DURATION_SEC,
):
    env = CtrlAviary(drone_model=ARGS.drone,
                     num_drones=ARGS.num_drones,
                     initial_xyzs=INIT_XYZ,
                     initial_rpys=INIT_RPY,
                     physics=ARGS.physics,
                     # physics=Physics.PYB_DW,
                     neighbourhood_radius=10,
                     pyb_freq=ARGS.simulation_freq_hz,
                     ctrl_freq=ARGS.control_freq_hz,
                     gui=ARGS.gui,
                     user_debug_gui=ARGS.user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    ctrl = [DSLPIDControl(drone_model=ARGS.drone) for i in range(ARGS.num_drones)]

    if self_log:
        log_pos_xs = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_ys = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_zs = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_hxc = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_hyc = np.zeros([NUM_DRONES, DURATION_SEC * DEFAULT_CONTROL_FREQ_HZ])
        log_pos_hzc = np.zeros([NUM_DRONES, DURATION_SEC * DEFAULT_CONTROL_FREQ_HZ])

    START = time.time()
    action = np.zeros((NUM_DRONES, 4))

    pos_x = np.zeros(NUM_DRONES)
    pos_y = np.zeros(NUM_DRONES)
    pos_z = np.zeros(NUM_DRONES)
    log_counter = 0

    for i in range(0, int(ARGS.duration_sec * env.CTRL_FREQ)):
        obs, reward, done, info, _ = env.step(action)
        positions = env.pos.T
        print(i)
        p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-30, cameraPitch=-40,
                                     cameraTargetPosition=[positions[0, 20], positions[1, 20], positions[2,20] - 0.5])


        if i%2 ==0:
            f_util.calc_dij(positions)
            f_util.calc_p_forces()
            f_util.calc_alignment_forces()
            f_util.calc_boun_rep(positions)
            u = f_util.calc_u_w()
            headings = f_util.get_heading()
            f_util.update_heading()

        # if i % (env.CTRL_FREQ*1) == 0:
        #     f_util.plot_swarm(positions[0], positions[1], positions[2], np.arccos(headings[0]), np.arccos(headings[1]), np.arccos(headings[2]))

        vel_cmd = u*headings
        pos_cmd = positions
        for j in range(NUM_DRONES):
            # pos_cmd = vel_cmd * (1 / env.CTRL_FREQ) + np.array([pos_x[j], pos_y[j], pos_z[j]])
            action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                              state=obs[j],
                                                              target_pos=pos_cmd[:,j],
                                                              target_vel=vel_cmd[:,j],
                                                              target_rpy=np.array([0, 0, 0])
                                                              )

        #### Log the simulation ####################################
        if self_log and i%2 ==0:
            log_pos_xs[:, log_counter] = positions[0]
            log_pos_ys[:, log_counter] = positions[1]
            log_pos_zs[:, log_counter] = positions[2]
            log_pos_hxc[:, log_counter] = headings[0]
            log_pos_hyc[:, log_counter] = headings[1]
            log_pos_hzc[:, log_counter] = headings[2]
            log_counter += 1

        if i%100 ==0:
            print("Progress: ", i/int(ARGS.duration_sec * env.CTRL_FREQ))

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            pass
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    if self_log:
        _save_dir = ARGS.save_dir
        filename_posx = save_dir + "log_pos_xs_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_posy = save_dir + "log_pos_ys_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_posz = save_dir + "log_pos_zs_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_pos_hxc = save_dir + "log_pos_hxc_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_pos_hyc = save_dir + "log_pos_hyc_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_pos_hzc = save_dir + "log_pos_hzc_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        np.save(filename_posx, log_pos_xs)
        np.save(filename_posy, log_pos_ys)
        np.save(filename_posz, log_pos_zs)
        np.save(filename_pos_hxc, log_pos_hxc)
        np.save(filename_pos_hyc, log_pos_hyc)
        np.save(filename_pos_hzc, log_pos_hzc)


if __name__ == "__main__":
    ### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--input_dict', required=True)
    ARGS = parser.parse_args()
    input_dict = ast.literal_eval(ARGS.input_dict)
    NUM_DRONES = input_dict["n_agent"]
    spacing = input_dict["spacing"]
    mean_noise = input_dict["mean_noise"]
    init_center_x = input_dict["init_x"]
    init_center_y = input_dict["init_y"]
    init_center_z = input_dict["init_z"]
    exp_index = input_dict["exp_index"]
    param_level = input_dict["param_level"]
    self_log = True
    exp_name = input_dict["exp_name"]
    save_dir = "./self_logs/" + exp_name + "/"
    dt_string = datetime.utcnow().strftime('%d_%m_%Y|')
    dir_path = dt_string + "_" + str(param_level) + "_" + str(exp_index)
    create_empty_directory(save_dir + dir_path)
    save_dir = save_dir + dir_path + "/"

    f_util = FlockingUtilsVec(NUM_DRONES, init_center_x, init_center_y, init_center_z, spacing, input_dict)
    positions, headings = f_util.initialize_positions()

    INIT_XYZ = positions.T
    INIT_RPY = np.array([[.0, .0, .0] for _ in range(NUM_DRONES)])

    parser.add_argument('--drone', default=DEFAULT_DRONES, type=DroneModel, help='Drone model (default: BETA)',
                        metavar='', choices=DroneModel)
    parser.add_argument('--num_drones', default=NUM_DRONES, type=int, help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics', default=DEFAULT_PHYSICS, type=Physics, help='Physics updates (default: PYB)',
                        metavar='', choices=Physics)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, help='Whether to use PyBullet GUI (default: True)',
                        metavar='')
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool,
                        help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool,
                        help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int,
                        help='Simulation frequency in Hz (default: 500)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int,
                        help='Control frequency in Hz (default: 25)', metavar='')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str,
                        help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--duration_sec',default=DURATION_SEC, type=int,
                        help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--save_dir', default=save_dir, type=str, help='', metavar='')
    ARGS = parser.parse_args()


    run(**vars(ARGS))