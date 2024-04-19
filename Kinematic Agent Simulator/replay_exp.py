import numpy as np
import plot_swarm_replay

el = 0
en = 0
exp_repeat = 0
en_name = en + exp_repeat * el
experiment_date = "18_04_2024"
experiment_time = str(el) + "_" + str(en_name)
filename_base = "_" + experiment_date + "|_" + str(en_name) + "_.npy"
filename_dir = ("/Users/tugaykaraguzel/Desktop/3D_AE/point_results/5_agents/"
                + experiment_date + "|_" + experiment_time + "/")
pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
pos_hxc = np.load(filename_dir + "log_pos_hxc" + filename_base, allow_pickle=True).T
pos_hyc = np.load(filename_dir + "log_pos_hyc" + filename_base, allow_pickle=True).T
pos_hzc = np.load(filename_dir + "log_pos_hzc" + filename_base, allow_pickle=True).T
log_us = np.load(filename_dir + "log_us" + filename_base, allow_pickle=True).T
exp_length = pos_xs.shape[0]
num_agents = pos_xs.shape[1]

plotter = plot_swarm_replay.SwarmPlotter(num_agents, 10, 10, 10)

for i in range(exp_length):
    if (not (i % 10)) and (i > 10):
        plotter.update_plot(pos_xs[i, :], pos_ys[i, :], pos_zs[i, :],
                            np.arccos(pos_hxc[i, :]), np.arccos(pos_hyc[i, :]), np.arccos(pos_hzc[i, :]))


