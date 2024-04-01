import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plot_swarm_replay

experiment_date = "01_04_2024"
experiment_time = "42_570"

filename_base = "_" + experiment_date + "|" + experiment_time + ".npy"
filename_dir = "./point_results/" + experiment_date + "|" + experiment_time + "/"
pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
pos_hxc = np.load(filename_dir + "log_pos_hxc" + filename_base, allow_pickle=True).T
pos_hyc = np.load(filename_dir + "log_pos_hyc" + filename_base, allow_pickle=True).T
pos_hzc = np.load(filename_dir + "log_pos_hzc" + filename_base, allow_pickle=True).T
exp_length = pos_xs.shape[0]
num_agents = pos_xs.shape[1]

plotter = plot_swarm_replay.SwarmPlotter(num_agents, 600, 600, 600)

# Plot the trajectory
for i in range(exp_length):
    if (not (i % 5)) and (i > 10):
        print(i)
        plotter.update_plot(pos_xs[i, :], pos_ys[i, :], pos_zs[i, :],
                            np.arccos(pos_hxc[i, :]), np.arccos(pos_hyc[i, :]), np.arccos(pos_hzc[i, :]))


