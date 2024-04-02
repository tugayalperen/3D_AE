import numpy as np
import matplotlib.pyplot as plt

experiment_date = "01_04_2024"
experiment_time = "21_981_0"

filename_base = "_" + experiment_date + "|" + experiment_time + ".npy"
filename_dir = "./" + experiment_date + "|" + experiment_time + "/"
pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
exp_length = pos_xs.shape[0]
num_agents = pos_xs.shape[1]

min_dij_xs = np.zeros(exp_length)
min_dij_ys = np.zeros(exp_length)
min_dij_zs = np.zeros(exp_length)
min_dij = np.zeros(exp_length)
com_speed = np.zeros(exp_length-1)
dt = 0.05

for i in range(exp_length):
    _pos_xs = pos_xs[i, :]
    _pos_ys = pos_ys[i, :]
    _pos_zs = pos_zs[i, :]
    d_ij_xs = np.abs(_pos_xs[:, None] - _pos_xs)
    d_ij_xs[d_ij_xs == 0] = np.inf
    min_dij_xs[i] = np.min(d_ij_xs)
    d_ij_ys = np.abs(_pos_ys[:, None] - _pos_ys)
    d_ij_ys[d_ij_ys == 0] = np.inf
    min_dij_ys[i] = np.min(d_ij_ys)
    d_ij_zs = np.abs(_pos_zs[:, None] - _pos_zs)
    d_ij_zs[d_ij_zs == 0] = np.inf
    min_dij_zs[i] = np.min(d_ij_zs)
    d_ij = np.sqrt(np.square(d_ij_xs) + np.square(d_ij_ys) + np.square(d_ij_zs))
    d_ij[d_ij == 0] = np.inf
    min_dij[i] = np.min(d_ij)


fig = plt.figure()
ax = fig.add_subplot(411)
ax.plot(min_dij)
ax = fig.add_subplot(412)
ax.plot(min_dij_xs)
ax = fig.add_subplot(413)
ax.plot(min_dij_ys)
ax = fig.add_subplot(414)
ax.plot(min_dij_zs)
plt.show()
