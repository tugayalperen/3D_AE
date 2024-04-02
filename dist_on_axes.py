import numpy as np
import matplotlib.pyplot as plt

experiment_date = "01_04_2024"
experiment_time = "12_22_56_468"

filename_base = "_" + experiment_date + "|" + experiment_time + ".npy"
filename_dir = "./" + experiment_date + "|" + experiment_time + "/"
pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
exp_length = pos_xs.shape[0]
num_agents = pos_xs.shape[1]

x_variance = np.zeros(exp_length)
x_mean = np.zeros(exp_length)
y_variance = np.zeros(exp_length)
y_mean = np.zeros(exp_length)
z_variance = np.zeros(exp_length)
z_mean = np.zeros(exp_length)
com_xs = np.zeros(exp_length)
com_ys = np.zeros(exp_length)
com_zs = np.zeros(exp_length)

dt = 0.05

for i in range(exp_length):
    _pos_xs = pos_xs[i, :]
    _pos_ys = pos_ys[i, :]
    _pos_zs = pos_zs[i, :]
    com_xs[i] = np.mean(_pos_xs)
    com_ys[i] = np.mean(_pos_ys)
    com_zs[i] = np.mean(_pos_zs)
    _pos_xs = np.abs(_pos_xs - com_xs[i])
    _pos_ys = np.abs(_pos_ys - com_ys[i])
    _pos_zs = np.abs(_pos_zs - com_zs[i])
    x_variance[i] = np.var(_pos_xs)
    x_mean[i] = np.mean(_pos_xs)
    y_variance[i] = np.var(_pos_ys)
    y_mean[i] = np.mean(_pos_ys)
    z_variance[i] = np.var(_pos_zs)
    z_mean[i] = np.mean(_pos_zs)



fig = plt.figure()
ax = fig.add_subplot(411)
ax.plot(x_mean)
ax = fig.add_subplot(412)
ax.plot(y_mean)
ax = fig.add_subplot(413)
ax.plot(z_mean)
# ax = fig.add_subplot(414)
# ax.plot(z_mean)

fig = plt.figure()
ax = fig.add_subplot(411)
ax.plot(x_variance)
ax = fig.add_subplot(412)
ax.plot(y_variance)
ax = fig.add_subplot(413)
ax.plot(z_variance)
# ax = fig.add_subplot(414)
# ax.plot(z_mean)
plt.show()
