import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

for et in ["03_193_0", "03_142_1", "03_318_2"]:
    experiment_date = "02_04_2024"
    experiment_time = et

    # experiment_date = "31_03_2024"
    # experiment_time = "15_01_08_092"

    filename_base = "_" + experiment_date + "|" + experiment_time + "_.npy"
    filename_dir = "./" + experiment_date + "|" + experiment_time + "/"
    pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
    pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
    pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
    exp_length = pos_xs.shape[0]
    num_agents = pos_xs.shape[1]

    com_xs = np.zeros(exp_length)
    com_ys = np.zeros(exp_length)
    com_zs = np.zeros(exp_length)
    com_speed = np.zeros(exp_length-1)
    dt = 0.05

    for i in range(exp_length):
            com_xs[i] = np.mean(pos_xs[i, :])
            com_ys[i] = np.mean(pos_ys[i, :])
            com_zs[i] = np.mean(pos_zs[i, :])

            if i > 0:
                com_speed[i-1] = np.sqrt(np.square(com_xs[i] - com_xs[i-1]) + np.square(com_ys[i] - com_ys[i-1]) + np.square(com_zs[i] - com_zs[i-1])) / dt



    fig = plt.figure()
    ax = fig.add_subplot(211)
    ax.plot(com_speed)
    mean_cs = np.mean(com_speed, axis=0)
    mean_cs = gaussian_filter(com_speed, sigma=5)
    ax2 = fig.add_subplot(212)
    ax2.plot(mean_cs)
    ax.set_title('CoM Speed')
    ax.set_ylim(0, 1.0)
    ax2.set_ylim(0, 1.0)
    plt.savefig(filename_dir + "com_speed.png")
    # plt.show()
