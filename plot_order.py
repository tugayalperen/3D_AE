import numpy as np
import matplotlib.pyplot as plt


for et in ["38_017_0", "38_043_1", "37_919_2"]:
    experiment_date = "02_04_2024"
    experiment_time = et

    filename_base = "_" + experiment_date + "|" + experiment_time + "_.npy"
    filename_dir = "./" + experiment_date + "|" + experiment_time + "/"
    pos_hxc = np.load(filename_dir + "log_pos_hxc" + filename_base, allow_pickle=True).T
    pos_hyc = np.load(filename_dir + "log_pos_hyc" + filename_base, allow_pickle=True).T
    pos_hzc = np.load(filename_dir + "log_pos_hzc" + filename_base, allow_pickle=True).T
    exp_length = pos_hxc.shape[0]
    num_agents = pos_hxc.shape[1]

    orders = np.zeros(exp_length)
    step_xs_sum = np.zeros(exp_length)
    step_ys_sum = np.zeros(exp_length)
    step_zs_sum = np.zeros(exp_length)

    for i in range(exp_length):
        for j in range(num_agents):
            step_xs_sum[i] += pos_hxc[i, j]
            step_ys_sum[i] += pos_hyc[i, j]
            step_zs_sum[i] += pos_hzc[i, j]

        orders[i] = np.sqrt(np.power(step_xs_sum[i], 2) + np.power(step_ys_sum[i], 2) + np.power(step_zs_sum[i], 2)) / num_agents

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(orders)
    ax.set_title('Order')
    ax.set_ylim(0, 1.1)
    plt.savefig(filename_dir + "order.png")
    # plt.show()

