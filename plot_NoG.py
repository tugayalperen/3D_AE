import numpy as np
import matplotlib.pyplot as plt


def calc_NoG(x, y, member_number, Da):
    xx1, xx2 = np.meshgrid(x, x)
    yy1, yy2 = np.meshgrid(y, y)
    d_ij_x = xx1-xx2
    d_ij_y = yy1-yy2
    d_ij = np.sqrt(d_ij_x**2 + d_ij_y**2)
    d_ij = np.triu(d_ij)
    mm = 0
    list1 = []
    list2 = []
    for j in range(0, member_number):
        for k in range(0, member_number):
            if d_ij[j, k] < Da and j != k and d_ij[j, k] != 0:
                mm = mm + 1
                list1.append(j)
                list2.append(k)
    m = mm
    n = member_number
    nf = np.zeros(n)
    for k in range(0, n):
        nf[k] = k
    for l in range(0, m):
        j = list1[l]
        while nf[j] != j:
            j = int(nf[j])
        k = list2[l]
        while nf[k] != k:
            k = int(nf[k])
        if j != k:
            nf[j] = k
    for j in range(0,n):
        while nf[j] != nf[int(nf[j])]:
            nf[j] = nf[int(nf[j])]

    number_of_groups = np.size(np.unique(nf))
    return number_of_groups


experiment_date = "31_03_2024"
experiment_time = "16_57_52_968"

filename_base = "_" + experiment_date + "|" + experiment_time + ".npy"
filename_dir = "./" + experiment_date + "|" + experiment_time + "/"
pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
exp_length = pos_xs.shape[0]
num_agents = pos_xs.shape[1]
sensing_range = 3.0

nogs = np.zeros(exp_length)

for i in range(exp_length):
        nogs[i] = calc_NoG(pos_xs[i, :], pos_ys[i, :], num_agents, sensing_range)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(nogs)
plt.show()