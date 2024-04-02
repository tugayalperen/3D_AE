import numpy as np
import matplotlib.pyplot as plt

for et in ["03_193_0", "03_142_1", "03_318_2"]:
    experiment_date = "02_04_2024"
    experiment_time = et

    filename_base = "_" + experiment_date + "|" + experiment_time + "_.npy"
    filename_dir = "./" + experiment_date + "|" + experiment_time + "/"
    pos_xs = np.load(filename_dir + "log_pos_xs" + filename_base, allow_pickle=True).T
    pos_ys = np.load(filename_dir + "log_pos_ys" + filename_base, allow_pickle=True).T
    pos_zs = np.load(filename_dir + "log_pos_zs" + filename_base, allow_pickle=True).T
    exp_length = pos_xs.shape[0]
    num_agents = pos_xs.shape[1]

    rel_pos_xs = np.zeros((exp_length, num_agents))
    rel_pos_ys = np.zeros((exp_length, num_agents))
    rel_pos_zs = np.zeros((exp_length, num_agents))
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
        rel_pos_xs[i, :] = _pos_xs - com_xs[i]
        rel_pos_ys[i, :] = _pos_ys - com_ys[i]
        rel_pos_zs[i, :] = _pos_zs - com_zs[i]

    # Plot setup
    fig, axs = plt.subplots(1, 3, figsize=(20, 6))  # Slightly wider figure to accommodate the colorbar

    # Common settings
    plot_limits = [[-1.5, 1.5], [-1.5, 1.5]]
    bins = [50, 50]  # Using 100x100 grid as you initially intended
    cmap = 'YlGnBu'

    # Plot 1: rel_pos_x vs. rel_pos_y
    im = axs[0].hist2d(rel_pos_xs.flatten(), rel_pos_ys.flatten(), bins=bins, range=plot_limits, cmap=cmap)
    axs[0].set_xlabel('Relative Position X')
    axs[0].set_ylabel('Relative Position Y')
    axs[0].set_title('X-Y Plane')
    axs[0].set_aspect('equal')  # Ensuring equal aspect ratio

    # Plot 2: rel_pos_y vs. rel_pos_z
    axs[1].hist2d(rel_pos_ys.flatten(), rel_pos_zs.flatten(), bins=bins, range=plot_limits, cmap=cmap)
    axs[1].set_xlabel('Relative Position Y')
    axs[1].set_ylabel('Relative Position Z')
    axs[1].set_title('Y-Z Plane')
    axs[1].set_aspect('equal')

    # Plot 3: rel_pos_x vs. rel_pos_z
    axs[2].hist2d(rel_pos_xs.flatten(), rel_pos_zs.flatten(), bins=bins, range=plot_limits, cmap=cmap)
    axs[2].set_xlabel('Relative Position X')
    axs[2].set_ylabel('Relative Position Z')
    axs[2].set_title('X-Z Plane')
    axs[2].set_aspect('equal')

    # Adjust layout to not overlap
    plt.tight_layout(pad=3.0)

    # Create an axis for the colorbar
    cbar_ax = fig.add_axes([0.93, 0.15, 0.02, 0.7])  # [left, bottom, width, height]
    fig.colorbar(im[3], cax=cbar_ax, label='Number of Agents in Cell')

    plt.suptitle('Distribution Heatmap Around CoM')  # Adding a title to the figure
    plt.savefig(filename_dir + "dist.png")

    # plt.show()
