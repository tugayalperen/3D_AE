# if ars["g[u"]:
#     import cupy as np
# else
#     import numpy as np
import time
import numpy as np
import plot_swarm_v2
from datetime import datetime
import os
import argparse
import ast

def create_empty_directory(directory_path):
    try:
        os.makedirs(directory_path)
        print(f"Directory '{directory_path}' created successfully.")
    except FileExistsError:
        print(f"Directory '{directory_path}' already exists.")


def calculate_rotated_vector_batch(V1, V2, wdt):
    e3 = np.cross(V1.T, V2.T)

    # Rodrigues' rotation formula for batch
    e2 = np.cross(e3, V1.T).T
    e2_norm = np.linalg.norm(e2, axis=0)
    e2_norm[e2_norm<=0] = 1
    e2 /= e2_norm

    e1 = V1/np.linalg.norm(V1, axis=0)
    v_rot = V1 * np.cos(wdt) + e2 * np.sin(wdt) * np.linalg.norm(V1, axis=0)
    return v_rot

def calculate_av_heading(heading_vecs):
    # Normalize each vector and sum them to get an average direction
    heading_vecs_normalised = heading_vecs / np.linalg.norm(heading_vecs, axis=0)

    # Calculate the average vector (sum of normalized vectors)
    sum_of_heading_vecs_normalised = np.sum(heading_vecs_normalised, axis=1)

    # Normalize the sum to get the unit vector with the average direction
    unit_vector_average_direction = sum_of_heading_vecs_normalised / np.linalg.norm(sum_of_heading_vecs_normalised)
    return unit_vector_average_direction.reshape(3, 1)


def place_agents(num_agents, init_pos, spacing, mean_noise):
    """
    Place agents in a 3D grid around an initialization point with specified spacing and noise.

    Parameters:
        num_agents (int): Number of agents to place.
        init_pos (tuple): Initialization point (x, y, z).
        spacing (float): Spacing between agents.
        mean_noise (float): Mean value of noise to apply to positions.

    Returns:
        np.array: 3D positions of agents.
    """
    # Approximate cube root to start searching for dimensions
    cube_root = round(num_agents ** (1 / 3))

    # Find dimensions that fill a space as equally as possible, even if some agents are left out
    best_diff = float('inf')
    for x in range(cube_root, 0, -1):
        for y in range(x, 0, -1):
            z = int(np.ceil(num_agents / (x * y)))
            total_agents = x * y * z
            diff = max(abs(x - y), abs(y - z), abs(x - z))
            if diff < best_diff and total_agents >= num_agents:
                best_diff = diff
                dimensions = (x, y, z)

    # Generate grid positions
    grid_positions = np.mgrid[0:dimensions[0], 0:dimensions[1], 0:dimensions[2]].reshape(3, -1).T
    grid_positions = grid_positions * spacing

    # Center the grid around the init_pos
    offset = np.array(init_pos) - (np.array(dimensions) * spacing / 2)
    grid_positions += offset

    # Apply noise
    noise = np.random.normal(loc=mean_noise, scale=mean_noise / 3, size=grid_positions.shape)
    grid_positions += noise

    return grid_positions[:num_agents, 0], grid_positions[:num_agents, 1], grid_positions[:num_agents, 2]


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Accept a dictionary as input.")
    parser.add_argument('--parameters', required=True)
    args = parser.parse_args()
    input_dict = ast.literal_eval(args.parameters)


    if_plot = input_dict["if_plot"]
    seed_rndm = input_dict["seed_rndm"]
    rng = np.random.default_rng(seed_rndm)
    n_agent = input_dict["n_agent"]
    spacing = input_dict["spacing"]
    mean_noise = input_dict["mean_noise"]
    init_x = input_dict["init_x"]
    init_y = input_dict["init_y"]
    init_z = input_dict["init_z"]

    pos_xs, pos_ys, pos_zs = place_agents(n_agent, (init_x, init_y, init_z), spacing, mean_noise)

    theta = np.random.uniform(0, 2*np.pi, n_agent)
    phi = np.random.uniform(0, np.pi, n_agent)

    pos_h_xc = np.sin(phi) * np.cos(theta)
    pos_h_yc = np.sin(phi) * np.sin(theta)
    pos_h_zc = np.cos(phi)

    pos_h_m = np.sqrt(np.square(pos_h_xc) + np.square(pos_h_yc) + np.square(pos_h_zc))

    pos_hxs = np.arccos(pos_h_xc / pos_h_m)
    pos_hys = np.arccos(pos_h_yc / pos_h_m)
    pos_hzs = np.arccos(pos_h_zc / pos_h_m)

    u = np.zeros(n_agent)
    w = np.zeros(n_agent)

    del spacing, init_x, init_y, init_z

    dt = 0.05
    epsilon = 12.0
    sigma_const = input_dict["sigma_const"]
    sigmas = np.full(n_agent, sigma_const)
    sigmas_b = np.full(n_agent, 0.01)
    umax_const = input_dict["umax_const"]
    wmax = input_dict["wmax"]
    alpha = input_dict["alpha"]
    beta = input_dict["beta"]
    k1 = input_dict["k1"]
    k2 = input_dict["k2"]
    exp_index = input_dict["exp_index"]
    sensing_range = 3.0
    run_wall_time = input_dict["run_wall_time"]
    h_alignment = input_dict["h_alignment"]
    self_log = True
    save_dir = "./point_results/"
    noise_pos = input_dict["noise_pos"]
    noise_h = input_dict["noise_h"]

    global boun_x, boun_y, boun_z, boun_thres
    boun_x = input_dict["boun_x"]
    boun_y = input_dict["boun_y"]
    boun_z = input_dict["boun_z"]
    center_pos = np.array((boun_x, boun_y, boun_z)).reshape((3, 1)) / 2
    boun_thresh = 0.5

    min_observed_bound = 99

    if self_log:
        log_pos_xs = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_ys = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_zs = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_hxc = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_hyc = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_hzc = np.zeros([n_agent, int(run_wall_time / dt)])
        log_us = np.zeros([n_agent, int(run_wall_time / dt)])
        log_ws = np.zeros([n_agent, int(run_wall_time / dt)])

    headings = np.stack((pos_h_xc, pos_h_yc, pos_h_zc))
    positions = np.stack((pos_xs, pos_ys, pos_zs))
    begin_all = time.time()
    for i in range(int(run_wall_time / dt)):
        if self_log:
            log_pos_xs[:, i] = positions[0]
            log_pos_ys[:, i] = positions[1]
            log_pos_zs[:, i] = positions[2]
            log_pos_hxc[:, i] = headings[0]
            log_pos_hyc[:, i] = headings[1]
            log_pos_hzc[:, i] = headings[2]
            log_us = u[:]
            log_ws = w[:]

        delta_x = positions[0] - positions[0][:, np.newaxis]
        delta_y = positions[1] - positions[1][:, np.newaxis]
        delta_z = positions[2] - positions[2][:, np.newaxis]
        dist = np.stack((delta_x, delta_y, delta_z))
        dist_norm = np.linalg.norm(dist, axis=0)
        dist_norm[(dist_norm > sensing_range) | (dist_norm == 0)] = np.inf
        dist_norm += rng.uniform(-noise_pos, noise_pos, (n_agent, n_agent)) * dt
        dist_ratio = dist / dist_norm

        forces = -epsilon * (2 * (sigmas**4 / dist_norm ** 5) - (sigmas**2 / dist_norm ** 3))
        force_vecs = alpha * np.sum(forces * dist_ratio, axis=2)

        mean_heading = calculate_av_heading(headings)
        fa_vec = beta * mean_heading * np.ones(np.shape(mean_heading)) * h_alignment

        d_bounds = center_pos - np.abs(positions - center_pos)
        in_bounds = d_bounds < boun_thresh
        if np.any(in_bounds):
            boundary_effect = np.maximum(-epsilon * 5 * (2 * (sigmas_b ** 4 / d_bounds ** 5) - (sigmas_b ** 2 / d_bounds ** 3)), 0)

            f_b = boundary_effect * in_bounds * -np.sign(positions - center_pos)
            force_vecs += fa_vec + f_b
            if np.min(d_bounds) < min_observed_bound:
                min_observed_bound = np.min(d_bounds)
            # print(min_observed_bound)
        else:
            force_vecs += fa_vec

        f_x = force_vecs[0]
        f_y = force_vecs[1]
        f_z = force_vecs[2]

        f_mag = np.linalg.norm(force_vecs, axis=0)
        f_mag = np.maximum(f_mag, 0.00001)

        dot_f_h = np.sum(force_vecs*headings, axis=0)

        cos_dot_f_h = dot_f_h / (f_mag * np.linalg.norm(headings, axis=0))
        ang_f_h = np.arccos(cos_dot_f_h)

        u = k1 * f_mag * np.cos(ang_f_h) + 0.05
        w = k2 * f_mag * np.sin(ang_f_h)

        u = np.clip(u, 0, umax_const)
        w = np.clip(w, -wmax, wmax)

        positions += (u * headings + rng.uniform(-noise_h, noise_h, (3, n_agent))) * dt

        headings = calculate_rotated_vector_batch(
            headings, force_vecs, w * dt)

        pos_h_vec = np.arccos(headings / np.linalg.norm(headings, axis=0))


    if self_log:
        # now = datetime.now()
        # dt_string = now.strftime("%d_%m_%Y|%H_%M_%S")
        dt_string = datetime.utcnow().strftime('%d_%m_%Y|%S_%f')[:-3]
        dir_path = dt_string + "_" + str(exp_index)
        create_empty_directory(save_dir + dir_path)
        save_dir = save_dir + dir_path + "/"
        filename_posx = save_dir + "log_pos_xs_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_posy = save_dir + "log_pos_ys_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_posz = save_dir + "log_pos_zs_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_pos_hxc = save_dir + "log_pos_hxc_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_pos_hyc = save_dir + "log_pos_hyc_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_pos_hzc = save_dir + "log_pos_hzc_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_us = save_dir + "log_us_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        filename_ws = save_dir + "log_ws_" + dt_string + "_" + str(exp_index) + "_" + ".npy"
        np.save(filename_posx, log_pos_xs)
        np.save(filename_posy, log_pos_ys)
        np.save(filename_posz, log_pos_zs)
        np.save(filename_pos_hxc, log_pos_hxc)
        np.save(filename_pos_hyc, log_pos_hyc)
        np.save(filename_pos_hzc, log_pos_hzc)
        np.save(filename_us, log_us)
        np.save(filename_ws, log_ws)
    print("Time elapsed: ", time.time() - begin_all)
    print(dt_string[-7:])

