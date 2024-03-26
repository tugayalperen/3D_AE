import numpy as np
import time
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


def calculate_rotated_vector_batch(X1, Y1, Z1, X2, Y2, Z2, wdt):
    # Convert inputs to NumPy arrays if they aren't already
    # X1, Y1, Z1, X2, Y2, Z2, wdt = [np.asarray(a) for a in [X1, Y1, Z1, X2, Y2, Z2, wdt]]

    # Stack the original and target vectors for batch processing
    vector1 = np.stack([X1, Y1, Z1], axis=-1)
    vector2 = np.stack([X2, Y2, Z2], axis=-1)

    # Calculate magnitudes for normalization
    original_magnitude = np.linalg.norm(vector1, axis=1, keepdims=True)  # I checked, this is always 1 now
    vector2_magnitude = np.linalg.norm(vector2, axis=1, keepdims=True)

    # Normalize and scale vector2
    vector2_normalized_scaled = vector2 * (original_magnitude / vector2_magnitude)

    # Calculate the normal vector for each pair
    normal_vector = np.cross(vector1, vector2_normalized_scaled)
    normal_magnitude = np.linalg.norm(normal_vector, axis=1, keepdims=True)

    # Avoid division by zero by ensuring non-zero magnitude
    normal_vector /= np.where(normal_magnitude > 0, normal_magnitude, 1)

    # Rodrigues' rotation formula for batch
    k_cross_vector1 = np.cross(normal_vector, vector1)
    cos_theta = np.cos(wdt)[:, np.newaxis]
    sin_theta = np.sin(wdt)[:, np.newaxis]
    one_minus_cos_theta = (1 - cos_theta)

    dot_product = np.sum(normal_vector * vector1, axis=1, keepdims=True)
    v_rot = vector1 * cos_theta + k_cross_vector1 * sin_theta + normal_vector * dot_product * one_minus_cos_theta

    return v_rot.T


def calculate_av_heading(x_components, y_components, z_components):
    # Normalize each vector and sum them to get an average direction
    normalized_vectors = []
    for x, y, z in zip(x_components, y_components, z_components):
        vec = np.array([x, y, z])
        norm = np.linalg.norm(vec)
        if norm != 0:  # Avoid division by zero
            normalized_vectors.append(vec / norm)
        else:
            normalized_vectors.append(vec)  # Keep zero vectors as is

    # Calculate the average vector (sum of normalized vectors)
    sum_of_normalized_vectors = np.sum(normalized_vectors, axis=0)

    # Normalize the sum to get the unit vector with the average direction
    unit_vector_average_direction = sum_of_normalized_vectors / np.linalg.norm(sum_of_normalized_vectors)

    return unit_vector_average_direction


def detect_bounds(pos_x, pos_y, pos_z):
    global boun_x, boun_y, boun_z, boun_thresh

    result_x = np.zeros_like(pos_x)
    result_y = np.zeros_like(pos_y)
    result_z = np.zeros_like(pos_z)

    result_x[pos_x < boun_thresh] = 1
    result_x[pos_x > boun_x - boun_thresh] = -1

    result_y[pos_y < boun_thresh] = 1
    result_y[pos_y > boun_y - boun_thresh] = -1

    result_z[pos_z < boun_thresh] = 1
    result_z[pos_z > boun_z - boun_thresh] = -1

    return result_x, result_y, result_z


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
    sigmas_b = np.full(n_agent, 0.05)
    umax_const = input_dict["umax_const"]
    wmax = input_dict["wmax"]
    alpha = input_dict["alpha"]
    beta = input_dict["beta"]
    k1 = input_dict["k1"]
    k2 = input_dict["k2"]
    krep = 50
    l0 = 0.5
    sensing_range = 3.0
    run_wall_time = input_dict["run_wall_time"]
    h_alignment = input_dict["h_alignment"]
    self_log = True
    save_dir = "./point_results/"

    global boun_x, boun_y, boun_z, boun_thres
    boun_x = input_dict["boun_x"]
    boun_y = input_dict["boun_y"]
    boun_z = input_dict["boun_z"]
    boun_thresh = 0.5

    sigma_map = 0.3

    map_x, map_y, map_z = [np.linspace(-1, 1, 150) for _ in range(3)]
    X, Y, Z = np.meshgrid(map_x, map_y, map_z)
    map_3d = 255 * np.exp(-(X ** 2 + Y ** 2 + Z ** 2) / (2 * sigma_map ** 2))
    grad_const_x, grad_const_y, grad_const_z = [150 / boun for boun in (boun_x, boun_y, boun_z)]

    if if_plot:
        plotter = plot_swarm_v2.SwarmPlotter(n_agent, boun_x, boun_y, boun_z)

    min_observed_bound = 99

    if self_log:
        log_pos_xs = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_ys = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_zs = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_hxs = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_hys = np.zeros([n_agent, int(run_wall_time / dt)])
        log_pos_hzs = np.zeros([n_agent, int(run_wall_time / dt)])
        log_us = np.zeros([n_agent, int(run_wall_time / dt)])
        log_ws = np.zeros([n_agent, int(run_wall_time / dt)])

    begin_all = time.time()
    for i in range(int(run_wall_time / dt)):

        if self_log:
            log_pos_xs[:, i] = pos_xs[:]
            log_pos_ys[:, i] = pos_ys[:]
            log_pos_zs[:, i] = pos_zs[:]
            log_pos_hxs[:, i] = pos_hxs[:]
            log_pos_hys[:, i] = pos_hys[:]
            log_pos_hzs[:, i] = pos_hzs[:]
            log_us = u[:]
            log_ws = w[:]

        d_ij = np.hypot(np.hypot(pos_xs[:, None] - pos_xs, pos_ys[:, None] - pos_ys), pos_zs[:, None] - pos_zs)
        d_ij[(d_ij > sensing_range) | (d_ij == 0)] = np.inf

        ij_ang_x = np.arccos((pos_xs - pos_xs[:, None]) / d_ij)
        ij_ang_y = np.arccos((pos_ys - pos_ys[:, None]) / d_ij)
        ij_ang_z = np.arccos((pos_zs - pos_zs[:, None]) / d_ij)

        grad_x = np.clip(np.ceil(pos_xs * grad_const_x).astype(int), 0, 149)
        grad_y = np.clip(np.ceil(pos_ys * grad_const_y).astype(int), 0, 149)
        grad_z = np.clip(np.ceil(pos_zs * grad_const_z).astype(int), 0, 149)

        grad_vals = map_3d[grad_x, grad_y, grad_z]
        grad_vals = np.clip(grad_vals, 0, 255)

        forces = -epsilon * (2 * (sigmas[:, np.newaxis] ** 4 / d_ij ** 5) - (sigmas[:, np.newaxis] ** 2 / d_ij ** 3))

        cos_ij_ang_x = np.cos(ij_ang_x)
        cos_ij_ang_y = np.cos(ij_ang_y)
        cos_ij_ang_z = np.cos(ij_ang_z)

        f_x = alpha * np.sum(forces * cos_ij_ang_x, axis=1)
        f_x = np.where(f_x == 0, 0.00001, f_x)

        f_y = alpha * np.sum(forces * cos_ij_ang_y, axis=1)
        f_y = np.where(f_y == 0, 0.00001, f_y)

        f_z = alpha * np.sum(forces * cos_ij_ang_z, axis=1)
        f_z = np.where(f_z == 0, 0.00001, f_z)

        av_heading = calculate_av_heading(pos_h_xc, pos_h_yc, pos_h_zc)

        fa_x = int(h_alignment) * beta * av_heading[0]
        fa_y = int(h_alignment) * beta * av_heading[1]
        fa_z = int(h_alignment) * beta * av_heading[2]

        d_bxi = np.minimum(np.abs(boun_x - pos_xs), pos_xs)
        d_byi = np.minimum(np.abs(boun_y - pos_ys), pos_ys)
        d_bzi = np.minimum(np.abs(boun_z - pos_zs), pos_zs)

        close_to_bound_x = np.logical_or(pos_xs < boun_thresh, pos_xs > (boun_x - boun_thresh))
        close_to_bound_y = np.logical_or(pos_ys < boun_thresh, pos_ys > (boun_y - boun_thresh))
        close_to_bound_z = np.logical_or(pos_zs < boun_thresh, pos_zs > (boun_z - boun_thresh))

        if np.any(close_to_bound_x) or np.any(close_to_bound_y) or np.any(close_to_bound_z):
            db_bxi, db_byi, db_bzi = detect_bounds(pos_xs, pos_ys, pos_zs)

            boundary_effect_x = -epsilon*5 * (2 * (sigmas_b ** 4 / d_bxi ** 5) -
                                          (sigmas_b ** 2 / d_bxi ** 3))
            boundary_effect_y = -epsilon*5 * (2 * (sigmas_b ** 4 / d_byi ** 5) -
                                          (sigmas_b ** 2 / d_byi ** 3))
            boundary_effect_z = -epsilon*5 * (2 * (sigmas_b ** 4 / d_bzi ** 5) -
                                          (sigmas_b ** 2 / d_bzi ** 3))

            boundary_effect_x[boundary_effect_x < 0] = 0.0
            boundary_effect_y[boundary_effect_y < 0] = 0.0
            boundary_effect_z[boundary_effect_z < 0] = 0.0

            f_x += fa_x + boundary_effect_x * db_bxi
            f_y += fa_y + boundary_effect_y * db_byi
            f_z += fa_z + boundary_effect_z * db_bzi

            if np.min(np.concatenate((d_bxi, d_byi, d_bzi))) < min_observed_bound:
                min_observed_bound = np.min(np.concatenate((d_bxi, d_byi, d_bzi)))

        else:
            f_x += fa_x
            f_y += fa_y
            f_z += fa_z

        f_mag = np.sqrt(np.square(f_x) + np.square(f_y) + np.square(f_z))
        f_mag = np.where(f_mag == 0, 0.00001, f_mag)

        dot_f_h = f_x * pos_h_xc + f_y * pos_h_yc + f_z * pos_h_zc
        cos_dot_f_h = dot_f_h / (f_mag * np.sqrt(pos_h_xc ** 2 + pos_h_yc ** 2 + pos_h_zc ** 2))
        ang_f_h = np.arccos(cos_dot_f_h)

        u = k1 * f_mag * np.cos(ang_f_h) + 0.05
        w = k2 * f_mag * np.sin(ang_f_h)

        u = np.clip(u, 0, umax_const)
        w = np.clip(w, -wmax, wmax)

        pos_xs += u * np.cos(pos_hxs) * dt
        pos_ys += u * np.cos(pos_hys) * dt
        pos_zs += u * np.cos(pos_hzs) * dt

        v_rot = calculate_rotated_vector_batch(
            pos_h_xc, pos_h_yc, pos_h_zc, f_x, f_y, f_z, w * dt)

        pos_h_xc = v_rot[0, :]
        pos_h_yc = v_rot[1, :]
        pos_h_zc = v_rot[2, :]
        pos_h_m = np.sqrt(np.square(v_rot[0]) + np.square(v_rot[1]) + np.square(v_rot[2]))

        pos_hxs = np.arccos(pos_h_xc / pos_h_m)
        pos_hys = np.arccos(pos_h_yc / pos_h_m)
        pos_hzs = np.arccos(pos_h_zc / pos_h_m)


        if (not (i % 7)) and (i > 10):
            plotter.update_plot(pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs)

    if self_log:
        # now = datetime.now()
        # dt_string = now.strftime("%d_%m_%Y|%H_%M_%S")
        dt_string = datetime.utcnow().strftime('%d_%m_%Y|%H_%M_%S_%f')[:-3]
        dir_path = dt_string
        create_empty_directory(save_dir + dir_path)
        save_dir = save_dir + dir_path + "/"
        filename_posx = save_dir + "log_pos_xs_" + dt_string + ".npy"
        filename_posy = save_dir + "log_pos_ys_" + dt_string + ".npy"
        filename_posz = save_dir + "log_pos_zs_" + dt_string + ".npy"
        filename_pos_hxs = save_dir + "log_pos_hxs_" + dt_string + ".npy"
        filename_pos_hys = save_dir + "log_pos_hys_" + dt_string + ".npy"
        filename_pos_hzs = save_dir + "log_pos_hzs_" + dt_string + ".npy"
        filename_us = save_dir + "log_us_" + dt_string + ".npy"
        filename_ws = save_dir + "log_ws_" + dt_string + ".npy"
        np.save(filename_posx, log_pos_xs)
        np.save(filename_posy, log_pos_ys)
        np.save(filename_posz, log_pos_zs)
        np.save(filename_pos_hxs, log_pos_hxs)
        np.save(filename_pos_hys, log_pos_hys)
        np.save(filename_pos_hzs, log_pos_hzs)
        np.save(filename_us, log_us)
        np.save(filename_ws, log_ws)
    print("Time elapsed: ", time.time() - begin_all)

