import numpy as np
import plot_swarm_v2


class FlockingUtilsVec:
    def __init__(self, n_agents, center_x, center_y, center_z, spacing, input_dict):
        self.n_agents = n_agents
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.spacing = spacing

        self.boun_thresh = 0.5
        self.boun_x = 10000
        self.boun_y = 10000
        self.boun_z = 10000
        self.center_pos = np.array((self.boun_x, self.boun_y, self.boun_z)).reshape((3, 1)) / 2
        self.sensing_range = 3.0
        self.sigma = 0.5
        self.sigma = input_dict["sigma_const"]
        self.sigmas = self.sigma * np.ones(n_agents)
        self.sigmas2 = self.sigmas**2
        self.sigmas4 = self.sigmas**2
        self.sigmas_b = 0.05 * np.ones(n_agents)
        self.sigmas_b2 = self.sigmas_b**2
        self.sigmas_b4 = self.sigmas_b**4
        self.epsilon = 12.0
        self.alpha = input_dict["alpha"]
        self.beta = input_dict["beta"]
        self.k1 = input_dict["k1"]
        self.k2 = input_dict["k2"]
        self.umax_const = input_dict["umax_const"]
        self.wmax = input_dict["wmax"]
        self.h_alignment = input_dict["h_alignment"]
        self.dt = 0.042
        self.noise_pos = 0.05
        self.noise_h = np.pi / 72
        self.rng = np.random.default_rng(1234)
        self.mean_noise = 0.5

        self.headings = None
        self.positions = None
        self.fa_vec = np.zeros((3, n_agents))
        self.force_vecs = np.zeros((3, n_agents))
        self.dist_ratio = None
        self.dist_norm = None
        self.forces = None
        self.u = np.zeros(n_agents)
        self.w = np.zeros(n_agents)

        map_x, map_y, map_z = [np.linspace(-1, 1, 150) for _ in range(3)]
        X, Y, Z = np.meshgrid(map_x, map_y, map_z)
        self.map_3d = 255 * np.exp(-(X ** 2 + Y ** 2 + Z ** 2) / (2 * self.sigma ** 2))
        self.grad_const_x, self.grad_const_y, self.grad_const_z = [150 / boun for boun in (self.boun_x, self.boun_y, self.boun_z)]

        self.plotter = plot_swarm_v2.SwarmPlotter(self.n_agents, self.boun_x, self.boun_y, self.boun_z)

    def initialize_positions(self):
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
        num_agents = self.n_agents
        spacing = self.spacing
        init_pos = (self.center_x, self.center_y, self.center_z)
        mean_noise = self.mean_noise

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

        theta = np.random.uniform(0, 2 * np.pi, self.n_agents)
        phi = np.random.uniform(0, np.pi, self.n_agents)

        pos_h_xc = np.sin(phi) * np.cos(theta)
        pos_h_yc = np.sin(phi) * np.sin(theta)
        pos_h_zc = np.cos(phi)

        agent_positions = grid_positions[:num_agents, :]
        self.positions = agent_positions.T
        self.headings = np.stack((pos_h_xc, pos_h_yc, pos_h_zc))

        return (self.positions, self.headings)

    def calculate_rotated_vector_batch(self, V1, V2, wdt):
        e3 = np.cross(V1.T, V2.T)

        # Rodrigues' rotation formula for batch
        e2 = np.cross(e3, V1.T).T
        e2_norm = np.linalg.norm(e2, axis=0)
        e2_norm[e2_norm <= 0] = 1
        e2 /= e2_norm

        v_rot = V1 * np.cos(wdt) + e2 * np.sin(wdt) * np.linalg.norm(V1, axis=0)
        return v_rot

    def calculate_av_heading(self, heading_vecs):
        # Normalize each vector and sum them to get an average direction
        heading_vecs_normalised = heading_vecs / np.linalg.norm(heading_vecs, axis=0)

        # Calculate the average vector (sum of normalized vectors)
        sum_of_heading_vecs_normalised = np.sum(heading_vecs_normalised, axis=1)

        # Normalize the sum to get the unit vector with the average direction
        unit_vector_average_direction = sum_of_heading_vecs_normalised / np.linalg.norm(sum_of_heading_vecs_normalised)
        return unit_vector_average_direction.reshape(3, 1)

    def calc_dij(self, positions):
        delta_x = positions[0] - positions[0][:, np.newaxis]
        delta_y = positions[1] - positions[1][:, np.newaxis]
        delta_z = positions[2] - positions[2][:, np.newaxis]
        dist = np.stack((delta_x, delta_y, delta_z))
        self.dist_norm = np.linalg.norm(dist, axis=0)
        self.dist_norm[(self.dist_norm > self.sensing_range) | (self.dist_norm == 0)] = np.inf
        self.dist_ratio = dist / self.dist_norm

    def calc_p_forces(self):
        self.forces = -self.epsilon * (2 * (self.sigmas4 / self.dist_norm ** 5) - (self.sigmas2 / self.dist_norm ** 3))
        self.force_vecs = self.alpha * np.sum(self.forces * self.dist_ratio, axis=2)

    def calc_alignment_forces(self):
        av_heading = self.calculate_av_heading(self.headings)
        self.fa_vec = int(self.h_alignment) * self.beta * av_heading

    def calc_boun_rep(self, positions):
        d_bounds = self.center_pos - np.abs(positions - self.center_pos)
        in_bounds = d_bounds < self.boun_thresh
        if np.any(in_bounds):
            boundary_effect = np.maximum(-self.epsilon * 5 * (2 * (self.sigmas_b4 / d_bounds ** 5) - (self.sigmas_b2 / d_bounds ** 3)), 0)

            f_b =  boundary_effect * in_bounds * -np.sign(positions - self.center_pos)
            self.force_vecs += self.fa_vec + f_b
        else:
            self.force_vecs += self.fa_vec

    def calc_u_w(self):
        f_mag = np.linalg.norm(self.force_vecs, axis=0)
        f_mag = np.maximum(f_mag, 0.00001)

        dot_f_h = np.sum(self.force_vecs*self.headings, axis=0)

        cos_dot_f_h = dot_f_h / (f_mag * np.linalg.norm(self.headings, axis=0))
        np.clip(cos_dot_f_h, -1, 1, out=cos_dot_f_h)
        ang_f_h = np.arccos(cos_dot_f_h)

        self.u = self.k1 * f_mag * np.cos(ang_f_h) + 0.05
        self.w = self.k2 * f_mag * np.sin(ang_f_h)

        self.u = np.clip(self.u, 0, self.umax_const)
        self.w = np.clip(self.w, -self.wmax, self.wmax)
        return self.u

    def get_heading(self):
        return self.headings

    def update_heading(self):
        self.headings = self.calculate_rotated_vector_batch(self.headings, self.force_vecs, self.w * self.dt)

    def plot_swarm(self, pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs):
        self.plotter.update_plot(pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs)