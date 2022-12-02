import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class EKFLocalization():
    def __init__(self, dt, var_v, var_z, map_data):
        self.delta_t = dt
        self.var_v = var_v
        self.var_z = var_z
        self.map_data = map_data
        self.num_beacons = map_data.shape[0]
        

    # sample normal distribution with variance = var
    def sample_normal_distribution(self, var):
        rand = np.random.uniform(low=-1.0, high=1.0, size=12)
        return var * np.sum(rand) / 6

    def motion_model(self, x, u):
        """
        Compute motion model
        Input:
            x (np.ndarray) - shape 3,1 : robot location in 3D space
            u (np.ndarray) - shape 3,1 : motion command (robot velocities)
        Output:
            xt (np.ndarray) - shape 3,1 : new robot location, i.e., current_location + sampling_time * u  
        """
        new_location = x + u*self.delta_t
        return new_location

    def measurement_model(self, x, m):
        """
        Compute measurement model given state x (robot loc.) & map m
        Input
            x (np.ndarray) - shape 3,1 : robot location in 3D space
            m (np.ndarray) - shape num_beacons,m : location of all beacons in the map
        Output:
            z (np.ndarray) - shape num_beacons, : distance from robot to each beacons
        """
        return np.linalg.norm(m-x[:,0], axis=1)

    def simulation_step(self, x, u):
        """
        Perform 1 step of simulation
        Input:
            x (np.ndarray) - shape (3,1) : robot location in 3D space
            u (np.ndarray) - shape (3,1): motion command (robot velocities)
        Output:
            xt (np.ndarray) - shape (3,1) : new robot location
            zt (np.ndarray) - shape (num_beacons,) : measurement at new location
        """
        # Perturb motion command
        u_noise = np.array([self.sample_normal_distribution(self.var_v) for i in range(3)])
        u_hat = u + u_noise
        
        # Compute new location
        xt = self.motion_model(x, u)  # here xt is the ground truth of robot location
        
        # Compute expected value of measurement
        zt_bar = self.measurement_model(xt, self.map_data) 
        # use self.sample_normal_distribution to generate white noise with variance self.var_z, i.e.,  zt = zt_bar + white noise 
        
        zt = zt_bar + np.array([self.sample_normal_distribution(self.var_z) for i in range(len(zt_bar))]) 
        # zt = zt_bar + self.sample_normal_distribution(self.var_z)*np.ones(len(zt_bar))
        return xt, zt

    def ekf(self, muy, sigma, u, z, m):
        """
        Implement an EKF to track robot location
        Input:
            muy (np.ndarray) - shape (3,1): expected robot location at previous time step (t-1)
            sigma (np.ndarray) - shape (3,3): covariance of robot location at previous time step (t-1)
            u (np.ndarray) - shape (3,1): motion command at this time step (t)
            z (np.ndarray) - shape (num_beacons,): measurement at this time step (t)
            m (np.ndarray) - shape (num_beacons,3): feature map contains location of all beacons
        Output:
            muy_t (np.ndarray) - shape (3,1): expected robot location at this time step (t)
            sigma_t (np.ndarray) - shape (3,3): covariance of robot location at this time step (t)
        """
        # predict location based on motion_model
        muy_bar =  self.motion_model(muy,u)
        # predict covariance
        sigma_bar = sigma + self.delta_t * np.diag([self.var_v, self.var_v, self.var_v])
        # measurement covariance
        Q = np.array([[self.var_z]])
        expected_meas = self.measurement_model(muy_bar, m)
        for i in range(self.num_beacons):
            # expected measurement for the ith measurement #TODO 
            z_hat = expected_meas[i] 
            # jacobian of measurement model
            q = np.sum((m[i, :] - muy_bar)**2)
            x_bar, y_bar, z_bar = muy_bar.squeeze() 
            H = np.array([[x_bar - m[i, 0], y_bar - m[i, 1], z_bar - m[i, 2]]]) / np.sqrt(q)
            # kalman update
            S = H @ sigma_bar @ H.T + Q
            K = sigma_bar @ H.T / S.squeeze()
            muy_bar += K * (z[i] - z_hat)
            sigma_bar = (np.eye(3) - K @ H) @ sigma_bar
        
        return muy_bar, sigma_bar
    
    def run_localization(self, ):
        # initialize
        t = 0
        muy = np.zeros((3,1))  # initial location
        sigma = np.diag([0.1, 0.1, 0.1])  # initial covariance
        u = np.array([[0.5, 0., 0.5]]).T  # keep constant for the whole trajectory

        ground_truth = []
        ekf_re = []
        est_std_dev = []
        time = []

        # main loop
        while t < 10.:
            # advance simulation 1 step 
            x_true, z = self.simulation_step(muy, u)
            
            # estimate new location
            muy_bar, sigma_bar = self.ekf(muy, sigma, u, z, self.map_data)
            
            # store ground truth & estimate
            ground_truth.append(x_true.reshape(1, -1))
            ekf_re.append(muy_bar.reshape(1, -1))
            est_std_dev.append(np.sqrt(np.diag(sigma_bar)))
            time.append(t)
            
            # update muy & sigma
            t += self.delta_t
            muy = muy_bar
            sigma = sigma_bar
            
        ground_truth = np.vstack(ground_truth)
        ekf_re = np.vstack(ekf_re)
        est_std_dev = np.vstack(est_std_dev)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(ground_truth[:, 0], ground_truth[:, 1], ground_truth[:, 2], 'gray', label='truth')
        ax.scatter3D(ekf_re[:, 0], ekf_re[:, 1], ekf_re[:, 2], cmap='Greens', label='ekf')
        print(ekf_re.shape)
        print(self.map_data.shape)
        ax.scatter3D(self.map_data[:, 0], self.map_data[:, 1], self.map_data[:, 2], label='beacons')
        ax.set_xlabel('x(m)')
        ax.set_ylabel('y(m)')
        ax.set_zlabel('z(m)')
        ax.legend()
        ax.set_title('3D Trajectory')
        plt.show()
        
        
beacon_locations = np.array([[12., 2., 0.],[5, 9., 0.], [4, 9, 2],[0., 1, 8]])  # each row defines 1 beacon
var_v = 1.  # m^2/s^2
var_z = 1.  # m^2
delta_t = 0.25  # s

ekf_localization = EKFLocalization(delta_t, var_v, var_z, beacon_locations)
ekf_localization.run_localization()