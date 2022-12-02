import sympy
sympy.init_printing(use_latex='mathjax')
from sympy import symbols, Matrix
from IPython.display import display
from numpy import array, sqrt
import numpy as np 
from numpy.random import randn
from math import sqrt, tan, cos, sin, atan2
import matplotlib.pyplot as plt
from math import sqrt
from copy import deepcopy
from matplotlib.patches import Ellipse
import math

class EKFLocalization():
    def __init__(self, dt, std_vel, std_steer, dim_x=3, dim_z=2, dim_u=2, sensor_size=2):
        self.dt = dt
        self.std_vel = std_vel
        self.std_steer = std_steer
        self.get_linearized_motion_model()
        self.subs = {self._x: 0, self._y: 0, self._vt:0, self._wt:0, self._dt:dt, self._theta:0}
        
        self.x = np.zeros((dim_x, 1)) # state
        self.P = np.eye(dim_x)        # uncertainty covariance
        self.R = np.eye(dim_z)        # state uncertainty
        self.Q = np.eye(dim_x)        # process uncertainty
        self.y = np.zeros((dim_z, 1)) # residual
        self.z =  np.array([None]*dim_z)
        self.K = np.zeros(self.x.shape) # kalman gain
        self.y = np.zeros((dim_z, 1))
        self._I = np.eye(dim_x)
        self.dim_z = dim_z
        self.dim_x = dim_x 
        self.sensor_size = 2
        

    def get_linearized_motion_model(self):
        x, y, theta, vt, wt, dt = sympy.symbols('x, y, theta, v_t, omega_t, delta_t')
        f = sympy.Matrix([[x+-(vt/wt)*sympy.sin(theta)+ (vt/wt)*sympy.sin(theta+ wt*dt)]
                  ,[y+ (vt/wt)*sympy.cos(theta) - (vt/wt)*sympy.cos(theta+ wt*dt)]
                  , [theta+wt*dt]])
        self._x, self._y, self._theta, self._vt, self._wt, self._dt = x, y, theta, vt, wt, dt
        self.state = sympy.Matrix([x, y, theta])
        self.control = sympy.Matrix([vt, wt])
        self.F_j = f.jacobian(self.state)
        self.V_j = f.jacobian(self.control)
        return
    
    def x_forward(self, x, u, dt):
        r = u[0]/u[1]
        theta = x[2]
        rotation = x[2] + u[1]*dt 
        x_plus = np.array([x[0] + -r*sin(theta) + r*sin(rotation),
                           x[1] + r*cos(theta) - r*cos(rotation),
                           x[2] + u[1]*dt])
        return  x_plus 
        
    def get_linearized_measurement_model(self, x, landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]
        hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
        dist = sqrt(hyp)
        Hx = array([[dist],[atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
        H = array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0]
                , [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
        return Hx, H
        

    def residual(self, a, b):
        """ compute residual (a-b) between measurements containing 
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
        y = a - b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # to [-pi, pi)
            y[1] -= 2 * np.pi
        return y
    
    def predict(self, u):
        self.x = self.x_forward(self.x, u, self.dt)
        self.subs[self._x] = self.x[0, 0]
        self.subs[self._y] = self.x[1, 0]
        self.subs[self._theta] = self.x[2, 0]
        self.subs[self._vt] = u[0]
        self.subs[self._wt] = u[1]

        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance in the control space
        M = array([[self.std_vel**2, 0],  [0, self.std_steer**2]])

        self.P = F @ self.P @ F.T + V @ M @ V.T + self.Q

    def ekf_update(self, z, landmarks):
        H_total = np.empty((self.dim_z, self.dim_x))
        Hx_total = np.empty((self.dim_z, 1))
        for id, land_mark in enumerate(landmarks):
            Hx, H = self.get_linearized_measurement_model(self.x, land_mark)
            H_total[id*H.shape[0]:(id+1)*H.shape[0], 0:self.dim_x] = H
            Hx_total[id*H.shape[0]:(id+1)*H.shape[0], 0:1] = Hx

        H = H_total
        Hx = Hx_total 
        PHT = np.dot(self.P, H.T)
        self.K = PHT.dot(np.linalg.inv(np.dot(H, PHT) + self.R))
        self.y = np.empty(self.dim_z)

        for id in range(0, int(Hx.shape[0]/self.sensor_size)):
            y_i = self.residual(z[id*self.sensor_size:(id+1)*self.sensor_size], Hx[id*self.sensor_size:(id+1)*self.sensor_size, 0:1].flatten())
            self.y[id*self.sensor_size:(id+1)*self.sensor_size] = y_i
       
        self.x = self.x + np.reshape(np.dot(self.K, self.y),(self.x.shape))

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # P = (I-KH)P is the optimal gain 
        I_KH = self._I - np.dot(self.K, H)
        self.P = np.dot(I_KH, self.P).dot(I_KH.T) + np.dot(self.K, self.R).dot(self.K.T)
        self.z = deepcopy(z)
        
    def z_landmark(self, lmark, sim_pos, std_rng, std_brg):
        x, y = sim_pos[0, 0], sim_pos[1, 0]
        d = np.sqrt((lmark[0] - x)**2 + (lmark[1] - y)**2)  
        a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
        z = np.array([[d + randn()*std_rng],
                    [a + randn()*std_brg]])
        return z
    
    def covariance_ellipse(self, P, deviations=1):
        U, s, _ = np.linalg.svd(P)
        orientation = math.atan2(U[1, 0], U[0, 0])
        width = deviations * math.sqrt(s[0])
        height = deviations * math.sqrt(s[1])
        if height > width:
            raise ValueError('width must be greater than height')
        return (orientation, width, height)
    
    
    def plot_covariance_ellipse(self, mean, cov, std=None, facecolor='b', edgecolor='g', alpha=0.7,ls='solid'):
        ellipse = self.covariance_ellipse(cov)
        ax = plt.gca()
        angle = np.degrees(ellipse[0])
        width = ellipse[1] * 2.
        height = ellipse[2] * 2.
        e = Ellipse(xy=mean, width=std*width, height=std*height, angle=angle, facecolor=facecolor, edgecolor=edgecolor, alpha=alpha, lw=2, ls=ls)
        ax.add_patch(e)
        x, y = mean
        plt.scatter(x, y, marker='+', color=edgecolor)
        a = ellipse[0]
        h, w = height/4, width/4
        plt.plot([x, x+ h*cos(a+np.pi/2)], [y, y + h*sin(a+np.pi/2)])
        plt.plot([x, x+ w*cos(a)], [y, y + w*sin(a)])
    
    def run_localization(self, landmarks, std_range, std_bearing, step=10, ellipse_step=2000, ylim=None, iteration_num=5):
        self.x = array([[2, 6, .3]]).T # x, y, steer angle
        self.P = np.diag([.1, .1, .1])
        self.R[0,0] = std_range**2
        self.R[1,1] = std_bearing**2
        sim_pos = self.x.copy()
        u = array([1.1, .01]) 
        plt.figure()
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=60)
        track = []
        for i in range(iteration_num):
            sim_pos = self.x_forward(sim_pos, u, dt) # simulate robot
            track.append(sim_pos)
            if i % step == 0:
                self.predict(u=u)
                if i % ellipse_step == 0:
                    self.plot_covariance_ellipse((self.x[0,0], self.x[1,0]), self.P[0:2, 0:2], std=6, facecolor='k', alpha=0.3)
                
                z_all_vec = []
                for lmark in landmarks:
                    z = self.z_landmark(lmark, sim_pos, std_range, std_bearing)
                    z_all_vec.append(z)
                z_all_vec = np.array(z_all_vec).flatten()
                self.ekf_update(z_all_vec, landmarks)
                if i % ellipse_step == 0:
                    self.plot_covariance_ellipse((self.x[0,0], self.x[1,0]), self.P[0:2, 0:2], std=6, facecolor='g', alpha=0.8)
        track = np.array(track)
        plt.plot(track[:, 0], track[:,1], color='k', lw=2)
        plt.axis('equal')
        plt.title("EKF Robot localization")
        if ylim is not None: plt.ylim(*ylim)
        plt.show()
        return ekf
        
dt = 0.1
landmarks = array([[50, 100], [40, 90], [150, 150], [-150, 200]])
ekf = EKFLocalization(dt, std_vel=5.1, std_steer=np.radians(1), dim_z=landmarks.shape[0]*2, sensor_size=2)
ekf.run_localization(landmarks, std_range=0.3, std_bearing=0.1, ellipse_step=200, iteration_num=2000)