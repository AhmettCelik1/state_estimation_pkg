#!/usr/bin/env python3
from PIL import ImageTk
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, dt, process_noise_var, measurement_noise_var, initial_state, initial_covariance):
        self.dt = dt
        self.A = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  
        
        self.H = np.array([[1, 0, 0, 0],  
                           [0, 1, 0, 0]])
        
        self.x = initial_state 
        self.P = initial_covariance 
        self.Q = np.array([[process_noise_var, 0, 0, 0],
                           [0, process_noise_var, 0, 0],
                           [0, 0, process_noise_var, 0],
                           [0, 0, 0, process_noise_var]])  
        
        self.R = np.array([[measurement_noise_var, 0],
                           [0, measurement_noise_var]]) 
        
        self.I = np.eye(4) 

    def predict(self):
        self.x = np.dot(self.A, self.x) 
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q  

    def update(self, measurement, version=1):
        y = measurement - np.dot(self.H, self.x) 
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R  
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  
        
        self.x = self.x + np.dot(K, y)  
        I_KH = self.I - np.dot(K, self.H)  
        
        if version == 1:
            self.P = np.dot(I_KH, self.P) 
        else:
            self.P = np.dot(np.dot(I_KH, self.P), I_KH.T) + np.dot(np.dot(K, self.R), K.T)  

    def get_state(self):
        return self.x


dt = 1.0 
n_steps = 50 

process_noise_variance = 2.1  
measurement_noise_variance = 2.0 

true_state = np.array([[0], [0], [0], [0]]) 

initial_covariance = np.eye(4)

kf_v1 = KalmanFilter(dt, process_noise_variance, measurement_noise_variance, true_state, initial_covariance)
kf_v2 = KalmanFilter(dt, process_noise_variance, measurement_noise_variance, true_state, initial_covariance)


true_x, true_y = [], []
measured_x, measured_y = [], []
estimated_x_v1, estimated_y_v1 = [], []
estimated_x_v2, estimated_y_v2 = [], []

rms_v1, rms_v2 = [], []

for _ in range(n_steps):
   
    true_state = np.dot(kf_v1.A, true_state) 
    process_noise = np.random.multivariate_normal([0, 0, 0, 0], kf_v1.Q) 
    true_state = true_state + process_noise[:, np.newaxis]  
    
    true_x.append(true_state[0][0])
    true_y.append(true_state[1][0])
    
    measurement = np.array([[true_state[0][0] + np.random.randn() * measurement_noise_variance**0.5],
                            [true_state[1][0] + np.random.randn() * measurement_noise_variance**0.5]])
    measured_x.append(measurement[0][0])
    measured_y.append(measurement[1][0])
    
    kf_v1.predict()
    kf_v1.update(measurement, version=1)
    estimated_x_v1.append(kf_v1.get_state()[0][0])
    estimated_y_v1.append(kf_v1.get_state()[1][0])
    
    kf_v2.predict()
    kf_v2.update(measurement, version=2)
    estimated_x_v2.append(kf_v2.get_state()[0][0])
    estimated_y_v2.append(kf_v2.get_state()[1][0])

    rms_v1.append(np.sqrt(np.mean((np.array(estimated_x_v1) - np.array(true_x))**2 + 
                                   (np.array(estimated_y_v1) - np.array(true_y))**2)))
    rms_v2.append(np.sqrt(np.mean((np.array(estimated_x_v2) - np.array(true_x))**2 + 
                                   (np.array(estimated_y_v2) - np.array(true_y))**2)))

fig, axs = plt.subplots(2, 1, figsize=(10, 10))

axs[0].plot(true_x, true_y, label="True Trajectory", color='g', linestyle='--')
axs[0].plot(estimated_x_v1, estimated_y_v1, label="LKF V1 State (x, y)", color='r')
axs[0].scatter(measured_x, measured_y, label="Measurement (x, y)", color='b', marker='x')
axs[0].set_title('LKF V1: True Trajectory, State, and Measurements')
axs[0].set_xlabel('X Position')
axs[0].set_ylabel('Y Position')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(true_x, true_y, label="True Trajectory", color='g', linestyle='--')
axs[1].plot(estimated_x_v2, estimated_y_v2, label="LKF V2 State (x, y)", color='purple')
axs[1].scatter(measured_x, measured_y, label="Measurement (x, y)", color='b', marker='x')
axs[1].set_title('LKF V2: True Trajectory, State, and Measurements')
axs[1].set_xlabel('X Position')
axs[1].set_ylabel('Y Position')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()

sum_rms_v1 = np.sum(rms_v1)
sum_rms_v2 = np.sum(rms_v2)

fig_rms, ax_rms = plt.subplots(2, 1, figsize=(10, 10))

ax_rms[0].plot(range(n_steps), rms_v1, label=f"RMS Error V1 (Sum: {sum_rms_v1:.2f})", color='r')
ax_rms[0].set_title('RMS Error for LKF V1')
ax_rms[0].set_xlabel('Time Step')
ax_rms[0].set_ylabel('RMS Error (Position in meters)')
ax_rms[0].grid(True)

ax_rms[1].plot(range(n_steps), rms_v2, label=f"RMS Error V2 (Sum: {sum_rms_v2:.2f})", color='purple')
ax_rms[1].set_title('RMS Error for LKF V2')
ax_rms[1].set_xlabel('Time Step')
ax_rms[1].set_ylabel('RMS Error (Position in meters)')
ax_rms[1].grid(True)

ax_rms[0].legend()
ax_rms[1].legend()

plt.tight_layout()
plt.show()
