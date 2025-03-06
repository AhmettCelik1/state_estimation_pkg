import numpy as np
import matplotlib.pyplot as plt
import sys
from sympy import *


class ExtendedKalmanFilter:
    def __init__(self, dt_numeric, QV1_numeric,QV2_numeric, R_numeric, num_landmarks_numeric, landmarks_numeric, sensor_range_numeric):
        self.dt_numeric = dt_numeric  
        self.QV1 = QV1_numeric
        self.QV2 = QV2_numeric
        self.R = R_numeric  
        self.num_landmarks = num_landmarks_numeric
        self.landmarks = landmarks_numeric  
        self.sensor_range = sensor_range_numeric
        
        # x_est = [x, y, yaw, v_x, v_y]
        self.x_est = np.array([[0], [0], [0], [0], [0]])
        
        # Covariance matrix P
        self.P = np.zeros((5, 5))
        
        # Initialize  extended kalman filter state
        # x = [x1,x2,x3,x4,x5]
        # x1 = x
        # x2 = y
        # x3 = yaw
        # x4 = v_x
        # x5 = v_y
        self.x=MatrixSymbol('x',5,1)
        
        # Initialize control input
        # u = [yaw_rate, ax, ay]
        self.u=MatrixSymbol('u',3,1)
        
        # Initialize noise input
        # n = [n_omega, n_ax, n_ay]
        self.n=MatrixSymbol('n',3,1)
        
        #parameters
        self.dt=symbols('dt')
        
        # defined symoblic translational x y and rotational theta
        self.theta, self.tx, self.ty = symbols('theta tx ty ')
        
        
        # THE PROCESS MODEL
        # x_{k+1} = x_k + v_x_k*dt*cos(yaw_k) - v_y_k*dt*sin(yaw_k)
        # y_{k+1} = y_k + v_x_k*dt*sin(yaw_k) + v_y_k*dt*cos(yaw_k)
        # yaw_{k+1} = yaw_k + omega_k*dt + n_omega_k*dt
        # v_x_{k+1} = v_x_k + ax_k*dt + dt*v_y_k*omega_k + n_ax_k*dt + dt*v_y_k*n_omega_k
        # v_y_{k+1} = v_y_k + ay_k*dt - dt*v_x_k*omega_k + n_ay_k*dt - dt*v_x_k*n_omega_k
        # x[0] = x_k
        # x[1] = y_k
        # x[2] = yaw_k
        # x[3] = v_x_k
        # x[4] = v_y_k
        self.f=Matrix([self.x[0]+self.x[3]*self.dt*cos(self.x[2])-self.x[4]*self.dt*sin(self.x[2]),
                       self.x[1]+self.x[3]*self.dt*sin(self.x[2])+self.x[4]*self.dt*cos(self.x[2]),
                       self.x[2]+self.u[0]*self.dt + self.n[0]*self.dt,
                       self.x[3]+self.u[1]*self.dt + self.dt*self.x[4]*self.u[0] + self.n[1]*self.dt + self.dt*self.x[4]*self.n[0],
                       self.x[4]+self.u[2]*self.dt - self.dt*self.x[3]*self.u[0] + self.n[2]*self.dt - self.dt*self.x[3]*self.n[0]])
        
        self.f=self.f.subs({self.dt : self.dt_numeric})
        
        self.JacobianState =self.f.jacobian(self.x)
        self.JacobianInput =self.f.jacobian(self.u)
        self.JacobianNoise =self.f.jacobian(self.n)
        

        # Measurement matrix calculates realtive x and y position using landmark and global position drone x y 
        self.h=Matrix([   self.landmarks[0][0] * cos(self.x[2]) + self.landmarks[0][1]*sin(self.x[2]) - self.x[0]*cos(self.x[2]) - self.x[1]*sin(self.x[2]),
                        - self.landmarks[0][0] * sin(self.x[2]) + self.landmarks[0][1]*cos(self.x[2]) + self.x[0]*sin(self.x[2]) - self.x[1]*cos(self.x[2])  ])

        self.JacobianObservation =self.h.jacobian(self.x)

    def calc_input(self):
        yawrate = 1.7  # [rad/s]
        ax = 2.0  # [m/s^2]
        ay = 3.4 # [m/s^2]
        u = np.array([[yawrate], [ax], [ay]])
        ud = u + np.random.multivariate_normal(mean=[0.0, 0.0, 0.0], cov=self.QV2, size=1).T
        return u, ud

    def predictV1(self):
        
        # define the control input noisefree and noisy
        u_local, ud_local = self.calc_input()   
             
        # define noise zero mean
        noise =np.array([[0.0],[0.0],[0.0]])        
        
        x_est_temp = self.f.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
    
        for i in range(5):
            self.x_est[i] = x_est_temp[i]
    
        F=self.JacobianState.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        self.P = F @ self.P @ F.T + self.QV1
                
    def predictV2(self):
        u_local, ud_local = self.calc_input()
        
        # define noise zero mean
        noise =np.array([[0.0],[0.0],[0.0]]) 
        
        x_est_temp = self.f.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        
        for i in range(5):
            self.x_est[i] = x_est_temp[i]
        
        F=self.JacobianState.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        L=self.JacobianNoise.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        self.P = F @ self.P @ F.T + L @ self.QV2 @ L.T


    def predictV3(self):
        u_local, ud_local = self.calc_input()
        
        # define noise zero mean
        noise =np.array([[0.0],[0.0],[0.0]])
        
        x_est_temp = self.f.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        
        for i in range(5):
            self.x_est[i] = x_est_temp[i]
        
        F=self.JacobianState.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        L=self.JacobianNoise.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        self.P = F @ self.P @ F.T + L @ self.QV2 @ L.T + self.QV1
    
    def predictV4(self):
        u_local, ud_local = self.calc_input()
        
        # define noise zero mean
        noise =np.array([[0.0],[0.0],[0.0]])
        
        x_est_temp = self.f.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        
        for i in range(5):
            self.x_est[i] = x_est_temp[i]
        
        F=self.JacobianState.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        L=self.JacobianNoise.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        B=self.JacobianInput.subs(self.x,Matrix(self.x_est)).subs(self.u,Matrix(ud_local)).subs(self.n,Matrix(noise))
        self.P = F @ self.P @ F.T + L @ self.QV2 @ L.T + B @ self.QV2 @ B.T + self.QV1
    
    
    def update(self, z):
        z_pred = self.h.subs(self.x, Matrix(self.x_est))

        H = self.JacobianObservation.subs(self.x, Matrix(self.x_est))

        y = z - z_pred
        S = H @ self.P @ H.T + self.R
        S_numeric = np.array(S).astype(np.float64)

        # Regularization to handle singularity
        epsilon = 1e-6
        S_regularized = S_numeric + epsilon * np.eye(S_numeric.shape[0])

        K = self.P @ H.T @ np.linalg.pinv(S_regularized)

        self.x_est = self.x_est + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

        return K


def main(): 
    
    dt = 0.05
    num_steps = 200
    num_landmarks = 1
    
    landmarks_pose = np.random.normal(loc=0, scale=20, size=(num_landmarks, 3))
    
    # Initialize the conrol signal noise covariance matrix
    QV1=np.diag([0.01, 0.01, 0.01, 0.01, 0.01])
    QV2=np.diag([0.01, 0.01, 0.01])
    
    # Initialize the observation noise covariance matrix
    variance_relative_x=0.01
    variance_relative_y=0.01
    R=np.diag([variance_relative_x,variance_relative_y] * num_landmarks)
    sensor_range = 80.0
    
    ekfv1 = ExtendedKalmanFilter(dt, QV1,QV2, R, num_landmarks, landmarks_pose, sensor_range)
    ekfv2 = ExtendedKalmanFilter(dt, QV1,QV2, R, num_landmarks, landmarks_pose, sensor_range)
    ekfv3 = ExtendedKalmanFilter(dt, QV1,QV2, R, num_landmarks, landmarks_pose, sensor_range)
    ekfv4 = ExtendedKalmanFilter(dt, QV1,QV2, R, num_landmarks, landmarks_pose, sensor_range)
    
    x_true = np.array([[0], [0], [0], [0], [0]])
    true_states = []
    measurements = []

    estimated_statesv1 = []
    estimated_statesv2 = []
    estimated_statesv3 = []
    estimated_statesv4 = []
    
    estimated_statesv1k=[]
    estimated_statesv2k=[]
    estimated_statesv3k=[]
    estimated_statesv4k=[]
    
    for step in range(num_steps):
        
        u, ud = ekfv1.calc_input()
        
        x_true = ekfv1.f.subs(ekfv1.x, Matrix(x_true)).subs(ekfv1.u, Matrix(ud)).subs(ekfv1.n, Matrix([[0.0], [0.0], [0.0]])) + np.random.multivariate_normal(mean=[0.0, 0.0, 0.0, 0.0, 0.0], cov=QV1, size=1).T
        
        z_true = ekfv1.h.subs(ekfv1.x, Matrix(x_true)) + (np.random.multivariate_normal([0, 0], R , size=1)).T

        ekfv1.predictV1()
        ekfv2.predictV2()
        ekfv3.predictV3()
        ekfv4.predictV4()
        

        Kv1=ekfv1.update(z_true)
        Kv2=ekfv2.update(z_true)
        Kv3=ekfv3.update(z_true)
        Kv4=ekfv4.update(z_true)
        
    
        true_states.append(np.array(x_true).astype(float)) 
        
        estimated_statesv1.append(np.array(ekfv1.x_est).astype(float))
        estimated_statesv2.append(np.array(ekfv2.x_est).astype(float))
        estimated_statesv3.append(np.array(ekfv3.x_est).astype(float))
        estimated_statesv4.append(np.array(ekfv4.x_est).astype(float))
        
        estimated_statesv1k.append(np.array(Kv1).astype(float))
        estimated_statesv2k.append(np.array(Kv2).astype(float))
        estimated_statesv3k.append(np.array(Kv3).astype(float))
        estimated_statesv4k.append(np.array(Kv4).astype(float))
               
    true_states = np.array(true_states)
    estimated_statesv1 = np.array(estimated_statesv1)
    estimated_statesv2 = np.array(estimated_statesv2)
    estimated_statesv3 = np.array(estimated_statesv3)
    estimated_statesv4 = np.array(estimated_statesv4)
    
    labels = ["X Position", "Y Position", "Theta", "X Velocity", "Y Velocity"]

    estimated_states_list = [estimated_statesv1, estimated_statesv2, estimated_statesv3, estimated_statesv4]
    estimated_labels = ["Estimated State V1", "Estimated State V2", "Estimated State V3", "Estimated State V4"]

    for idx, estimated_states in enumerate(estimated_states_list):
        fig, axs = plt.subplots(5, 1, figsize=(19.2, 10.8)) 

        for i in range(5):
            axs[i].plot(range(num_steps), true_states[:, i], label=f'True {labels[i]}', linestyle='-', marker='o')
            axs[i].plot(range(num_steps), estimated_states[:, i], label=f'{estimated_labels[idx]} {labels[i]}', linestyle='--', marker='x')
            axs[i].set_ylabel(labels[i])
            axs[i].set_title(f"{labels[i]} Over Time ({estimated_labels[idx]})")
            axs[i].legend()
            axs[i].grid(True)

        axs[4].set_xlabel("Time Step")

        plt.tight_layout()

        filename = f"state_estimation_{idx+1}.jpg"
        plt.savefig(filename, format='jpeg', dpi=300)

        # plt.show()

    
    # ---- PLOT ESTIMATED STATES ----
    plt.figure(figsize=(19.2, 10.8), dpi=100)

    plt.plot(true_states[:, 0], true_states[:, 1], label="True State", linestyle='-', marker='o', color='blue', alpha=0.7)
    plt.plot(estimated_statesv1[:, 0], estimated_statesv1[:, 1], label="Estimated State V1", linestyle='--', marker='x', color='red', alpha=0.7)
    plt.plot(estimated_statesv2[:, 0], estimated_statesv2[:, 1], label="Estimated State V2", linestyle='--', marker='s', color='green', alpha=0.7)
    plt.plot(estimated_statesv3[:, 0], estimated_statesv3[:, 1], label="Estimated State V3", linestyle='--', marker='d', color='purple', alpha=0.7)
    plt.plot(estimated_statesv4[:, 0], estimated_statesv4[:, 1], label="Estimated State V4", linestyle='--', marker='x', color='black', alpha=0.7)

    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("True State vs Estimated States (V1, V2, V3, V4)")
    plt.legend()
    plt.grid()
    plt.savefig("true_vs_estimated_states.jpg", format="jpeg", dpi=100)  
    plt.close() 

    # ---- COMPUTE RMSE ----
    rmsev1 = np.sqrt(np.mean((true_states[:, :2] - estimated_statesv1[:, :2])**2, axis=1))
    rmsev2 = np.sqrt(np.mean((true_states[:, :2] - estimated_statesv2[:, :2])**2, axis=1))
    rmsev3 = np.sqrt(np.mean((true_states[:, :2] - estimated_statesv3[:, :2])**2, axis=1))
    rmsev4 = np.sqrt(np.mean((true_states[:, :2] - estimated_statesv4[:, :2])**2, axis=1))

    # Compute sum of RMSE for legend
    sum_rmsev1 = np.sum(rmsev1)
    sum_rmsev2 = np.sum(rmsev2)
    sum_rmsev3 = np.sum(rmsev3)
    sum_rmsev4 = np.sum(rmsev4)

    # ---- PLOT RMSE ----
    plt.figure(figsize=(19.2, 10.8), dpi=100)

    plt.plot(range(len(rmsev1)), rmsev1, linestyle='-', color='red', label=f"RMSE V1 (Sum: {sum_rmsev1:.2f})")
    plt.plot(range(len(rmsev2)), rmsev2, linestyle='--', color='green', label=f"RMSE V2 (Sum: {sum_rmsev2:.2f})")
    plt.plot(range(len(rmsev3)), rmsev3, linestyle='-.', color='purple', label=f"RMSE V3 (Sum: {sum_rmsev3:.2f})")
    plt.plot(range(len(rmsev4)), rmsev4, linestyle='-.', color='black', label=f"RMSE V4 (Sum: {sum_rmsev4:.2f})")

    plt.xlabel("Time Step")
    plt.ylabel("RMSE")
    plt.title("Root Mean Squared Error (RMSE) Over Time")
    plt.legend()
    plt.grid()
    plt.savefig("rmse_over_time.jpg", format="jpeg", dpi=100) 
    plt.close() 

    # Plot Estimated States v1, v2, v3, v4 (first two elements) in 4x1 subplots
    fig, axs = plt.subplots(4, 1, figsize=(19.2, 10.8), dpi=100)

    axs[0].plot(true_states[:, 0], true_states[:, 1], label="True State", linestyle='-', marker='o', color='blue')
    axs[0].plot(estimated_statesv1[:, 0], estimated_statesv1[:, 1], label="Estimated State v1", linestyle='--', marker='x', color='red')
    axs[0].set_xlabel("X Position")
    axs[0].set_ylabel("Y Position")
    axs[0].set_title("Estimated State v1 vs True State")
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(true_states[:, 0], true_states[:, 1], label="True State", linestyle='-', marker='o', color='blue')
    axs[1].plot(estimated_statesv2[:, 0], estimated_statesv2[:, 1], label="Estimated State v2", linestyle='--', marker='x', color='green')
    axs[1].set_xlabel("X Position")
    axs[1].set_ylabel("Y Position")
    axs[1].set_title("Estimated State v2 vs True State")
    axs[1].legend()
    axs[1].grid()

    axs[2].plot(true_states[:, 0], true_states[:, 1], label="True State", linestyle='-', marker='o', color='blue')
    axs[2].plot(estimated_statesv3[:, 0], estimated_statesv3[:, 1], label="Estimated State v3", linestyle='--', marker='x', color='purple')
    axs[2].set_xlabel("X Position")
    axs[2].set_ylabel("Y Position")
    axs[2].set_title("Estimated State v3 vs True State")
    axs[2].legend()
    axs[2].grid()

    axs[3].plot(true_states[:, 0], true_states[:, 1], label="True State", linestyle='-', marker='o', color='blue')
    axs[3].plot(estimated_statesv4[:, 0], estimated_statesv4[:, 1], label="Estimated State v4", linestyle='--', marker='x', color='purple')
    axs[3].set_xlabel("X Position")
    axs[3].set_ylabel("Y Position")
    axs[3].set_title("Estimated State v4 vs True State")
    axs[3].legend()
    axs[3].grid()

    plt.tight_layout()
    plt.savefig("estimated_states_subplots.jpg", format="jpeg", dpi=100)  
    plt.close()  
    
    # Plot RMSE for v1, v2, v3, v4 in 4x1 subplots
    fig, axs = plt.subplots(4, 1, figsize=(19.2, 10.8), dpi=100)

    axs[0].plot(range(len(rmsev1)), rmsev1, linestyle='-', color='red')
    axs[0].set_xlabel("Time Step")
    axs[0].set_ylabel("RMSE")
    axs[0].set_title("RMSE for v1 Over Time")
    axs[0].legend([f"RMSE v1 (Sum: {sum_rmsev1:.2f})"])
    axs[0].grid()

    axs[1].plot(range(len(rmsev2)), rmsev2, linestyle='-', color='green')
    axs[1].set_xlabel("Time Step")
    axs[1].set_ylabel("RMSE")
    axs[1].set_title("RMSE for v2 Over Time")
    axs[1].legend([f"RMSE v2 (Sum: {sum_rmsev2:.2f})"])
    axs[1].grid()

    axs[2].plot(range(len(rmsev3)), rmsev3, linestyle='-', color='purple')
    axs[2].set_xlabel("Time Step")
    axs[2].set_ylabel("RMSE")
    axs[2].set_title("RMSE for v3 Over Time")
    axs[2].legend([f"RMSE v3 (Sum: {sum_rmsev3:.2f})"])
    axs[2].grid()

    axs[3].plot(range(len(rmsev4)), rmsev4, linestyle='-', color='black')
    axs[3].set_xlabel("Time Step")
    axs[3].set_ylabel("RMSE")
    axs[3].set_title("RMSE for v4 Over Time")
    axs[3].legend([f"RMSE v4 (Sum: {sum_rmsev4:.2f})"])
    axs[3].grid()

    plt.tight_layout()
    plt.savefig("rmse_subplots.jpg", format="jpeg", dpi=100)
    plt.close()
    

    
    
    
if __name__ == '__main__':
    main()