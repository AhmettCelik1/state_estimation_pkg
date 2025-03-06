import numpy as np
import matplotlib.pyplot as plt

class ExtendedKalmanFilter:
    def __init__(self, dt, Q, R, num_landmarks, landmarks, sensor_range):
        self.dt = dt  
        self.Q = Q 
        self.R = R  
        self.num_landmarks = num_landmarks
        self.landmarks = landmarks  
        self.sensor_range = sensor_range
        self.INPUT_NOISE = np.diag([0.01, 0.01, 0.01]) ** 2
        
        # x_est = [x, y, yaw_rate, v_x, v_y]
        self.x_est = np.array([0, 0, 0, 0, 0]) 
        
        # Covariance matrix P
        self.P = np.eye(5)
        
        # State transition matrix F
        self.F = np.array([
            [1, 0, 0, self.dt, 0],
            [0, 1, 0, 0, self.dt],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]])
        
        # Control input matrix B 
        self.B = np.array([
            [0, 0, 0],
            [0, 0, 0],
            [1, 0, 0],
            [0, self.dt, 0],
            [0, 0, self.dt]])

    def calc_input(self):
        ax = 1.0  # [m/s^2]
        ay = 0.1 # [m/s^2]
        yawrate = 0.1  # [rad/s]
        u = np.array([[ax], [ay], [yawrate]])
        ud = u + self.INPUT_NOISE @ np.random.randn(3, 1)  
        return u, ud

    def predict(self):
        u, ud = self.calc_input() 
        control_input = self.B @ ud
        self.x_est = self.F @ self.x_est + control_input.flatten()
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def measurement_model(self, x):
        return np.array([np.linalg.norm(x[:2] - lm) if np.linalg.norm(x[:2] - lm) <= self.sensor_range else np.nan for lm in self.landmarks])
    
    def jacobian_measurement_model(self, x):
        H = np.zeros((self.num_landmarks, 5)) 
        for i in range(self.num_landmarks):
            dx = x[0] - self.landmarks[i, 0]
            dy = x[1] - self.landmarks[i, 1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist > 0 and dist <= self.sensor_range:
                H[i, 0] = dx / dist
                H[i, 1] = dy / dist
            
            H[i, 2] = 0
            H[i, 3] = 0
            H[i, 4] = 0
        return H
    
    def update(self, z):
        H = self.jacobian_measurement_model(self.x_est)
        z_pred = self.measurement_model(self.x_est)
        valid_idx = ~np.isnan(z_pred)
        H = H[valid_idx]
        z_pred = z_pred[valid_idx]
        z = z[valid_idx]
        y = z - z_pred  
        S = H @ self.P @ H.T + self.R[valid_idx][:, valid_idx]
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x_est = self.x_est + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        return K
    

def plot_state(ax, time, state, std, label, color):
    state = np.array(state) 
    std = np.array(std)  
    ax.plot(time, state, color=color, label=label)
    ax.fill_between(time, state - std, state + std, color=color, alpha=0.3)
    ax.grid(True)
    ax.legend()



def main(): 
    dt = 1.0
    num_steps = 150
    num_landmarks = 5
    # landmarks = np.random.uniform(-10, 10, (num_landmarks, 2))
    landmarks = np.random.normal(loc=0, scale=20, size=(num_landmarks, 2))
    Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01])  
    R = np.diag([0.5] * num_landmarks)
    sensor_range = 80.0

    ekf = ExtendedKalmanFilter(dt, Q, R, num_landmarks, landmarks, sensor_range)

    x_true = np.array([0, 0, 0, 0, 0]) 

    true_states = []
    estimated_states = []
    
    ekf_est_x = []
    ekf_est_y = []
    ekf_est_yaw_rate = []
    ekf_est_v_x = []
    ekf_est_v_y = []
    
    ekf_est_x_std = []
    ekf_est_y_std = []
    ekf_est_yaw_rate_std = []
    ekf_est_v_x_std = []
    ekf_est_v_y_std = []
    
    ekf_est_x_kalman_gain = []
    ekf_est_y_kalman_gain = []
    ekf_est_yaw_rate_kalman_gain = []
    ekf_est_v_x_kalman_gain = []
    ekf_est_v_y_kalman_gain = []

    plt.figure(figsize=(10, 6))
    plt.ion()  

    for step in range(num_steps):
        x_true = ekf.F @ x_true + np.random.multivariate_normal([0, 0, 0, 0, 0], Q)

        z = np.array([np.linalg.norm(x_true[:2] - lm) if np.linalg.norm(x_true[:2] - lm) <= sensor_range else np.nan for lm in landmarks]) + np.random.normal(0, 0.5, size=num_landmarks)

        ekf.predict()
        K=ekf.update(z)
        

        true_states.append(x_true.copy())
        estimated_states.append(ekf.x_est.copy())

        true_states_np = np.array(true_states)
        estimated_states_np = np.array(estimated_states)

        plt.clf()  
        plt.plot(true_states_np[:, 0], true_states_np[:, 1], 'g-', label="True Path")
        plt.plot(estimated_states_np[:, 0], estimated_states_np[:, 1], 'b--', label="EKF Estimate")
        plt.scatter(landmarks[:, 0], landmarks[:, 1], c='r', marker='x', label="Landmarks")
        
        z_pred = ekf.measurement_model(ekf.x_est) 
        for i, lm in enumerate(landmarks):
            if not np.isnan(z_pred[i]) and np.linalg.norm(ekf.x_est[:2] - lm) <= sensor_range: 
                plt.plot([estimated_states_np[-1, 0], lm[0]], [estimated_states_np[-1, 1], lm[1]], 'k:', alpha=0.5)

        ekf_est_x.append(ekf.x_est[0])
        ekf_est_y.append(ekf.x_est[1])
        ekf_est_yaw_rate.append(ekf.x_est[2])
        ekf_est_v_x.append(ekf.x_est[3])
        ekf_est_v_y.append(ekf.x_est[4])
        
        ekf_est_x_std.append(np.sqrt(ekf.P[0, 0]))
        ekf_est_y_std.append(np.sqrt(ekf.P[1, 1]))
        ekf_est_yaw_rate_std.append(np.sqrt(ekf.P[2, 2]))
        ekf_est_v_x_std.append(np.sqrt(ekf.P[3, 3]))
        ekf_est_v_y_std.append(np.sqrt(ekf.P[4, 4]))
        
        if K.size > 0:
            ekf_est_x_kalman_gain.append(K[0, 0])
            ekf_est_y_kalman_gain.append(K[1, 0])
            ekf_est_yaw_rate_kalman_gain.append(K[2, 0])
            ekf_est_v_x_kalman_gain.append(K[3, 0])
            ekf_est_v_y_kalman_gain.append(K[4, 0])

        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title(f"True vs. EKF Estimated Trajectory (Step {step+1})")
        plt.legend()
        plt.grid()
        plt.pause(0.1) 

    plt.ioff()  
    plt.show()


    fig, axs = plt.subplots(5, 1, figsize=(19.2, 10.8), dpi=100)
    
    time_axis = np.arange(0, len(ekf_est_x) * dt, dt)
    
    plot_state(axs[0], time_axis, ekf_est_x, ekf_est_x_std, "X Position", "b")
    plot_state(axs[1], time_axis, ekf_est_y, ekf_est_y_std, "Y Position", "g")
    plot_state(axs[2], time_axis, ekf_est_yaw_rate, ekf_est_yaw_rate_std, "Yaw Rate", "r")
    plot_state(axs[3], time_axis, ekf_est_v_x, ekf_est_v_x_std, "Velocity X", "m")
    plot_state(axs[4], time_axis, ekf_est_v_y, ekf_est_v_y_std, "Velocity Y", "c")
    
    axs[0].set_ylabel("X Position (m)")
    axs[1].set_ylabel("Y Position (m)")
    axs[2].set_ylabel("Yaw Rate (rad/s)")
    axs[3].set_ylabel("Velocity X (m/s)")
    axs[4].set_ylabel("Velocity Y (m/s)")
    axs[4].set_xlabel("Time (s)")
    
    plt.tight_layout()
    
    plt.savefig("ekf_states_with_variance.jpg", format="jpeg", dpi=100)
    plt.show()
    
    fig, axs = plt.subplots(5, 1, figsize=(19.2, 10.8), dpi=100)
    
    time_axis = np.arange(0, len(ekf_est_x_kalman_gain) * dt, dt)
    
    num_ekf_est_x_kalman_gain = np.array(ekf_est_x_kalman_gain)
    num_ekf_est_y_kalman_gain = np.array(ekf_est_y_kalman_gain)
    num_ekf_est_yaw_rate_kalman_gain = np.array(ekf_est_yaw_rate_kalman_gain)
    num_ekf_est_v_x_kalman_gain = np.array(ekf_est_v_x_kalman_gain)
    num_ekf_est_v_y_kalman_gain = np.array(ekf_est_v_y_kalman_gain)
    
    axs[0].plot(time_axis, num_ekf_est_x_kalman_gain, label="X Position Kalman Gain", color="b")
    axs[1].plot(time_axis, num_ekf_est_y_kalman_gain, label="Y Position Kalman Gain", color="g")
    axs[2].plot(time_axis, num_ekf_est_yaw_rate_kalman_gain, label="Yaw Rate Kalman Gain", color="r")
    axs[3].plot(time_axis, num_ekf_est_v_x_kalman_gain, label="Velocity X Kalman Gain", color="m")
    axs[4].plot(time_axis, num_ekf_est_v_y_kalman_gain, label="Velocity Y Kalman Gain", color="c")
    
    axs[0].set_ylabel("X Position Kalman Gain")
    axs[1].set_ylabel("Y Position Kalman Gain")
    axs[2].set_ylabel("Yaw Rate Kalman Gain")
    axs[3].set_ylabel("Velocity X Kalman Gain")
    axs[4].set_ylabel("Velocity Y Kalman Gain")
    axs[4].set_xlabel("Time (s)")
    
    plt.tight_layout()
    plt.savefig("ekf_kalman_gains.jpg", format="jpeg", dpi=100)
    plt.show()
    
if __name__ == '__main__':
    main()
