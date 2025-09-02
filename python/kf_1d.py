import numpy as np
import matplotlib.pyplot as plt
import os

#---- Model (discrete, const dt) ------
# State: x = [position, veclocity] ^ T
# x_{k+1} = A x_k + B u_k + w,   z_k = H x_k + v
# A = [[1, dt],
#      [0,  1 ]]
# B = [[0.5*dt^2],
#      [     dt ]]
# H = [1, 0]  (we measure position only)

def simulate_and_filter(T = 10.0, dt = 0.1, accel = 0.5, pos_sigma = 2.0, q_var = 1e-2, r_var = 4.0):
    # Set seed for reproducibility
    np.random.seed(0)

    # Calculate number of steps
    steps = int(T/dt)
    
    # Create time vector
    t = np.arange(steps) * dt

    # Define state transition matrix
    A = np.array([[1.0, dt], 
                  [0.0, 1.0]])
    
    # Define control matrix
    B = np.array([[0.5 * dt * dt], [dt]])

    # Define measurement matrix
    H = np.array([[1.0,0.0]])

    # Process + measurement noise covariance
    Q = q_var * np.array([[dt**4/4, dt**3/2],
                          [dt**3/2, dt**2]])
    
    # Define measurement noise covariance
    R = np.array([[r_var]])

    # Ground Truth Simulation
    # Initialize arrays for ground truth and measurements
    x_true = np.zeros((2, steps))
    z_meas = np.zeros(steps)

    # Initialize state vector
    x = np.array([[0.0],  # pos
                  [0.0]]) # val
    
    # Simulate ground truth
    for k in range(steps):
        
        # True dynamics without process noise (for clean "truth")
        x = A @ x + B * accel
        x_true[:, k] = x.flatten()

        # Add measurement noise
        z_meas[k] = x[0,0] + np.random.normal(0, pos_sigma)

    # Initialize arrays for Kalman filter
    ## --- Kalman Filter ---
    x_hat = np.zeros((2,steps))
    P_hist = np.zeros((2,2,steps))

    # Initialize state estimate and covariance
    x_est = np.array([[0.0],
                      [0.0]])
    
    # Initialize covariance matrix
    P = np.eye(2) * 10.0

    # Define identity matrix
    I = np.eye(2)

    # Kalman Filter Loop
    for k in range(steps):

        # Predict
        # Apply control input (acceleration)
        u = accel
        x_pred = A @ x_est + B * u
        P_pred = A @ P @ A.T + Q

        # Update
        # Compute measurement residual
        z = np.array([[z_meas[k]]])
        # Compute innovation
        y_update = z - H @ x_pred       
        # Compute innovation covariance
        S = H @ P_pred @ H.T + R
        # Compute Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        
        # Update state estimate and covariance
        x_est = x_pred + K @ y_update
        # Update covariance
        P = (I - K @ H) @ P_pred

        # Store history
        P_hist[:,:,k] = P
        # Store state estimate
        x_hat[:,k] = x_est.flatten()

    return t, x_true, z_meas, x_hat

def main():
    t, x_true, z_meas, x_hat = simulate_and_filter(
        T = 12.0, dt = 0.1, accel = 0.5,
        pos_sigma=2.0, q_var = 1e-3, r_var = 4.0
    )

    #Plot 
    plt.figure(figsize = (10,5))
    plt.plot(t, x_true[0], label = 'True position')
    plt.plot(t, z_meas, '.', alpha = 0.5, label = 'Measured position (noisy)')
    plt.plot(t, x_hat[0], label = 'KF estimated position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position')
    plt.legend()
    plt.title('1D Kalman Filter (pos/vel, position-only measurement)')
    plt.tight_layout()

    os.makedirs('../data', exist_ok=True)
    out_png = '../data/kf_1d_position.png'
    plt.savefig(out_png, dpi=150)
    print(f"Saved plot to {out_png}")

if __name__ == "__main__":
    main()

        




