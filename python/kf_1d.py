import numpy as np
import matplotlib.pyplot as plt
import os

def run_kf(T=12.0, dt=0.1, accel=0.5, pos_sigma=2.0, q_var=1e-3, r_var=4.0):
    steps = int(T / dt)
    t = np.arange(steps) * dt   # FIXED

    # State: x = [position, velocity]^T
    A = np.array([[1.0, dt],
                  [0.0, 1.0]])
    B = np.array([[0.5 * dt * dt],
                  [dt]])
    H = np.array([[1.0, 0.0]])     # measure position only

    # Discrete white noise model for constant-velocity
    Q = q_var * np.array([[dt**4/4, dt**3/2],
                          [dt**3/2, dt**2]])
    R = np.array([[r_var]])

    # ----- Simulate ground truth and noisy measurements -----
    x_true = np.zeros((2, steps))
    z_meas = np.zeros(steps)
    x = np.zeros((2, 1))

    rng = np.random.default_rng(0)
    for k in range(steps):
        x = A @ x + B * accel
        x_true[:, k] = x.flatten()
        z_meas[k] = x[0, 0] + rng.normal(0.0, pos_sigma)

    # ----- Kalman filter -----
    x_est = np.zeros((2, 1))     # initial estimate
    P = np.eye(2) * 10.0         # initial covariance
    x_hat = np.zeros((2, steps))
    I = np.eye(2)

    for k in range(steps):
        # Predict
        x_pred = A @ x_est + B * accel
        P_pred = A @ P @ A.T + Q

        # Update (FIXED to invert S, not a scalar)
        z = np.array([[z_meas[k]]])
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_est = x_pred + K @ y
        P = (I - K @ H) @ P_pred

        x_hat[:, k] = x_est.flatten()

    return t, x_true, z_meas, x_hat

if __name__ == "__main__":
    t, x_true, z_meas, x_hat = run_kf()

    plt.figure(figsize=(10, 5))
    plt.plot(t, x_true[0], label="True Position")
    plt.plot(t, z_meas, ".", alpha=0.5, label="Measured Position")
    plt.plot(t, x_hat[0], label="KF Estimate")
    plt.xlabel("Time [s]")
    plt.ylabel("Position")
    plt.title("1D Kalman Filter (pos/vel, position-only measurement)")
    plt.legend()
    plt.tight_layout()

    os.makedirs("../data", exist_ok=True)
    out_png = "../data/kf_1d_position.png"
    plt.savefig(out_png, dpi=150)
    print(f"Saved plot to {out_png}")
