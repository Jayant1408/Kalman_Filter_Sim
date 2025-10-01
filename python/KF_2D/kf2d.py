import numpy as np

class KalmanFilter2D:
    """
    Discrete-time CV (constant velocity) KF.
    State x = [x, y, vx, vy]^T
    Measurement z = [x, y]^T
    """

    def __init__(self, dt: float,
                 process_var_pos: float = 1e-3,
                 process_var_vel: float = 1e-3,
                 meas_var_pos: float = 1e-2):
        self.dt = dt

        # State transition
        self.F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Measurement model: we observe x and y
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        # Process noise covariance Q (pos and vel blocks)
        q_pos = process_var_pos
        q_vel = process_var_vel
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])

        # Measurement noise covariance R
        r_pos = meas_var_pos
        self.R = np.diag([r_pos, r_pos])

        # Initialize state and covariance
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 1.0

        # For logging
        self._I = np.eye(4)

    def set_state(self, x0: np.ndarray, P0: np.ndarray | None = None):
        self.x = x0.reshape(4, 1).astype(float)
        if P0 is not None:
            self.P = P0.astype(float)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x, self.P

    def update(self, z: np.ndarray):
        z = z.reshape(2, 1).astype(float)
        y = z - (self.H @ self.x)                  # innovation
        S = self.H @ self.P @ self.H.T + self.R    # innovation cov
        K = self.P @ self.H.T @ np.linalg.inv(S)   # Kalman gain
        self.x = self.x + K @ y
        self.P = (self._I - K @ self.H) @ self.P
        return self.x, self.P
