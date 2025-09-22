# State-Space Model & Kalman Filter Notes

## 1. System Model

A general **discrete-time linear dynamical system** is written as:

- **State transition:**
  \[
  x_{k+1} = A x_k + B u_k + w_k
  \]

- **Measurement model:**
  \[
  z_k = H x_k + v_k
  \]

Where:
- \(x_k\): State vector at time step k (e.g., position, velocity, heading).  
- \(u_k\): Control input (acceleration, steering angle, …).  
- \(z_k\): Measurement (sensor reading, e.g., GPS position).  
- \(A\): State transition matrix (describes system dynamics).  
- \(B\): Control input matrix.  
- \(H\): Observation/measurement matrix.  
- \(w_k \sim \mathcal{N}(0, Q)\): Process noise.  
- \(v_k \sim \mathcal{N}(0, R)\): Measurement noise.  

---

## 2. Kalman Filter Algorithm

The KF is an **optimal recursive estimator** for linear Gaussian systems.  
It alternates between two steps:

### Prediction
- Predict state:
  \[
  \hat{x}_{k}^- = A \hat{x}_{k-1}^+ + B u_{k-1}
  \]
- Predict covariance:
  \[
  P_k^- = A P_{k-1}^+ A^T + Q
  \]



### Update
- Innovation (residual):
  \[
  y_k = z_k - H \hat{x}_k^-
  \]
- Innovation covariance:
  \[
  S_k = H P_k^- H^T + R
  \]
- Kalman gain:
  \[
  K_k = P_k^- H^T S_k^{-1}
  \]
- Update state estimate:
  \[
  \hat{x}_k^+ = \hat{x}_k^- + K_k y_k
  \]
- Update covariance:
  \[
  P_k^+ = (I - K_k H) P_k^-
  \]

Where:
- \( \hat{x}_k^- \): Predicted state estimate  
- \( \hat{x}_k^+ \): Updated (posterior) state estimate  
- \(P_k\): Error covariance matrix  


### Example (1D Constant Velocity Model)

Let the state be:
x = [position, velocity]^T

A = [[1, dt],
     [0, 1 ]],
B = [[0.5*dt^2],
     [dt]],
H = [1, 0]

This models:
- Position update with velocity and acceleration.
- Measurement of position only.


---




## 3. Why This Matters in Robotics

- **Localization**: estimating a robot’s position/velocity from noisy GPS/IMU.  
- **Tracking**: following objects in perception (cars, pedestrians).  
- **SLAM**: building maps while localizing.  
- **Sensor Fusion**: combining multiple noisy sensors (e.g., LiDAR + IMU).  

KF is the foundation → extended to **EKF** for nonlinear models, and **PF** for non-Gaussian or multimodal cases.  

---

## 4. Tuning Parameters

- \(Q\) (process noise covariance):  
  - High Q → trust sensors more, filter reacts faster.  
  - Low Q → trust model more, smoother estimate.

- \(R\) (measurement noise covariance):  
  - High R → assume sensors are noisy, rely more on prediction.  
  - Low R → trust sensors more, filter hugs measurements.

---

## 5. Roadmap for This Repo

- Implement KF in 1D and 2D (Python, C++).  
- Extend to EKF (nonlinear bicycle model).  
- Add Particle Filter (PF) for robustness.  
- Compare KF vs EKF vs PF → benchmark error, plots.  
