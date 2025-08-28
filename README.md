# Kalman Filter Simulation (1D/2D) – C++ & Python

Small, self-contained demos of Kalman Filtering for robotics localization.

## ✨ Goals
- Build intuition for KF/EKF with minimal code.
- Show predict–update math with clean plots.
- Prepare building blocks for full autonomy stacks.

## 📐 Models
Discrete linear system:
- State: x_{k+1} = A x_k + B u_k + w_k
- Measurement: z_k = H x_k + v_k

Where:
- x ∈ ℝ^n (e.g., [position, velocity]^T)
- u ∈ ℝ^m (e.g., acceleration)
- z ∈ ℝ^p (sensor measurement)
- w ~ N(0, Q), v ~ N(0, R)

## 📘 Roadmap
- [x] Day 1: Repo + notes
- [ ] Day 2: 1D KF in Python (position/velocity with noisy position)
- [ ] Day 3: Port 1D KF to C++ (Eigen) + plotting via CSV + Python
- [ ] Day 4: 2D constant-velocity model
- [ ] Day 5: EKF sketch (nonlinear heading)
- [ ] Day 6: README polish + GIFs

## 🛠 Tech
- C++17, Eigen
- Python 3, numpy, matplotlib

## 📂 Structure

