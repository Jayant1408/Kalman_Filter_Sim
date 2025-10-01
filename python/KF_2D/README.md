### Day 4 – 2D Kalman Filter (CV model)

- State: \[x, y, vx, vy\]^T  
- Measurement: \[x, y\] (Gaussian noise σ=0.2)  
- Motion: constant velocity along diagonal (vx=vy=1 m/s)  
- Files:
  - `kf2d.py` – KF implementation (predict + update)
  - `simulate_kf2d.py` – simulation + plotting
- Outputs:
  - `data/kf_2d_xy.png` – true vs noisy vs KF trajectory
  - `data/kf_2d_x.png` – x vs time (true/measurements/KF)
- Run:
  ```bash
  pip install -r requirements.txt
  python simulate_kf2d.py
