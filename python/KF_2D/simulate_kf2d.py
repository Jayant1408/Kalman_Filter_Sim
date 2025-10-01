import os
import numpy as np
import matplotlib.pyplot as plt
import sys

# add current directory to sys.path so Python can find kf2d.py
sys.path.append(os.path.dirname(__file__))

import kf2d  # import the module, not the class

def ensure_dir(p):
    if not os.path.exists(p):
        os.makedirs(p, exist_ok=True)

def simulate():
    np.random.seed(42)
    dt = 0.1
    T = 20.0
    steps = int(T / dt)

    # true initial state: starting at origin vx = vy = 1 m/s
    x_true = np.zeros((4, 1))
    x_true[2, 0] = 1.0
    x_true[3, 0] = 1.0

    # measurement noise std dev
    meas_std = 0.2

    kf = kf2d.KalmanFilter2D(
        dt=dt,
        process_var_pos=1e-4,
        process_var_vel=1e-4,
        meas_var_pos=meas_std**2,
    )

    kf.set_state(
        np.array([0.2, -0.1, 0.8, 1.2]),
        P0=np.diag([0.5, 0.5, 0.5, 0.5]),
    )

    xs_true, ys_true = [], []
    xs_meas, ys_meas = [], []
    xs_filt, ys_filt = [], []
    xs_time = []

    for k in range(steps):
        t = k * dt

        # True motion: x += vx*dt, y += vy*dt
        x_true = kf.F @ x_true  # reuse KF's F for true propagation (CV)

        # noisy measurement of x,y
        z = np.array([
            x_true[0, 0] + np.random.randn() * meas_std,
            x_true[1, 0] + np.random.randn() * meas_std,
        ])

        # KF cycle
        kf.predict()
        kf.update(z)

        # logs
        xs_true.append(x_true[0, 0]); ys_true.append(x_true[1, 0])
        xs_meas.append(z[0]);         ys_meas.append(z[1])
        xs_filt.append(kf.x[0, 0]);   ys_filt.append(kf.x[1, 0])
        xs_time.append(t)

    # plots
    outdir = "data"
    ensure_dir(outdir)

    # Trajectory XY
    plt.figure()
    plt.plot(xs_true, ys_true, label="True")
    plt.scatter(xs_meas, ys_meas, s=8, alpha=0.5, label="Measured")
    plt.plot(xs_filt, ys_filt, linestyle="--", label="KF")
    plt.xlabel("x"); plt.ylabel("y"); plt.legend(); plt.title("2D KF Trajectory")
    plt.savefig(os.path.join(outdir, "kf_2d_xy.png"), dpi=150, bbox_inches="tight")
    plt.close()

    # x vs time
    plt.figure()
    plt.plot(xs_time, xs_true, label="True x")
    plt.scatter(xs_time, xs_meas, s=8, alpha=0.5, label="Measured x")
    plt.plot(xs_time, xs_filt, linestyle="--", label="KF x")
    plt.xlabel("time [s]"); plt.ylabel("x"); plt.legend(); plt.title("2D KF: x vs time")
    plt.savefig(os.path.join(outdir, "kf_2d_x.png"), dpi=150, bbox_inches="tight")
    plt.close()

    print("Saved: data/kf_2d_xy.png, data/kf_2d_x.png")

if __name__ == "__main__":
    simulate()
