import pandas as pd
import matplotlib.pyplot as plt

# Load CSV from C++ output
df = pd.read_csv("../data/kf_1d_cpp.csv")

plt.figure(figsize=(10,5))
plt.plot(df["t"], df["true_pos"], label="True Position")
plt.plot(df["meas_pos"], ".", alpha=0.5, label="Measured Position")
plt.plot(df["est_pos"], label="KF Estimate")
plt.xlabel("Time [s]")
plt.ylabel("Position")
plt.title("1D KF (C++ Eigen results)")
plt.legend()
plt.tight_layout()

out_path = "../data/kf_1d_cpp_plot.png"
plt.savefig(out_path, dpi=150)
print(f"Saved {out_path}")
