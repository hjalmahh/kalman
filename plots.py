import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load Kalman filter output CSV
df = pd.read_csv("kf_output.csv")
odom_raw = pd.read_csv("data/kjor/odom_raw.csv")
odom_raw["datetime"] = pd.to_datetime(df["timestamp"], unit="s")
imu = pd.read_csv("data/kjor/imu.csv")
imu["datetime"] = pd.to_datetime(df["timestamp"], unit="s")
acc_x = imu[".linear_acceleration.x"].values
acc_y = imu[".linear_acceleration.y"].values

#IMU drift
timestamps = imu["timestamp"].values
dt = np.diff(timestamps, prepend=timestamps[0])
bias_x = np.mean(acc_x[:100])
bias_y = np.mean(acc_y[:100])
acc_x -= bias_x
acc_y -= bias_y
vel_x = np.cumsum(acc_x * dt)
vel_y = np.cumsum(acc_y * dt)
pos_x = np.cumsum(vel_x * dt)
pos_y = np.cumsum(vel_y * dt)



# Convert epoch timestamp to pandas datetime
df["datetime"] = pd.to_datetime(df["timestamp"], unit="s")

# ---- Optional: 2D trajectory plot (X vs Y) ----
plt.figure(figsize=(8, 6))
plt.plot(df["est_pos_x"], df["est_pos_y"], label="Estimated 2D Trajectory")
plt.plot(odom_raw['.pose.pose.position.x'], odom_raw['.pose.pose.position.y'],linestyle="dashed", label="Odometry")
#plt.plot(pos_x, pos_y, linestyle='dotted', label='IMU Drift Trajectory')
plt.xlabel("Position X (m)")
plt.ylabel("Position Y (m)")
plt.title("2D Trajectory (X vs Y)")
plt.grid(True)
plt.legend()
plt.axis("equal")

# ---- Position X & Y over time ----
plt.figure(figsize=(10, 6))
plt.plot(df["datetime"], df["est_pos_x"], color="blue", label="Position X (Kalman)")
plt.plot(df["datetime"], df["est_pos_y"], color="orange", label="Position Y (Kalman)")
plt.plot(odom_raw["datetime"], odom_raw[".pose.pose.position.x"], color="blue", linestyle="dashed", label="Position X (Odometry)")
plt.plot(odom_raw["datetime"], odom_raw[".pose.pose.position.y"], color="orange", linestyle="dashed", label="Position Y (Odometry)")
plt.xlabel("Time")
plt.ylabel("Position (m)")
plt.title("Position Over Time")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)

# ---- Velocity X & Y over time ----
plt.figure(figsize=(10, 6))
plt.plot(df["datetime"], df["est_vel_x"], label="Velocity X")
plt.plot(df["datetime"], df["est_vel_y"], label="Velocity Y")
plt.xlabel("Time")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Over Time")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)

# ---- Acceleration X & Y over time ----
plt.figure(figsize=(10, 6))
plt.plot(df["datetime"], df["est_acc_x"], label="Acceleration X")
plt.plot(df["datetime"], df["est_acc_y"], label="Acceleration Y")
plt.xlabel("Time")
plt.ylabel("Acceleration (m/s²)")
plt.title("Acceleration Over Time")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)

plt.tight_layout()
plt.show()
