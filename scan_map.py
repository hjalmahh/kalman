import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation
import math


folder = 'data/csv_export_slam_etc/scan.csv'
data = {}
poses_df = pd.read_csv('kf_output.csv')
poses_df['theta'] = np.arctan2(poses_df['est_vel_y'], poses_df['est_vel_x'])



df = pd.read_csv(folder)
data['scan.csv'] = df

for name, df in data.items():
    print(f"\n{name} — Always Zero Columns:")
    always_zero = df.loc[:, df.nunique(dropna=False) == 1]
    for col in always_zero.columns:
        if (always_zero[col] == 0).all():
            print(f"  {col} is always 0")

for name, df in data.items():
    print(f"\n{name} — Constant Columns:")
    for col in df.columns:
        if df[col].nunique(dropna=False) == 1:
            print(f"  {col} = {df[col].iloc[0]}")

for name, df in data.items():
    print(f"\n{name} — Low-Change Columns (std < 1e-2 and >1 unique value):")
    for col in df.select_dtypes(include='number').columns:
        std = df[col].std()
        uniq = df[col].nunique()
        if 1 < uniq and std < 1e-4:
            print(f"  {col} (std = {std:.2e}, unique vals = {uniq})")

#------------------------------------------------
merged = pd.merge_asof(df.sort_values('timestamp'),
                       poses_df.sort_values('timestamp'),
                       on='timestamp',
                       direction='nearest')





all_x = []
all_y = []

for _, row in merged.iterrows():
    ranges = np.array(eval(row['.ranges'], {'inf': math.inf, '__builtins__': None}))
    angle_min = row['.angle_min']
    angle_inc = row['.angle_increment']
    angles = angle_min + np.arange(len(ranges)) * angle_inc

    # Local points
    x_local = ranges * np.cos(angles)
    y_local = ranges * np.sin(angles)

    # Use robot pose
    x_robot = row['est_pos_x']
    y_robot = row['est_pos_y']

    # Calculate theta from velocities (if no theta provided)
    if 'theta' in row:
        theta = row['theta']
    else:
        theta = np.arctan2(row['est_vel_y'], row['est_vel_x'])

    # Rotate + translate to global frame
    x_global = x_robot + x_local * np.cos(theta) - y_local * np.sin(theta)
    y_global = y_robot + x_local * np.sin(theta) + y_local * np.cos(theta)

    all_x.extend(x_global)
    all_y.extend(y_global)

# Plot all points
plt.figure(figsize=(8, 8))
plt.scatter(all_x, all_y, s=1)
plt.axis('equal')
plt.title('LiDAR Map (scatter-based)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()