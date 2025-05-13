import numpy as np
import matplotlib.pyplot as plt
import csv

def polar_to_cartesian(ranges, angle_min, angle_increment):
    points = []
    for i, r in enumerate(ranges):
        if r == 0 or np.isnan(r) or np.isinf(r):
            continue
        angle = angle_min + i * angle_increment
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        points.append((x, y))
    return points

def transform_points(points, px, py, theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return [(px + x * cos_t - y * sin_t, py + x * sin_t + y * cos_t) for x, y in points]

# Example data classes
class Scan:
    def __init__(self, ranges, angle_min, angle_increment):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta  # yaw in radians

# Example: load from CSVs (pseudo)
scan_data = []  # List of Scan objects
odom_data = []  # List of Pose objects

# TODO: Implement CSV reading here
# Each scan must align with a pose (same timestamp or index)

# Plotting
for scan, pose in zip(scan_data, odom_data):
    points_local = polar_to_cartesian(scan.ranges, scan.angle_min, scan.angle_increment)
    points_global = transform_points(points_local, pose.x, pose.y, pose.theta)
    if points_global:  # avoid error on empty scans
        xs, ys = zip(*points_global)
        plt.scatter(xs, ys, s=1, color='black')

plt.axis('equal')
plt.title('Offline LiDAR SLAM Map')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True)
plt.show()
