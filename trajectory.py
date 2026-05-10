import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
import csv

frames_folder = "frames"

frame_files = sorted([
    f for f in os.listdir(frames_folder)
    if f.endswith(".jpg")
])

orb = cv2.ORB_create()

x_positions = [0]
y_positions = [0]

current_x = 0
current_y = 0

trajectory_data = []

for i in range(len(frame_files) - 1):

    img1 = cv2.imread(os.path.join(frames_folder, frame_files[i]), 0)
    img2 = cv2.imread(os.path.join(frames_folder, frame_files[i + 1]), 0)

    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    if des1 is None or des2 is None:
        continue

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    matches = bf.match(des1, des2)

    matches = sorted(matches, key=lambda x: x.distance)

    matches = matches[:50]

    dx_list = []
    dy_list = []

    for match in matches:

        pt1 = kp1[match.queryIdx].pt
        pt2 = kp2[match.trainIdx].pt

        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]

        dx_list.append(dx)
        dy_list.append(dy)

    median_dx = np.median(dx_list)
    median_dy = np.median(dy_list)

    current_x -= median_dx
    current_y -= median_dy

    x_positions.append(current_x)
    y_positions.append(current_y)

    trajectory_data.append({
        "frame": i,
        "x": current_x,
        "y": current_y
    })

    print(
        f"Frame {i} -> {i+1}: "
        f"dx={median_dx:.2f}, "
        f"dy={median_dy:.2f}, "
        f"position=({current_x:.2f}, {current_y:.2f})"
    )

# -----------------------------
# Save trajectory CSV
# -----------------------------

os.makedirs("results", exist_ok=True)

with open("results/orb_trajectory.csv", "w", newline="") as csvfile:

    fieldnames = ["frame", "x", "y"]

    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()

    for row in trajectory_data:
        writer.writerow(row)

print("\nSaved: results/orb_trajectory.csv")

# -----------------------------
# Plot trajectory
# -----------------------------

plt.figure(figsize=(8, 8))

plt.plot(x_positions, y_positions, marker='o')

plt.title("Estimated Drone Trajectory from ORB Feature Matching")

plt.xlabel("Estimated X movement")
plt.ylabel("Estimated Y movement")

plt.grid(True)

plt.savefig("results/estimated_trajectory.png")

plt.show()