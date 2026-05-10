import csv
import matplotlib.pyplot as plt

# -----------------------------
# Load GPS trajectory
# -----------------------------

gps_x = []
gps_y = []

with open("results/telemetry.csv", "r") as file:
    reader = csv.DictReader(file)

    for row in reader:
        gps_x.append(float(row["longitude"]))
        gps_y.append(float(row["latitude"]))

# Normalize GPS values
gps_x0 = gps_x[0]
gps_y0 = gps_y[0]

gps_x = [(x - gps_x0) * 100000 for x in gps_x]
gps_y = [(y - gps_y0) * 100000 for y in gps_y]

# -----------------------------
# Load ORB trajectory
# -----------------------------

orb_x = [0]
orb_y = [0]

with open("results/orb_trajectory.csv", "r") as file:
    reader = csv.DictReader(file)

    for row in reader:
        orb_x.append(float(row["x"]))
        orb_y.append(float(row["y"]))

# -----------------------------
# Plot comparison
# -----------------------------

plt.figure(figsize=(10, 8))

plt.plot(gps_x, gps_y, label="GPS Telemetry", linewidth=2)
plt.plot(orb_x, orb_y, label="ORB Estimated Trajectory", linewidth=2)

plt.title("ORB Trajectory vs GPS Telemetry")
plt.xlabel("X movement")
plt.ylabel("Y movement")

plt.legend()
plt.grid(True)
plt.axis("equal")

plt.savefig("results/trajectory_comparison.png")
plt.show()

print("Saved: results/trajectory_comparison.png")