import csv
import os
import matplotlib.pyplot as plt

input_csv = "results/telemetry.csv"
output_folder = "results"
output_image = os.path.join(output_folder, "gps_trajectory.png")

latitudes = []
longitudes = []

with open(input_csv, "r", encoding="utf-8") as file:
    reader = csv.DictReader(file)

    for row in reader:
        latitudes.append(float(row["latitude"]))
        longitudes.append(float(row["longitude"]))

plt.figure()
plt.plot(longitudes, latitudes, marker=".", linewidth=1)
plt.title("GPS Trajectory from SRT Telemetry")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.grid(True)
plt.axis("equal")

plt.savefig(output_image)
plt.show()

print(f"GPS trajectory saved to: {output_image}")