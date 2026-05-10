import cv2
import os
import csv
import glob
import numpy as np

frames_folder = "frames"
telemetry_csv = "results/telemetry.csv"

orb = cv2.ORB_create(nfeatures=1000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)


def load_telemetry(csv_path):
    telemetry = []

    with open(csv_path, "r", encoding="utf-8") as file:
        reader = csv.DictReader(file)

        for row in reader:
            telemetry.append({
                "time_seconds": float(row["time_seconds"]),
                "latitude": float(row["latitude"]),
                "longitude": float(row["longitude"]),
                "rel_alt": float(row["rel_alt"]),
                "abs_alt": float(row["abs_alt"])
            })

    return telemetry


def get_nearest_telemetry(telemetry, time_seconds):
    return min(
        telemetry,
        key=lambda row: abs(row["time_seconds"] - time_seconds)
    )


def build_reference_database(frame_paths, telemetry, fps=30):
    database = []

    for i, frame_path in enumerate(frame_paths):
        image = cv2.imread(frame_path, cv2.IMREAD_GRAYSCALE)

        if image is None:
            continue

        keypoints, descriptors = orb.detectAndCompute(image, None)

        if descriptors is None:
            continue

        # כי חילצנו frame אחד כל שנייה בערך
        frame_time = i * 1.0

        telemetry_entry = get_nearest_telemetry(telemetry, frame_time)

        database.append({
            "frame_path": frame_path,
            "time_seconds": frame_time,
            "keypoints": keypoints,
            "descriptors": descriptors,
            "latitude": telemetry_entry["latitude"],
            "longitude": telemetry_entry["longitude"],
            "rel_alt": telemetry_entry["rel_alt"],
            "abs_alt": telemetry_entry["abs_alt"]
        })

    return database


def localize_frame(query_frame_path, database):
    query_image = cv2.imread(query_frame_path, cv2.IMREAD_GRAYSCALE)

    if query_image is None:
        print("Could not read query frame.")
        return None

    query_keypoints, query_descriptors = orb.detectAndCompute(query_image, None)

    if query_descriptors is None:
        print("No ORB features found in query frame.")
        return None

    best_match = None
    best_score = -1

    for entry in database:
        matches = bf.match(query_descriptors, entry["descriptors"])
        matches = sorted(matches, key=lambda x: x.distance)

        good_matches = [m for m in matches if m.distance < 60]

        score = len(good_matches)

        if score > best_score:
            best_score = score
            best_match = entry

    if best_match is None:
        return None

    return {
        "matched_frame": best_match["frame_path"],
        "score": best_score,
        "estimated_latitude": best_match["latitude"],
        "estimated_longitude": best_match["longitude"],
        "estimated_rel_alt": best_match["rel_alt"],
        "estimated_abs_alt": best_match["abs_alt"]
    }


if __name__ == "__main__":
    frame_paths = sorted(glob.glob(os.path.join(frames_folder, "*.jpg")))

    if not frame_paths:
        print("No frames found. Run extract_frames.py first.")
        exit()

    telemetry = load_telemetry(telemetry_csv)

    print("Building visual reference database...")
    database = build_reference_database(frame_paths, telemetry)

    print(f"Reference database contains {len(database)} frames.")

    # מדמה פריים בזמן אמת
    query_frame = input("Enter query frame path, for example frames/frame_0015.jpg: ")

    result = localize_frame(query_frame, database)

    if result:
        print("\nEstimated Real-Time Drone Position:")
        print(f"Matched reference frame: {result['matched_frame']}")
        print(f"Matching score: {result['score']}")
        print(f"Latitude: {result['estimated_latitude']}")
        print(f"Longitude: {result['estimated_longitude']}")
        print(f"Relative altitude: {result['estimated_rel_alt']}")
        print(f"Absolute altitude: {result['estimated_abs_alt']}")
    else:
        print("Localization failed.")