import re
import csv
import os


def srt_time_to_seconds(time_str):
    """
    Convert SRT time format HH:MM:SS,mmm to seconds.
    Example: 00:00:01,500 -> 1.5
    """
    h, m, rest = time_str.split(":")
    s, ms = rest.split(",")
    return int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000


def parse_srt(file_path):
    telemetry_data = []

    with open(file_path, "r", encoding="utf-8", errors="ignore") as file:
        content = file.read()

    blocks = content.strip().split("\n\n")

    for block in blocks:
        lines = block.splitlines()

        if len(lines) < 3:
            continue

        index = lines[0].strip()
        time_line = lines[1].strip()
        text = " ".join(lines[2:])

        start_time = time_line.split("-->")[0].strip()
        time_seconds = srt_time_to_seconds(start_time)

        lat_match = re.search(r"\[latitude:\s*([-0-9.]+)\]", text)
        lon_match = re.search(r"\[longitude:\s*([-0-9.]+)\]", text)
        alt_match = re.search(r"\[rel_alt:\s*([-0-9.]+)\s+abs_alt:\s*([-0-9.]+)\]", text)

        if not lat_match or not lon_match or not alt_match:
            continue

        latitude = float(lat_match.group(1))
        longitude = float(lon_match.group(1))
        rel_alt = float(alt_match.group(1))
        abs_alt = float(alt_match.group(2))

        # Ignore invalid GPS entries at the beginning
        if latitude == 0.0 and longitude == 0.0:
            continue

        telemetry_data.append({
            "index": int(index),
            "time_seconds": time_seconds,
            "latitude": latitude,
            "longitude": longitude,
            "rel_alt": rel_alt,
            "abs_alt": abs_alt
        })

    return telemetry_data


def save_to_csv(data, output_path):
    if not data:
        print("No telemetry data to save.")
        return

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    fieldnames = ["index", "time_seconds", "latitude", "longitude", "rel_alt", "abs_alt"]

    with open(output_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(data)


if __name__ == "__main__":
    srt_name = input("Enter SRT file name: ")
    srt_file = f"data/{srt_name}"
    output_csv = "results/telemetry.csv"

    data = parse_srt(srt_file)

    print(f"\nParsed valid GPS telemetry entries: {len(data)}\n")

    for entry in data[:5]:
        print(entry)

    save_to_csv(data, output_csv)
    print(f"\nTelemetry CSV saved to: {output_csv}")