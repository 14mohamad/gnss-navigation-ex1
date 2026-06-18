import cv2
import time
import os
import math

from parse_srt import parse_srt
from visual_odometry import VisualOdometry

VIDEO_SOURCE = "data/v2.mp4"
SRT_SOURCE = "data/v2.SRT"
OUTPUT_KML = "results/estimated_drone_path.kml"
FRAME_INTERVAL = 5  # process every 5th frame for speed/motion balance

# Initialize Visual Odometry
vo = VisualOdometry()

# Load telemetry
print(f"Loading SRT telemetry from {SRT_SOURCE}...")
telemetry = parse_srt(SRT_SOURCE)
if not telemetry:
    print("Error: Could not parse telemetry data.")
    exit()

print(f"Parsed {len(telemetry)} telemetry entries.")

# Dynamically self-calibrate the takeoff heading using first horizontal movement
def self_calibrate_takeoff_heading(video_path, telemetry_data, frame_interval):
    start_lat = telemetry_data[0]["latitude"]
    start_lon = telemetry_data[0]["longitude"]
    
    # 1. Find alignment time where drone has moved 15m
    t_align = None
    flight_course = 0.0
    for entry in telemetry_data:
        lat = entry["latitude"]
        lon = entry["longitude"]
        d_north = (lat - start_lat) * 111320.0
        d_east = (lon - start_lon) * 111320.0 * math.cos(math.radians(start_lat))
        dist = math.sqrt(d_north**2 + d_east**2)
        if dist > 15.0:
            t_align = entry["time_seconds"]
            flight_course = math.atan2(d_east, d_north)
            break
            
    if t_align is None:
        print("Warning: Drone did not move horizontally. Defaulting to 0 takeoff heading.")
        return 0.0
        
    print(f"Self-calibration: Alignment time detected at {t_align:.1f}s. Target flight course: {math.degrees(flight_course):.1f}°")
    
    # 2. Accumulate rotation up to t_align
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0
        
    frame_counter = 0
    prev_frame = None
    accum_rot = 0.0
    prev_rel_alt = telemetry_data[0]["rel_alt"]
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_counter += 1
        if frame_counter % frame_interval != 0:
            continue
            
        time_seconds = frame_counter / fps
        if time_seconds > t_align:
            break
            
        t_entry = min(telemetry_data, key=lambda x: abs(x["time_seconds"] - time_seconds))
        curr_rel_alt = t_entry["rel_alt"]
        is_climbing_or_descending = abs(curr_rel_alt - prev_rel_alt) > 0.5
        
        if prev_frame is not None:
            tx, ty, theta, success = vo.estimate_motion(prev_frame, frame)
            if success and not is_climbing_or_descending:
                if abs(theta) > 0.002:
                    accum_rot -= theta
                    
        prev_frame = frame.copy()
        prev_rel_alt = curr_rel_alt
        
    cap.release()
    
    takeoff_heading = flight_course - accum_rot
    print(f"Self-calibration: Accumulated {math.degrees(accum_rot):.1f}° rotation up to align. Calculated takeoff heading: {math.degrees(takeoff_heading):.1f}°")
    return takeoff_heading

def detect_takeoff_altitude_offset(telemetry_data):
    start_lat = telemetry_data[0]["latitude"]
    start_lon = telemetry_data[0]["longitude"]
    start_alt = telemetry_data[0]["rel_alt"]
    
    max_climb = 0.0
    max_dist = 0.0
    
    # Analyze the first 20 seconds of telemetry
    first_20s_telemetry = [entry for entry in telemetry_data if entry["time_seconds"] <= 20.0]
    
    for entry in first_20s_telemetry:
        climb = entry["rel_alt"] - start_alt
        d_north = (entry["latitude"] - start_lat) * 111320.0
        d_east = (entry["longitude"] - start_lon) * 111320.0 * math.cos(math.radians(start_lat))
        dist = math.sqrt(d_north**2 + d_east**2)
        max_climb = max(max_climb, climb)
        max_dist = max(max_dist, dist)
        
    # Heuristic: vertical takeoff has high altitude climb but very small horizontal movement
    is_takeoff = (max_climb > 15.0 and max_dist < 10.0)
    takeoff_offset = start_alt if is_takeoff else 0.0
    
    print(f"Takeoff Offset Detection: Max Climb: {max_climb:.1f}m, Max Horiz Dist: {max_dist:.1f}m. "
          f"Determined Takeoff: {is_takeoff}. Takeoff Altitude Offset: {takeoff_offset:.1f}m")
    return takeoff_offset

start_t = telemetry[0]
current_lat = start_t["latitude"]
current_lon = start_t["longitude"]
current_alt = start_t["abs_alt"]
current_heading = self_calibrate_takeoff_heading(VIDEO_SOURCE, telemetry, FRAME_INTERVAL)
takeoff_altitude_offset = detect_takeoff_altitude_offset(telemetry)
print(f"Dynamically aligned takeoff heading: {math.degrees(current_heading):.1f}°")

estimated_positions = [{
    "frame": 0,
    "latitude": current_lat,
    "longitude": current_lon,
    "altitude": current_alt
}]

def get_telemetry_at_time(time_sec):
    return min(telemetry, key=lambda x: abs(x["time_seconds"] - time_sec))

def save_positions_to_kml(positions, output_path):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<name>Estimated Drone Path (Visual Odometry)</name>
<Placemark>
<name>Estimated Path</name>
<LineString>
<altitudeMode>absolute</altitudeMode>
<coordinates>
""")
        for pos in positions:
            f.write(f"{pos['longitude']},{pos['latitude']},{pos['altitude']}\n")
        f.write("""</coordinates>
</LineString>
</Placemark>
</Document>
</kml>
""")

# Open video
cap = cv2.VideoCapture(VIDEO_SOURCE)
if not cap.isOpened():
    print(f"Error: Could not open video: {VIDEO_SOURCE}")
    exit()

fps = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = 30.0

print(f"Video opened. FPS: {fps:.2f}. Processing every {FRAME_INTERVAL}th frame.")
print("\nStarting Real-Time Visual Odometry...\n")

frame_counter = 0
prev_frame = None
prev_time = 0.0
prev_rel_alt = telemetry[0]["rel_alt"]

total_start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame_counter += 1
    
    # Process at interval
    if frame_counter % FRAME_INTERVAL != 0:
        continue
        
    time_seconds = frame_counter / fps
    t = get_telemetry_at_time(time_seconds)
    
    if prev_frame is None:
        # First frame initialization
        prev_frame = frame.copy()
        prev_time = time_seconds
        prev_rel_alt = t["rel_alt"]
        continue
        
    start_match = time.time()
    
    # Estimate frame-to-frame motion in pixels/radians
    tx, ty, theta, success = vo.estimate_motion(prev_frame, frame)
    elapsed_match = time.time() - start_match
    
    curr_rel_alt = t["rel_alt"]
    is_climbing_or_descending = abs(curr_rel_alt - prev_rel_alt) > 0.5
    
    if success:
        # Convert pixels to meters (subtracting takeoff altitude offset and using calibrated 79° FOV)
        height_above_ground = max(2.0, curr_rel_alt - takeoff_altitude_offset)
        dx_m, dy_m = vo.pixels_to_meters(tx, ty, height_above_ground, frame.shape[1], fov_horizontal_deg=79.0)
        
        # Accumulate heading rotation only if not climbing/descending
        if not is_climbing_or_descending:
            # Accumulate theta using calibrated 0.002 threshold
            if abs(theta) > 0.002:
                current_heading -= theta
        
        # Rotate body coordinates (dx_m, dy_m) to world coordinates (dEast, dNorth)
        # dx_m is right, dy_m is forward
        dNorth = dy_m * math.cos(current_heading) - dx_m * math.sin(current_heading)
        dEast = dy_m * math.sin(current_heading) + dx_m * math.cos(current_heading)
        
        # Convert meters to GPS coordinates
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(current_lat))
        
        current_lat += dNorth / meters_per_deg_lat
        current_lon += dEast / meters_per_deg_lon
        current_alt = t["abs_alt"]
        
        estimated_positions.append({
            "frame": frame_counter,
            "latitude": current_lat,
            "longitude": current_lon,
            "altitude": current_alt
        })
        
        if len(estimated_positions) % 10 == 0 or frame_counter % 100 == 0:
            print(f"Frame: {frame_counter:4d} | Time: {time_seconds:5.1f}s | "
                  f"GPS: ({current_lat:.6f}, {current_lon:.6f}) | Heading: {math.degrees(current_heading):.1f}° | "
                  f"dx: {dx_m:5.2f}m | dy: {dy_m:5.2f}m | Match Time: {elapsed_match:.3f}s")
                  
    else:
        # If motion estimation fails, assume persistence of previous position (hover/constant motion)
        # print(f"Warning: Motion estimation failed at frame {frame_counter}")
        pass
        
    prev_frame = frame.copy()
    prev_time = time_seconds
    prev_rel_alt = curr_rel_alt

cap.release()
print(f"\nProcessing complete in {time.time() - total_start_time:.2f} seconds.")

if len(estimated_positions) > 1:
    save_positions_to_kml(estimated_positions, OUTPUT_KML)
    print(f"KML saved successfully to: {OUTPUT_KML}")
    print(f"Total positions recorded: {len(estimated_positions)}")
else:
    print("Error: No positions estimated.")
