# Vision-Based Navigation for Drones

## Project Overview

This project presents a preliminary implementation of a vision-based drone navigation system using computer vision and telemetry data. The system processes drone flight videos together with SRT telemetry information in order to estimate drone motion and trajectory without relying entirely on GNSS during navigation.

The implementation focuses on visual feature extraction, feature matching, trajectory estimation, and comparison between estimated visual motion and real GPS telemetry.

---

# Main Features

- Drone video preprocessing
- Frame extraction from video
- ORB feature detection and matching
- Relative motion estimation
- Estimated trajectory generation
- SRT telemetry parsing
- GPS trajectory visualization
- Comparison between visual trajectory and real telemetry trajectory

---

# Project Structure

```text
gnss-navigation-ex1/
│
├── data/
│   ├── video1.mp4
│   ├── v11.mp4
│   ├── v11.srt
│
├── frames/
├── matches/
├── results/
│   ├── estimated_trajectory.png
│   ├── gps_trajectory.png
│   ├── trajectory_comparison.png
│   ├── telemetry.csv
│   ├── orb_trajectory.csv
│
├── extract_frames.py
├── orb_matching.py
├── trajectory.py
├── parse_srt.py
├── gps_trajectory.py
├── compare_trajectories.py
├── requirements.txt
└── README.md
```

---

# Technologies Used

- Python 3
- OpenCV
- NumPy
- Matplotlib
- CSV processing
- Drone SRT telemetry data

---

# Installation

Install the required Python packages:

```bash
pip install -r requirements.txt
```

---

# Input Data

The project uses:

- Drone video recordings
- SRT telemetry files containing:
  - GPS coordinates
  - Relative altitude
  - Absolute altitude
  - Timestamps

---

# Usage

## 1. Extract video frames

```bash
python3 extract_frames.py
```

Frames will be saved inside:

```text
frames/
```

---

## 2. Perform ORB feature matching

```bash
python3 orb_matching.py
```

Matched feature visualizations will be saved inside:

```text
matches/
```

---

## 3. Generate estimated ORB trajectory

```bash
python3 trajectory.py
```

Outputs:

- Estimated trajectory image
- ORB trajectory CSV

Saved inside:

```text
results/
```

---

## 4. Parse SRT telemetry

```bash
python3 parse_srt.py
```

Outputs:

```text
results/telemetry.csv
```

---

## 5. Generate GPS trajectory

```bash
python3 gps_trajectory.py
```

Outputs:

```text
results/gps_trajectory.png
```

---

## 6. Compare ORB trajectory with GPS telemetry

```bash
python3 compare_trajectories.py
```

Outputs:

```text
results/trajectory_comparison.png
```

---

# Results

The system successfully demonstrates:

- Relative motion estimation using visual features
- Drone trajectory estimation from video frames
- GPS telemetry extraction from SRT files
- Comparison between visual navigation and real telemetry data

Although the estimated ORB trajectory is not yet geo-referenced or scale-corrected, the system demonstrates the feasibility of vision-based drone navigation.

---

# Future Improvements

Possible future improvements include:

- Integration of IMU sensor fusion
- Full ORB-SLAM3 integration
- Scale correction and trajectory alignment
- Real-time processing
- Loop closure optimization
- Deep-learning-based feature extraction
- GNSS-denied autonomous navigation

---

# Authors

Mohamad mousa and Melissa Liebowitz
Vision-Based Navigation Project