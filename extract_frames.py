import cv2
import os

video_name = input("Enter video file name: ")
video_path = f"data/{video_name}"

output_folder = "frames"

os.makedirs(output_folder, exist_ok=True)

cap = cv2.VideoCapture(video_path)

fps = cap.get(cv2.CAP_PROP_FPS)
max_seconds = 30
max_frames = int(fps * max_seconds)

frame_count = 0
saved_count = 0
frame_interval = int(fps)  # save 1 frame per second

while frame_count < max_frames:
    ret, frame = cap.read()

    if not ret:
        break

    if frame_count % frame_interval == 0:
        frame_name = f"{output_folder}/frame_{saved_count:04d}.jpg"
        cv2.imwrite(frame_name, frame)
        print(f"Saved: {frame_name}")
        saved_count += 1

    frame_count += 1

cap.release()

print("\nDone extracting frames.")
print(f"FPS: {fps}")
print(f"Total frames saved: {saved_count}")