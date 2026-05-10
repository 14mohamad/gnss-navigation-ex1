import cv2
import os
import glob

frames_folder = "frames"
output_folder = "matches"

os.makedirs(output_folder, exist_ok=True)

orb = cv2.ORB_create(nfeatures=1000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

frame_paths = sorted(glob.glob(os.path.join(frames_folder, "*.jpg")))

for i in range(len(frame_paths) - 1):
    img1 = cv2.imread(frame_paths[i], cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(frame_paths[i + 1], cv2.IMREAD_GRAYSCALE)

    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    if des1 is None or des2 is None:
        print(f"Skipping pair {i}: no descriptors")
        continue

    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)

    good_matches = matches[:50]

    matched_img = cv2.drawMatches(
        img1, kp1,
        img2, kp2,
        good_matches,
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )

    output_path = os.path.join(output_folder, f"match_{i:04d}.jpg")
    cv2.imwrite(output_path, matched_img)

    print(f"Saved: {output_path} | Matches: {len(good_matches)}")

print("\nDone ORB matching.")