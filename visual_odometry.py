import cv2
import numpy as np
import math

class VisualOdometry:
    def __init__(self, nfeatures=1000):
        self.orb = cv2.ORB_create(nfeatures=nfeatures)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def estimate_motion(self, prev_img, curr_img):
        """
        Estimate 2D translation and rotation from prev_img to curr_img.
        Returns:
            tx (float): Translation in x (pixels, rightward)
            ty (float): Translation in y (pixels, downward)
            theta (float): Rotation change (radians, counter-clockwise)
            success (bool): True if motion was successfully estimated
        """
        # Convert to grayscale if color
        if len(prev_img.shape) == 3:
            prev_gray = cv2.cvtColor(prev_img, cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)
        else:
            prev_gray = prev_img
            curr_gray = curr_img

        # Detect and compute ORB features
        kp1, des1 = self.orb.detectAndCompute(prev_gray, None)
        kp2, des2 = self.orb.detectAndCompute(curr_gray, None)

        if des1 is None or des2 is None or len(des1) < 10 or len(des2) < 10:
            return 0.0, 0.0, 0.0, False

        # Match descriptors
        matches = self.bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        # Filter by distance
        good_matches = [m for m in matches if m.distance < 50]

        if len(good_matches) < 8:
            return 0.0, 0.0, 0.0, False

        # Extract coordinates of matched keypoints
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Center keypoints around image center to remove translation bias from rotation/scale
        h, w = prev_img.shape[:2]
        cx, cy = w / 2.0, h / 2.0
        src_pts_centered = src_pts - [[[cx, cy]]]
        dst_pts_centered = dst_pts - [[[cx, cy]]]

        # Estimate partial affine transform (translation, rotation, scale)
        M, inliers = cv2.estimateAffinePartial2D(src_pts_centered, dst_pts_centered, method=cv2.RANSAC, ransacReprojThreshold=3.0)

        if M is None or inliers is None or np.sum(inliers) < 5:
            return 0.0, 0.0, 0.0, False

        # Extract rotation (theta) and translation (tx, ty) from affine matrix
        tx = M[0, 2]
        ty = M[1, 2]
        theta = math.atan2(M[1, 0], M[0, 0])

        return tx, ty, theta, True

    def pixels_to_meters(self, tx_px, ty_px, relative_altitude, img_width, fov_horizontal_deg=73.7):
        """
        Convert pixel translation offsets to physical meters translation offsets.
        Note: OpenCV image space has Y-axis pointing down. 
        We return dX (rightward, meters) and dY (forward/upward on image, meters).
        """
        # Ground width visible in frame at this relative altitude
        fov_rad = math.radians(fov_horizontal_deg)
        ground_width_m = 2.0 * relative_altitude * math.tan(fov_rad / 2.0)
        
        meters_per_pixel = ground_width_m / img_width

        # Convert to local coordinate system (X right, Y forward in image plane)
        # If ground features move left (tx_px < 0), the drone moved right (dx_m > 0).
        # If ground features move down (ty_px > 0), the drone moved forward (dy_m > 0).
        dx_m = -tx_px * meters_per_pixel
        dy_m = ty_px * meters_per_pixel

        return dx_m, dy_m
