import cv2
import numpy as np
import glob
from pathlib import Path


# === INSTELLINGEN ===

# aantal binnenhoeken (kolommen, rijen)
CHECKERBOARD = (8, 6)  # 8 binnenhoeken in breedte; 6 binnenhoeken in hoogte
SQUARE_SIZE = 0.285  # mm, of een andere eenheid

current_file = Path(__file__).resolve()
camera_dir = current_file.parent.parent
img_left_paths = sorted((camera_dir / "dual_calib_images" / "left").glob("*.jpg"))
img_right_paths = sorted((camera_dir / "dual_calib_images" / "right").glob("*.jpg"))

# === OBJECTPUNTEN VAN HET BORD ===
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D punten in echte wereld
imgpoints_l = []  # 2D punten linker camera
imgpoints_r = []  # 2D punten rechter camera

# Stop refinement after maximum 30 iterations or when improvement < 0.001
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# === HOEKPUNTEN ZOEKEN ===
for left_path, right_path in zip(img_left_paths, img_right_paths):
    img_l = cv2.imread(left_path)
    img_r = cv2.imread(right_path)

    gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

    # ret_l && ret_r: return values True || False
    # corners_l && corners_r: 2D image coords of found inner corners
    # findChessboardCorners gives a first rough estimation
    ret_l, corners_l = cv2.findChessboardCorners(gray_l, CHECKERBOARD, None)
    ret_r, corners_r = cv2.findChessboardCorners(gray_r, CHECKERBOARD, None)

    if ret_l and ret_r:  # Only go on when both images are found
        # cornerSubPix gives a more detailed estimation (uses subpixels)
        # 3rd argument: field of ...x... pixels around each corner point
        # 4th argument: zeroZone; (-1, -1): excluding nothing
        # 5th argument: STOP criterion
        corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), criteria)
        corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), criteria)

        # Add the points
        objpoints.append(objp)
        imgpoints_l.append(corners_l)
        imgpoints_r.append(corners_r)

img_size = gray_l.shape[
    ::-1
]  # (nr_of_rows, nr_of_columns); amount of pixels, so e.g. (1080, 1920)

# === CALIBRATING EACH CAMERA SEPARATELY ===
# ! ret_l & ret_r are not boolean here, but the RMS reprojection error
# rvecs1: rotation vectors of checkboard pose
# tvecs1: translation vectors ==
ret_l, K1, dist1, rvecs1, tvecs1 = cv2.calibrateCamera(
    objpoints, imgpoints_l, img_size, None, None
)

ret_r, K2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(
    objpoints, imgpoints_r, img_size, None, None
)

# === STEREO CALIBRATION ===
# Individual camera models are already known, now we only need to find the stereo relationship

flags = cv2.CALIB_FIX_INTRINSIC  # keep intrinsic params fixed during stereo calib

ret_stereo, K1, dist1, K2, dist2, R, T, E, F = cv2.stereoCalibrate(
    objpoints,
    imgpoints_l,
    imgpoints_r,
    K1,
    dist1,
    K2,
    dist2,
    img_size,
    criteria=criteria,
    flags=flags,
)

# === RECTIFICATIE ===

# R1,R2 are rectification rotations
# P1, P2 are the new matrices (instead of K1 and K2)
# Q handy for 3D reprojection
# roi: regions of interest in the rectified images
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, dist1, K2, dist2, img_size, R, T)

# Gives which pixels to read from the original left and right images
# Applies undistortion + rectification
# 6th argument: datatype of the maps
map1x, map1y = cv2.initUndistortRectifyMap(K1, dist1, R1, P1, img_size, cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(K2, dist2, R2, P2, img_size, cv2.CV_32FC1)

current_file = Path(__file__).resolve()
camera_dir = current_file.parent.parent
output_dir = camera_dir / "calibration_npz" / "stereo_calib.npz"

# === SAVE ===
np.savez(
    output_dir,
    K1=K1,
    dist1=dist1,
    K2=K2,
    dist2=dist2,
    R=R,
    T=T,
    R1=R1,
    R2=R2,
    P1=P1,
    P2=P2,
    Q=Q,
    map1x=map1x,
    map1y=map1y,
    map2x=map2x,
    map2y=map2y,
)

print("Stereo calibratie klaar.")
print("K1:\n", K1)
print("dist1:\n", dist1)
print("K2:\n", K2)
print("dist2:\n", dist2)
print("R:\n", R)
print("T:\n", T)
