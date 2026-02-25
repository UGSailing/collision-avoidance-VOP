#!/usr/bin/env python3
"""
Compute distance to an object using two Raspberry Pi HQ cameras,
with Z = (f * B) / d

- B = baseline (meters) between two camera centers
- f = focal length in *pixels* (fx from camera calibration)
- d = disparity in pixels (x_left - x_right) for the same object point

This file supports:
  1) Stereo calibration with a chessboard (recommended) -> saves calib_stereo.npz
  2) Live run mode: captures both cameras, rectifies, finds a colored object (default: red),
     computes disparity from centroid, and outputs Z.

Dependencies:
  pip install opencv-python numpy
  sudo apt install -y python3-picamera2   (recommended on Raspberry Pi OS)

Run:
  # 1) Collect calibration frames and compute calibration (chessboard)
  python3 stereo_distance_rpi_hq.py calibrate \
      --square-size-mm 24.0 --rows 6 --cols 9 --frames 25 --out calib_stereo.npz

  # 2) Live distance estimation (uses saved calibration)
  python3 stereo_distance_rpi_hq.py run --calib calib_stereo.npz --baseline-m 0.06

Notes:
- Accurate depth requires stereo calibration + rectification.
- Baseline must be measured accurately (center-to-center of lenses).
- Ensure both cameras use same resolution and are rigidly mounted.
"""

import argparse
import time
import sys
import os
import numpy as np
import cv2

# ----------------------------
# Camera backend (Picamera2 preferred, fallback to OpenCV VideoCapture)
# ----------------------------

def open_picamera2_dual(width: int, height: int, fps: int):
    """
    Opens two cameras using Picamera2 if available.
    Returns (cam0, cam1, read_fn) where read_fn() -> (frame0_bgr, frame1_bgr)
    """
    try:
        from picamera2 import Picamera2
    except Exception as e:
        return None, None, None

    cam0 = Picamera2(0)
    cam1 = Picamera2(1)

    # Use same config for both; use RGB888 then convert to BGR for OpenCV
    config0 = cam0.create_video_configuration(
        main={"format": "RGB888", "size": (width, height)},
        controls={"FrameRate": fps},
    )
    config1 = cam1.create_video_configuration(
        main={"format": "RGB888", "size": (width, height)},
        controls={"FrameRate": fps},
    )

    cam0.configure(config0)
    cam1.configure(config1)

    cam0.start()
    cam1.start()

    # Let AE settle a bit
    time.sleep(0.5)

    def read_fn():
        f0 = cam0.capture_array()
        f1 = cam1.capture_array()
        # Picamera2 returns RGB; OpenCV expects BGR
        return cv2.cvtColor(f0, cv2.COLOR_RGB2BGR), cv2.cvtColor(f1, cv2.COLOR_RGB2BGR)

    return cam0, cam1, read_fn


def open_opencv_dual(dev0: str, dev1: str, width: int, height: int, fps: int):
    cap0 = cv2.VideoCapture(dev0, cv2.CAP_V4L2)
    cap1 = cv2.VideoCapture(dev1, cv2.CAP_V4L2)

    for cap in (cap0, cap1):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap0.isOpened() or not cap1.isOpened():
        return None, None, None

    def read_fn():
        ok0, f0 = cap0.read()
        ok1, f1 = cap1.read()
        if not ok0 or not ok1:
            return None, None
        return f0, f1

    return cap0, cap1, read_fn


# ----------------------------
# Calibration utilities
# ----------------------------

def build_object_points(cols: int, rows: int, square_size_mm: float):
    """
    Build the 3D object points for a chessboard pattern on Z=0 plane.
    OpenCV uses (cols, rows) = number of inner corners per chessboard row/col.
    """
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= (square_size_mm / 1000.0)  # convert mm -> meters
    return objp


def find_chessboard(gray, cols: int, rows: int):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return False, None
    # Subpixel refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def stereo_calibrate_from_live(read_fn, cols: int, rows: int, square_size_mm: float,
                               frames: int, width: int, height: int, out_path: str):
    """
    Capture 'frames' valid pairs where chessboard is found in both cameras,
    then perform stereo calibration and save rectification maps.
    """
    objp = build_object_points(cols, rows, square_size_mm)

    objpoints = []
    imgpoints0 = []
    imgpoints1 = []

    print("[INFO] Calibration capture starting.")
    print("       Show the chessboard to BOTH cameras. Press 'c' to capture a pair when detected.")
    print("       Press 'q' to quit early.\n")

    captured = 0
    while captured < frames:
        f0, f1 = read_fn()
        if f0 is None or f1 is None:
            print("[ERROR] Failed to read frames.")
            break

        g0 = cv2.cvtColor(f0, cv2.COLOR_BGR2GRAY)
        g1 = cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)

        ok0, corners0 = find_chessboard(g0, cols, rows)
        ok1, corners1 = find_chessboard(g1, cols, rows)

        disp0 = f0.copy()
        disp1 = f1.copy()

        if ok0:
            cv2.drawChessboardCorners(disp0, (cols, rows), corners0, ok0)
        if ok1:
            cv2.drawChessboardCorners(disp1, (cols, rows), corners1, ok1)

        both = ok0 and ok1
        cv2.putText(disp0, f"Cam0 chessboard: {'OK' if ok0 else 'NO'}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if ok0 else (0, 0, 255), 2)
        cv2.putText(disp1, f"Cam1 chessboard: {'OK' if ok1 else 'NO'}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if ok1 else (0, 0, 255), 2)

        stacked = np.hstack([disp0, disp1])
        cv2.putText(stacked, f"Captured pairs: {captured}/{frames} | Press 'c' to capture | 'q' quit",
                    (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Stereo Calibration (left | right)", stacked)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('c'):
            if both:
                objpoints.append(objp)
                imgpoints0.append(corners0)
                imgpoints1.append(corners1)
                captured += 1
                print(f"[INFO] Captured {captured}/{frames}")
            else:
                print("[WARN] Chessboard not detected in both cameras; not captured.")

    cv2.destroyAllWindows()

    if captured < 10:
        raise RuntimeError("Not enough calibration pairs. Capture at least ~15-25 for decent results.")

    image_size = (width, height)

    print("\n[INFO] Calibrating individual cameras...")
    ret0, K0, D0, rvecs0, tvecs0 = cv2.calibrateCamera(objpoints, imgpoints0, image_size, None, None)
    ret1, K1, D1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints1, image_size, None, None)

    print("[INFO] Stereo calibration...")
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    rms, K0, D0, K1, D1, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints0, imgpoints1,
        K0, D0, K1, D1,
        image_size,
        criteria=criteria,
        flags=flags
    )

    # Rectification
    R0, R1, P0, P1, Q, roi0, roi1 = cv2.stereoRectify(
        K0, D0, K1, D1, image_size, R, T, alpha=0
    )

    map0x, map0y = cv2.initUndistortRectifyMap(K0, D0, R0, P0, image_size, cv2.CV_32FC1)
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)

    # fx in pixels for depth formula (after rectification, use P0[0,0])
    fx = float(P0[0, 0])
    # baseline magnitude in meters from T
    baseline = float(np.linalg.norm(T))

    print("\n[RESULT]")
    print(f"  RMS reprojection error stereo: {rms:.6f}")
    print(f"  fx (pixels): {fx:.3f}")
    print(f"  Baseline from calibration |T| (m): {baseline:.6f}")
    print(f"\n[INFO] Saving calibration to: {out_path}")

    np.savez(
        out_path,
        K0=K0, D0=D0, K1=K1, D1=D1,
        R=R, T=T, E=E, F=F,
        R0=R0, R1=R1, P0=P0, P1=P1, Q=Q,
        map0x=map0x, map0y=map0y, map1x=map1x, map1y=map1y,
        fx=fx, baseline=baseline,
        image_size=np.array([width, height], dtype=np.int32),
    )


def load_calib(npz_path: str):
    d = np.load(npz_path, allow_pickle=True)
    calib = {k: d[k] for k in d.files}
    return calib


# ----------------------------
# Object detection (simple, robust baseline: detect a red object by HSV threshold)
# ----------------------------

def detect_red_centroid(bgr):
    """
    Returns (cx, cy, area, mask) for the largest red blob, or (None, None, 0, mask) if not found.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Red wraps around hue=0; combine two ranges.
    lower1 = np.array([0, 120, 70])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 120, 70])
    upper2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Clean up
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, 0, mask

    c = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area < 200.0:  # tune based on distance/resolution
        return None, None, area, mask

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, None, area, mask
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy, area, mask


# ----------------------------
# Live run: compute Z = fx * B / disparity
# ----------------------------

def run_live(read_fn, calib, baseline_m_override: float | None,
             show_masks: bool, min_disparity_px: float):
    map0x = calib["map0x"]
    map0y = calib["map0y"]
    map1x = calib["map1x"]
    map1y = calib["map1y"]

    fx = float(calib["fx"])
    baseline_from_calib = float(calib["baseline"])
    B = float(baseline_m_override) if baseline_m_override is not None else baseline_from_calib

    print("[INFO] Live run starting.")
    print(f"       Using fx={fx:.3f} px")
    print(f"       Using baseline B={B:.6f} m  (calib baseline={baseline_from_calib:.6f} m)")
    print("       Press 'q' to quit.\n")

    while True:
        f0, f1 = read_fn()
        if f0 is None or f1 is None:
            print("[ERROR] Frame read failed.")
            break

        # Rectify
        left = cv2.remap(f0, map0x, map0y, cv2.INTER_LINEAR)
        right = cv2.remap(f1, map1x, map1y, cv2.INTER_LINEAR)

        # Detect object in each
        lx, ly, larea, lmask = detect_red_centroid(left)
        rx, ry, rarea, rmask = detect_red_centroid(right)

        vis = np.hstack([left.copy(), right.copy()])
        h, w = left.shape[:2]

        # draw epipolar reference lines (should align after rectification)
        for y in range(0, h, 80):
            cv2.line(vis, (0, y), (2*w, y), (255, 255, 255), 1)

        z_text = "Z: n/a"
        if lx is not None and rx is not None:
            # In rectified images, corresponding points share (approximately) same y
            disparity = float(lx - rx)

            if abs(disparity) >= min_disparity_px:
                Z = (fx * B) / disparity  # meters (sign depends on camera ordering)
                Z = abs(Z)
                z_text = f"Z: {Z:.3f} m | d={disparity:.2f}px"
            else:
                z_text = f"Z: n/a (|d|<{min_disparity_px}px) | d={disparity:.2f}px"

            # Draw detections
            cv2.circle(vis, (lx, ly), 8, (0, 255, 0), -1)
            cv2.circle(vis, (w + rx, ry), 8, (0, 255, 0), -1)
            cv2.line(vis, (lx, ly), (w + rx, ry), (0, 255, 255), 2)

        cv2.putText(vis, z_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4)
        cv2.putText(vis, z_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(vis, "Left | Right (rectified)", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("Stereo Distance", vis)

        if show_masks:
            cv2.imshow("Mask Left", lmask)
            cv2.imshow("Mask Right", rmask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


# ----------------------------
# Main
# ----------------------------

def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    # Shared camera args
    def add_cam_args(p):
        p.add_argument("--width", type=int, default=1280)
        p.add_argument("--height", type=int, default=720)
        p.add_argument("--fps", type=int, default=30)
        p.add_argument("--backend", choices=["picamera2", "opencv"], default="picamera2")
        p.add_argument("--dev0", default="/dev/video0", help="OpenCV backend device for cam0")
        p.add_argument("--dev1", default="/dev/video1", help="OpenCV backend device for cam1")

    # calibrate
    p_cal = sub.add_parser("calibrate", help="Stereo calibrate with chessboard and save npz")
    add_cam_args(p_cal)
    p_cal.add_argument("--cols", type=int, required=True, help="Chessboard inner corners per row (columns)")
    p_cal.add_argument("--rows", type=int, required=True, help="Chessboard inner corners per column (rows)")
    p_cal.add_argument("--square-size-mm", type=float, required=True, help="Chessboard square size (mm)")
    p_cal.add_argument("--frames", type=int, default=25, help="Number of valid stereo pairs to capture")
    p_cal.add_argument("--out", default="calib_stereo.npz")

    # run
    p_run = sub.add_parser("run", help="Live distance estimation using saved calibration")
    add_cam_args(p_run)
    p_run.add_argument("--calib", default="calib_stereo.npz")
    p_run.add_argument("--baseline-m", type=float, default=None,
                       help="Override baseline in meters (recommended to measure physically). If omitted, uses calibration baseline.")
    p_run.add_argument("--show-masks", action="store_true", help="Show debug masks for object detection")
    p_run.add_argument("--min-disparity-px", type=float, default=1.5,
                       help="Minimum |d| in pixels to accept (avoid huge/noisy Z)")

    args = ap.parse_args()

    # Open cameras
    if args.backend == "picamera2":
        cam0, cam1, read_fn = open_picamera2_dual(args.width, args.height, args.fps)
        if read_fn is None:
            print("[WARN] Picamera2 not available; falling back to OpenCV backend.")
            args.backend = "opencv"

    if args.backend == "opencv":
        cam0, cam1, read_fn = open_opencv_dual(args.dev0, args.dev1, args.width, args.height, args.fps)

    if read_fn is None:
        print("[ERROR] Could not open both cameras. Check /dev/video* or Picamera2 setup.")
        sys.exit(1)

    try:
        if args.cmd == "calibrate":
            stereo_calibrate_from_live(
                read_fn=read_fn,
                cols=args.cols,
                rows=args.rows,
                square_size_mm=args.square_size_mm,
                frames=args.frames,
                width=args.width,
                height=args.height,
                out_path=args.out,
            )
            print("[DONE] Calibration finished.")
        elif args.cmd == "run":
            if not os.path.exists(args.calib):
                print(f"[ERROR] Calibration file not found: {args.calib}")
                print("        Run calibration first.")
                sys.exit(1)
            calib = load_calib(args.calib)
            run_live(
                read_fn=read_fn,
                calib=calib,
                baseline_m_override=args.baseline_m,
                show_masks=args.show_masks,
                min_disparity_px=args.min_disparity_px,
            )
    finally:
        # Release cameras
        if args.backend == "picamera2":
            try:
                cam0.stop(); cam1.stop()
            except Exception:
                pass
        else:
            try:
                cam0.release(); cam1.release()
            except Exception:
                pass


if __name__ == "__main__":
    main()
