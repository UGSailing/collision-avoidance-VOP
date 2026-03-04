#!/usr/bin/env python3
import os
import time
import glob
import argparse
import subprocess
import numpy as np
import cv2

def run(cmd, timeout=30):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout, r.stderr

def _try_open_preview(camera_id, width, height):
    # Try V4L2 first (common on Pi if libcamera-v4l2 is enabled)
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(camera_id)  # fallback
    if not cap.isOpened():
        cap.release()
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap

def capture_images_keypress(out_dir, n, camera_id, width, height):
    os.makedirs(out_dir, exist_ok=True)
    print(f"[CAPTURE] Keypress capture: need {n} images from camera {camera_id} -> {out_dir}")
    print("Controls: 'c' = capture, 'q' = quit")

    # Try a live preview using OpenCV
    cap = _try_open_preview(camera_id, width, height)

    i = 0
    if cap is not None:
        win = "preview (press 'c' to capture, 'q' to quit)"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        while i < n:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Preview frame grab failed. Switching to terminal key capture.")
                break

            # simple overlay
            overlay = frame.copy()
            cv2.putText(
                overlay,
                f"{i}/{n}  press 'c' to capture",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(win, overlay)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                print("[CAPTURE] Quit by user.")
                break
            if k == ord('c'):
                fname = os.path.join(out_dir, f"img_{i:03d}.jpg")
                cmd = [
                    "rpicam-still",
                    "--camera", str(camera_id),
                    "--output", fname,
                    "--timeout", "200",
                    "--width", str(width),
                    "--height", str(height),
                    "--nopreview",
                    "--denoise", "off",
                    "--awbgains", "1.0,1.0",
                    "--shutter", "5000",
                    "--gain", "2.0"
                ]
                print("  ", " ".join(cmd))
                code, out, err = run(cmd, timeout=20)
                if code != 0 or not os.path.exists(fname):
                    print("[ERROR] capture failed:", err.strip() or out.strip())
                    print("Stop. Fix exposure/light/camera first.")
                    cap.release()
                    cv2.destroyAllWindows()
                    return False
                print(f"[OK] {fname} ({os.path.getsize(fname)} bytes)")
                i += 1

        cap.release()
        cv2.destroyAllWindows()

    # If preview not available or it broke: terminal-driven key capture
    if i < n:
        print("[INFO] Terminal capture mode (no preview). Press Enter to capture, 'q' + Enter to quit.")
        while i < n:
            s = input(f"Capture {i}/{n} > ").strip().lower()
            if s == 'q':
                print("[CAPTURE] Quit by user.")
                break

            fname = os.path.join(out_dir, f"img_{i:03d}.jpg")
            cmd = [
                "rpicam-still",
                "--camera", str(camera_id),
                "--output", fname,
                "--timeout", "200",
                "--width", str(width),
                "--height", str(height),
                "--nopreview",
                "--denoise", "off",
                "--awbgains", "1.0,1.0",
                "--shutter", "5000",
                "--gain", "2.0"
            ]
            print("  ", " ".join(cmd))
            code, out, err = run(cmd, timeout=20)
            if code != 0 or not os.path.exists(fname):
                print("[ERROR] capture failed:", err.strip() or out.strip())
                print("Stop. Fix exposure/light/camera first.")
                return False
            print(f"[OK] {fname} ({os.path.getsize(fname)} bytes)")
            i += 1

    return i >= 10  # keep same "need enough images" spirit

def calibrate(images, board_cols, board_rows, square_size_m, show=False):
    pattern_size = (board_cols, board_rows)

    objp = np.zeros((board_rows * board_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp *= square_size_m

    objpoints = []
    imgpoints = []

    img_size = None
    good = 0

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for path in images:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not found:
            print(f"[MISS] {os.path.basename(path)}: no chessboard")
            continue

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        good += 1
        print(f"[HIT]  {os.path.basename(path)}")

        if show:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, pattern_size, corners2, found)
            cv2.imshow("corners", vis)
            cv2.waitKey(250)

    if show:
        cv2.destroyAllWindows()

    if good < 10:
        raise RuntimeError(f"Too few valid images ({good}). Aim for 15–30 good shots.")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_size, None, None
    )

    total_err = 0.0
    total_pts = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2)
        total_err += err * err
        total_pts += len(objpoints[i])
    rmse = np.sqrt(total_err / total_pts)

    return {
        "image_size": img_size,
        "rms": float(ret),
        "rmse_px": float(rmse),
        "K": K,
        "dist": dist
    }

def save_yaml(out_path, data):
    try:
        import yaml
        payload = {
            "image_size": [int(data["image_size"][0]), int(data["image_size"][1])],
            "rms": data["rms"],
            "rmse_px": data["rmse_px"],
            "K": data["K"].tolist(),
            "dist": data["dist"].tolist(),
        }
        with open(out_path, "w") as f:
            yaml.safe_dump(payload, f, sort_keys=False)
        print(f"[SAVE] {out_path}")
    except ImportError:
        fs = cv2.FileStorage(out_path, cv2.FILE_STORAGE_WRITE)
        fs.write("image_width", int(data["image_size"][0]))
        fs.write("image_height", int(data["image_size"][1]))
        fs.write("rms", float(data["rms"]))
        fs.write("rmse_px", float(data["rmse_px"]))
        fs.write("K", data["K"])
        fs.write("dist", data["dist"])
        fs.release()
        print(f"[SAVE] {out_path} (OpenCV FileStorage, install python3-yaml for YAML)")

def main():
    ap = argparse.ArgumentParser(description="Single-camera chessboard calibration (camera 0).")
    ap.add_argument("--camera", type=int, default=0, help="rpicam camera index (default 0)")
    ap.add_argument("--cols", type=int, default=9, help="inner corners cols (default 9)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners rows (default 6)")
    ap.add_argument("--square", type=float, default=0.025, help="square size in meters (default 0.025 = 25mm)")
    ap.add_argument("--out", default="calib_cam0.yaml", help="output file")
    ap.add_argument("--capture", type=int, default=25, help="number of images to capture (default 25)")
    ap.add_argument("--dir", default="calib_cam0_imgs", help="directory to store images")
    ap.add_argument("--w", type=int, default=1920, help="capture width")
    ap.add_argument("--h", type=int, default=1080, help="capture height")
    ap.add_argument("--use-existing", action="store_true", help="skip capture and use existing jpgs in --dir")
    ap.add_argument("--show", action="store_true", help="show detected corners briefly")
    args = ap.parse_args()

    if not args.use_existing:
        ok = capture_images_keypress(args.dir, args.capture, args.camera, args.w, args.h)
        if not ok:
            return

    images = sorted(glob.glob(os.path.join(args.dir, "*.jpg")))
    if not images:
        print("[ERROR] No images found. Capture failed or wrong directory.")
        return

    print(f"[CALIB] Using {len(images)} images. Pattern {args.cols}x{args.rows}, square={args.square}m")
    data = calibrate(images, args.cols, args.rows, args.square, show=args.show)

    print("\n=== RESULTS ===")
    print("Image size:", data["image_size"])
    print("RMS (opencv):", data["rms"])
    print("RMSE (px):", data["rmse_px"])
    print("K:\n", data["K"])
    print("dist:\n", data["dist"])

    save_yaml(args.out, data)

if __name__ == "__main__":
    main()