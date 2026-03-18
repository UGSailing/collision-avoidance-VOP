#!/usr/bin/env python3
import os
import time
import argparse
import subprocess

import cv2
import numpy as np
import yaml

### gebruikt YAML file, check of bestaat en juiste resolutie
###  - Press 'c' to capture a still with rpicam-still (AE/AWB settle time applies)
### python3 calibrations/calibration_filter.py --calib calibration_yamls/camera_calibration.yaml --camera 0 --out captures --alpha 0

### met bestaande map
### python3 calibrations/calibration_filter.py --calib calib_cam0.yaml --input-dir calib_good_calibration_imgs --out captures --alpha 0

### met PNG's:
### python3 Calibration_filter.py --calib calib_cam0.yaml --input-dir calib_good_calibration_imgs --glob "*.png" --out captures --alpha 0

def run(cmd, timeout=30):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout, r.stderr

def load_calib_yaml(path: str):
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    w, h = data["image_size"]
    K = np.array(data["K"], dtype=np.float64)
    dist = np.array(data["dist"], dtype=np.float64).reshape(-1, 1)

    return (int(w), int(h)), K, dist

def build_undistort_maps(image_size, K, dist, alpha=0.0):
    w, h = image_size
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist, R=None, newCameraMatrix=newK, size=(w, h), m1type=cv2.CV_16SC2
    )
    return newK, roi, map1, map2

def capture_one_rpicam(camera_id, out_path, w, h, settle_ms, extra_args):
    cmd = [
        "rpicam-still",
        "--camera", str(camera_id),
        "--output", out_path,
        "--timeout", str(settle_ms),
        "--width", str(w),
        "--height", str(h),
        "--nopreview",
        "--denoise", "off",
        "--quality", "95",
    ] + extra_args

    code, out, err = run(cmd, timeout=max(10, int(settle_ms/1000) + 10))
    if code != 0 or not os.path.exists(out_path):
        raise RuntimeError(err.strip() or out.strip() or "rpicam-still failed")

def start_preview(camera_id: int, width: int, height: int):
    cmd = [
        "rpicam-hello",
        "-t", "0",
        "--camera", str(camera_id),
        "--width", str(width),
        "--height", str(height),
        "--qt-preview",
    ]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def stop_preview(p):
    if p is None:
        return
    if p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait(timeout=2)

def process_existing_images(input_dir, pattern, out_dir, w, h, map1, map2, roi, alpha, show_undist_only=False):
    import glob

    paths = sorted(glob.glob(os.path.join(input_dir, pattern)))
    if not paths:
        print(f"[ERROR] No images found in {input_dir} matching {pattern}")
        return

    print(f"[INFO] Processing {len(paths)} existing images from '{input_dir}'")

    for idx, path in enumerate(paths):
        frame = cv2.imread(path)
        if frame is None:
            print(f"[WARN] Could not read: {path}")
            continue

        if frame.shape[1] != w or frame.shape[0] != h:
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

        undist = cv2.remap(
            frame, map1, map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )

        if alpha == 0.0:
            x, y, rw, rh = roi
            und_view = undist[y:y+rh, x:x+rw]
        else:
            und_view = undist

        base = os.path.splitext(os.path.basename(path))[0]
        raw_out = os.path.join(out_dir, f"{base}_raw.jpg")
        und_out = os.path.join(out_dir, f"{base}_undist.jpg")

        cv2.imwrite(raw_out, frame)
        cv2.imwrite(und_out, und_view)
        print(f"[SAVE] {raw_out}")
        print(f"[SAVE] {und_out}")

        if show_undist_only:
            disp = und_view
        else:
            disp_und = und_view
            if disp_und.shape[0] != frame.shape[0]:
                disp_und = cv2.resize(
                    disp_und,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_AREA
                )
            disp = np.hstack([frame, disp_und])

        # cv2.imshow("RAW | UNDISTORTED", disp)
        # k = cv2.waitKey(300) & 0xFF
        # if k == ord("q"):
        #     print("[INFO] Stopped by user.")
        #     break
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default="calib_cam0.yaml", help="Path to calibration YAML")
    ap.add_argument("--camera", type=int, default=0, help="Camera index for rpicam-still")
    ap.add_argument("--out", default="captures", help="Output folder")
    ap.add_argument("--alpha", type=float, default=0.0, help="0=crop, 1=keep all (black borders)")
    ap.add_argument("--settle-ms", type=int, default=1500, help="AE/AWB settle time for each still")
    ap.add_argument("--w", type=int, default=0, help="Override width (0 = use calib width)")
    ap.add_argument("--h", type=int, default=0, help="Override height (0 = use calib height)")
    ap.add_argument("--show-undist-only", action="store_true", help="Show only undistorted view")
    ap.add_argument("--extra", default="", help="Extra rpicam-still args as one string (advanced)")
    ap.add_argument("--input-dir", default="", help="Process existing images from this folder instead of live capture")
    ap.add_argument("--glob", default="*.jpg", help="Filename pattern for --input-dir")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    (cw, ch), K, dist = load_calib_yaml(args.calib)
    w = args.w if args.w > 0 else cw
    h = args.h if args.h > 0 else ch

    _, roi, map1, map2 = build_undistort_maps((w, h), K, dist, alpha=args.alpha)
    if args.input_dir:
        os.makedirs(args.out, exist_ok=True)
        cv2.namedWindow("RAW | UNDISTORTED", cv2.WINDOW_NORMAL)
        process_existing_images(
            args.input_dir,
            args.glob,
            args.out,
            w,
            h,
            map1,
            map2,
            roi,
            args.alpha,
            show_undist_only=args.show_undist_only
        )
        cv2.destroyAllWindows()
        return

    extra_args = args.extra.split() if args.extra.strip() else []

    print("[INFO] Window controls: 'c' = capture, 'q' = quit")
    print(f"[INFO] rpicam camera={args.camera}, size={w}x{h}, alpha={args.alpha}, out='{args.out}'")
    preview = start_preview(args.camera, w, h)

    cv2.namedWindow("RAW | UNDISTORTED", cv2.WINDOW_NORMAL)

    idx = 0
    last_disp = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(last_disp, "Press 'c' to capture (rpicam-still)", (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    while True:
        cv2.imshow("RAW | UNDISTORTED", last_disp)
        k = cv2.waitKey(30) & 0xFF

        if k == ord("q"):
            break

        if k == ord("c"):
            ts = time.strftime("%Y%m%d_%H%M%S")
            raw_path = os.path.join(args.out, f"raw_{ts}_{idx:04d}.jpg")
            und_path = os.path.join(args.out, f"undist_{ts}_{idx:04d}.jpg")

            stop_preview(preview)
            time.sleep(0.2)  # camera release

            try:
                capture_one_rpicam(args.camera, raw_path, w, h, args.settle_ms, extra_args)
            except Exception as e:
                print("[ERROR] capture failed:", e)
                preview = start_preview(args.camera, w, h)
                continue

            preview = start_preview(args.camera, w, h)

            frame = cv2.imread(raw_path)
            if frame is None:
                print("[ERROR] cv2.imread failed on captured file")
                continue

            # Ensure expected size (rpicam should already match, but just in case)
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

            undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            if args.alpha == 0.0:
                x, y, rw, rh = roi
                und_view = undist[y:y+rh, x:x+rw]
            else:
                und_view = undist

            cv2.imwrite(und_path, und_view)
            print(f"[SAVE] {raw_path}")
            print(f"[SAVE] {und_path}")

            if args.show_undist_only:
                disp = und_view
            else:
                disp_und = und_view
                if disp_und.shape[0] != frame.shape[0]:
                    disp_und = cv2.resize(disp_und, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_AREA)
                disp = np.hstack([frame, disp_und])

            last_disp = disp
            idx += 1
    stop_preview(preview)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()