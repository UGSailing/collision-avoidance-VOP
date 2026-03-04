#!/usr/bin/env python3
import os
import time
import argparse

import cv2
import numpy as np

### This script captures images from a camera, applies undistortion using calibration data from a YAML file, and saves both raw and undistorted images. It also displays a side-by-side preview of the raw and undistorted frames.
### usage: python undistort_capture.py --calib calib_cam0.yaml --camera 0 --out captures --backend dshow
try:
    import yaml
except ImportError:
    raise SystemExit("Missing dependency: pyyaml. Install with: pip install pyyaml")


def load_calib_yaml(path: str):
    # Read raw bytes first (handles weird encodings / BOM)
    raw = open(path, "rb").read()

    # Try common encodings
    text = None
    for enc in ("utf-8-sig", "utf-16", "utf-16le", "utf-16be", "cp1252"):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    if text is None:
        raise SystemExit("Could not decode YAML file (unknown encoding). Save as UTF-8.")

    # ---- OpenCV FileStorage YAML cleanup ----
    # OpenCV often writes: %YAML:1.0  (not valid YAML for PyYAML)
    # Also sometimes has a leading '---'
    lines = text.splitlines()

    if lines and lines[0].strip().startswith("%YAML:"):
        lines = lines[1:]  # drop OpenCV header line
        # also drop optional leading '---'
        if lines and lines[0].strip() == "---":
            lines = lines[1:]

    cleaned = "\n".join(lines).strip()

    data = yaml.safe_load(cleaned)
    if not isinstance(data, dict):
        raise SystemExit("Parsed YAML but got no dict. File content is not what we expect.")

    # Expecting your structure
    w, h = data["image_size"]
    K = np.array(data["K"], dtype=np.float64)
    dist = np.array(data["dist"], dtype=np.float64).reshape(-1, 1)

    return (int(w), int(h)), K, dist


def build_undistort_maps(image_size, K, dist, alpha=0.0):
    w, h = image_size
    # alpha=0 => crop to valid pixels, alpha=1 => keep all pixels (black borders)
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist, R=None, newCameraMatrix=newK, size=(w, h), m1type=cv2.CV_16SC2
    )
    return newK, roi, map1, map2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default="calib_cam0.yaml", help="Path to calibration YAML")
    ap.add_argument("--camera", type=int, default=0, help="Camera index")
    ap.add_argument("--out", default="captures", help="Output folder")
    ap.add_argument("--alpha", type=float, default=0.0, help="0=crop, 1=keep all (black borders)")
    ap.add_argument("--backend", default="auto", choices=["auto", "dshow", "msmf"], help="Windows capture backend")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    (w, h), K, dist = load_calib_yaml(args.calib)
    newK, roi, map1, map2 = build_undistort_maps((w, h), K, dist, alpha=args.alpha)

    # Pick a backend (Windows sometimes gives weird colors with MSMF)
    if args.backend == "dshow":
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    elif args.backend == "msmf":
        cap = cv2.VideoCapture(args.camera, cv2.CAP_MSMF)
    else:
        cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise SystemExit(f"Could not open camera {args.camera}")

    # Force Full HD
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

    print("[INFO] Press 'c' to capture. Press 'q' to quit.")
    print(f"[INFO] Using {w}x{h}, alpha={args.alpha}, output='{args.out}'")

    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Frame grab failed")
            time.sleep(0.05)
            continue

        # Ensure size matches calibration
        if frame.shape[1] != w or frame.shape[0] != h:
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

        undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Optional crop to ROI when alpha=0 (cleaner view)
        if args.alpha == 0.0:
            x, y, rw, rh = roi
            undist_view = undist[y:y+rh, x:x+rw]
        else:
            undist_view = undist

        # Show side-by-side (raw | undist)
        # Make same height for display
        if undist_view.shape[0] != frame.shape[0]:
            disp_und = cv2.resize(undist_view, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_AREA)
        else:
            disp_und = undist_view

        disp = np.hstack([frame, disp_und])
        cv2.imshow("RAW | UNDISTORTED", disp)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("c"):
            ts = time.strftime("%Y%m%d_%H%M%S")
            raw_path = os.path.join(args.out, f"raw_{ts}_{idx:04d}.jpg")
            und_path = os.path.join(args.out, f"undist_{ts}_{idx:04d}.jpg")

            cv2.imwrite(raw_path, frame)
            cv2.imwrite(und_path, undist_view)
            print(f"[SAVE] {raw_path}")
            print(f"[SAVE] {und_path}")
            idx += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()