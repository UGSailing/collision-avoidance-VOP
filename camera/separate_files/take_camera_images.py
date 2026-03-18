#!/usr/bin/env python3
import os
import time
import argparse
import subprocess

# python take_images.py --out-dir images --count 10


def run(cmd, timeout=30):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout, r.stderr


def start_preview(camera_id: int, width: int, height: int):
    cmd = [
        "rpicam-hello",
        "-t",
        "0",
        "--camera",
        str(camera_id),
        "--width",
        str(width),
        "--height",
        str(height),
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


def main():
    ap = argparse.ArgumentParser(description="Preview + capture 10 images on keypress.")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--out-dir", default="captured_imgs")
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--w", type=int, default=1920)
    ap.add_argument("--h", type=int, default=1080)
    ap.add_argument("--settle-ms", type=int, default=200)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    print("[PREVIEW] Starting live preview...")
    preview = start_preview(args.camera, args.w, args.h)

    i = 0
    try:
        while i < args.count:
            key = (
                input(
                    f"[{i}/{args.count}] Press 'c' + Enter to capture, 'q' + Enter to quit: "
                )
                .strip()
                .lower()
            )

            if key == "q":
                print("[STOP] Quit by user.")
                break
            if key != "c":
                continue

            stop_preview(preview)
            time.sleep(0.2)

            fname = os.path.join(args.out_dir, f"img_{i:03d}.jpg")
            cmd = [
                "rpicam-still",
                "--camera",
                str(args.camera),
                "--output",
                fname,
                "--timeout",
                str(args.settle_ms),
                "--width",
                str(args.w),
                "--height",
                str(args.h),
                "--nopreview",
                "--denoise",
                "off",
                "--quality",
                "95",
            ]

            code, out, err = run(cmd, timeout=max(10, int(args.settle_ms / 1000) + 10))
            if code != 0 or not os.path.exists(fname):
                print("[ERROR] Capture failed:", (err.strip() or out.strip()))
            else:
                print(f"[OK] {fname}")
                i += 1

            preview = start_preview(args.camera, args.w, args.h)

    finally:
        stop_preview(preview)

    print(f"[DONE] Saved {i} image(s) to: {args.out_dir}")


if __name__ == "__main__":
    main()
