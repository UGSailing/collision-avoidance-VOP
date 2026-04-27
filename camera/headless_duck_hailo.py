#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2
from picamera2 import Picamera2
from picamera2.devices import Hailo


def extract_detections(hailo_output, w, h, labels, threshold):
    results = []
    for class_id, detections in enumerate(hailo_output):
        for det in detections:
            score = float(det[4])
            if score < threshold:
                continue
            y0, x0, y1, x1 = det[:4]
            bbox = (
                int(x0 * w),
                int(y0 * h),
                int(x1 * w),
                int(y1 * h),
            )
            label = labels[class_id] if class_id < len(labels) else str(class_id)
            results.append((label, score, bbox))
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--labels", default=None)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    if args.labels:
        labels = Path(args.labels).read_text().splitlines()
    else:
        labels = ["duck"]

    with Hailo(str(args.model)) as hailo:
        model_h, model_w, _ = hailo.get_input_shape()

        with Picamera2(args.camera) as picam2:
            config = picam2.create_video_configuration(
                main={"size": (args.width, args.height), "format": "RGB888"},
                lores={"size": (model_w, model_h), "format": "RGB888"},
                controls={"FrameRate": args.fps},
            )
            picam2.configure(config)
            picam2.start()

            print("Running headless duck detection...")
            t0 = time.time()
            frames = 0

            while True:
                lores = picam2.capture_array("lores")
                output = hailo.run(lores)
                detections = extract_detections(
                    output,
                    args.width,
                    args.height,
                    labels,
                    args.threshold,
                )

                for label, score, bbox in detections:
                    print(f"{label} {score:.2f} bbox={bbox}", flush=True)

                frames += 1
                if frames % 30 == 0:
                    fps = frames / (time.time() - t0)
                    print(f"FPS: {fps:.2f}", flush=True)


if __name__ == "__main__":
    main()
