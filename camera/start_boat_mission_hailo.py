#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from datetime import datetime
from typing import Any

import numpy as np
import yaml

from gi.repository import Gst

import hailo
from hailo_apps_infra.hailo_rpi_common import app_callback_class
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

import can_comms
from depth_calculation.single_camera_depth_calculation import (
    distance_and_angle_from_bbox,
)


"""
python camera/start_boat_mission.py --backend webcam --webcam-left 0 --webcam-right -1 --model camera/yolo_models/duck.pt --single-camera-depth --object-height-m 0.175 --calib-yaml camera/calibration_yamls/camera_calibration.yaml
"""

"""
python camera/start_boat_mission.py --backend pi --camera-left 0 --camera-right 1 --model camera/yolo_models/duck.pt --single-camera-depth --object-height-m 0.175 --calib-yaml camera/calibration_yamls/camera_calibration.yaml
"""


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

# python camera/start_boat_mission.py --hailo-apps-root ./Documents/hailo-apps --network yolov8n --input rpi --save-output --camera-resolution fhd --frame-rate 5

def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run Hailo official object_detection app from this project."
    )

    parser.add_argument(
        "--hailo-apps-root",
        type=Path,
        required=True,
        help="Path to your local hailo-apps repo root",
    )
    parser.add_argument(
        "--network",
        type=str,
        default="yolov8n",
        help="Hailo model name (e.g. yolov8n) or local HEF path",
    )
    parser.add_argument(
        "--input",
        type=str,
        default="rpi",
        help="Input source for Hailo app: rpi, usb, /dev/video0, video.mp4, ...",
    )
    parser.add_argument(
        "--save-output",
        action="store_true",
        help="Save annotated output video/images",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "hailo_output",
        help="Where Hailo app should save output",
    )
    parser.add_argument(
        "--track",
        action="store_true",
        help="Enable Hailo tracking",
    )
    parser.add_argument(
        "--show-fps",
        action="store_true",
        help="Show FPS in Hailo app",
    )
    parser.add_argument(
        "--frame-rate",
        type=int,
        default=None,
        help="Camera frame rate override for Hailo app",
    )
    parser.add_argument(
        "--camera-resolution",
        type=str,
        choices=("sd", "hd", "fhd"),
        default="fhd",
        help="Camera resolution preset for Hailo app",
    )

    # Kept only so your CLI stays familiar.
    # These do NOT affect the official Hailo app yet.
    parser.add_argument(
        "--single-camera-depth",
        action="store_true",
        help="Reserved for later distance calculation integration",
    )
    parser.add_argument(
        "--object-height-m",
        type=float,
        default=0.175,
        help="Reserved for later distance calculation integration",
    )
    parser.add_argument(
        "--calib-yaml",
        type=Path,
        default=SCRIPT_DIR / "calibration_yamls" / "camera_calibration.yaml",
        help="Reserved for later distance calculation integration",
    )

    return parser


def main() -> int:
    args = create_parser().parse_args()

    hailo_root = args.hailo_apps_root.resolve()
    app_path = (
        hailo_root
        / "hailo_apps"
        / "python"
        / "standalone_apps"
        / "object_detection"
        / "object_detection.py"
    )

    if not hailo_root.exists():
        print(f"ERROR: hailo-apps root does not exist: {hailo_root}", file=sys.stderr)
        return 1

    if not app_path.exists():
        print(f"ERROR: object_detection.py not found at: {app_path}", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    cmd: list[str] = [
        sys.executable,
        str(app_path),
        "-n",
        args.network,
        "-i",
        args.input,
        "-o",
        str(args.output_dir),
        "--output-resolution",
        "1920",
        "1080",
        "--camera-resolution",
        args.camera_resolution,
    ]

    if args.save_output:
        cmd.append("--save-output")

    if args.track:
        cmd.append("--track")

    if args.show_fps:
        cmd.append("--show-fps")

    if args.frame_rate is not None:
        cmd.extend(["-f", str(args.frame_rate)])

    print("Running:")
    print(" ".join(cmd))

    completed = subprocess.run(cmd, cwd=str(hailo_root))
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())